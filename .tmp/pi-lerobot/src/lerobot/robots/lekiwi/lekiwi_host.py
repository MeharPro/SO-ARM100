#!/usr/bin/env python

# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import base64
import json
import logging
import runpy
import time
from dataclasses import dataclass, field
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import cv2
import draccus
import zmq

from .config_lekiwi import LeKiwiConfig, LeKiwiHostConfig
from .lekiwi import LeKiwi

logging.basicConfig(level=logging.WARNING)
logger = logging.getLogger(__name__)


def _load_power_helper() -> tuple[type[Any], Path]:
    for parent in Path(__file__).resolve().parents:
        helper_path = parent / "scripts" / "lekiwi_power.py"
        if helper_path.exists():
            helper_ns = runpy.run_path(str(helper_path))
            return helper_ns["PowerTelemetryLogger"], helper_path

    raise RuntimeError("Power telemetry helper is unavailable.")


PowerTelemetryLogger, POWER_HELPER_PATH = _load_power_helper()

DEFAULT_ARM_TORQUE_LIMITS = {
    "arm_shoulder_pan": 500,
    "arm_shoulder_lift": 650,
    "arm_elbow_flex": 600,
    "arm_wrist_flex": 450,
    "arm_wrist_roll": 250,
    "arm_gripper": 1000,
}
DEFAULT_ARM_MAX_STEP = {
    "arm_shoulder_pan": 8.0,
    "arm_shoulder_lift": 8.0,
    "arm_elbow_flex": 8.0,
    "arm_wrist_flex": 10.0,
    "arm_wrist_roll": 12.0,
    "arm_gripper": 2.0,
}
DEFAULT_ARM_CURRENT_SOFT_LIMIT_MA = {
    "arm_shoulder_pan": 900.0,
    "arm_shoulder_lift": 950.0,
    "arm_elbow_flex": 950.0,
    "arm_wrist_flex": 700.0,
    "arm_wrist_roll": 650.0,
    "arm_gripper": 1200.0,
}
SAFE_ABSOLUTE_POSITION_LIMITS = {
    "arm_gripper": (0.0, 85.0),
}
DEFAULT_TOTAL_POWER_SOFT_LIMIT_W = 45.0
DEFAULT_BATTERY_LOW_PERCENT = 35.0
DEFAULT_BATTERY_CRITICAL_PERCENT = 15.0
DEFAULT_MIN_BATTERY_SCALE = 0.6
DEFAULT_MIN_POWER_SCALE = 0.45
DEFAULT_MIN_SERVO_SCALE = 0.35


@dataclass
class LeKiwiServerConfig:
    """Configuration for the LeKiwi host script."""

    robot: LeKiwiConfig = field(default_factory=LeKiwiConfig)
    host: LeKiwiHostConfig = field(default_factory=LeKiwiHostConfig)


def _power_args() -> SimpleNamespace:
    return SimpleNamespace(
        disable_power_monitor=False,
        power_sample_hz=2.0,
        power_print_every_s=1.0,
        power_log_dir="~/lekiwi-power-logs",
        power_num_retry=1,
        power_voltage_v_per_raw=0.1,
        power_current_ma_per_raw=6.5,
        battery_empty_v=9.6,
        battery_full_v=12.6,
        battery_smoothing_alpha=0.2,
    )


def apply_arm_torque_limits(robot: LeKiwi) -> None:
    motor_names = list(DEFAULT_ARM_TORQUE_LIMITS)
    torque_disabled = False
    try:
        robot.bus.disable_torque(motor_names, num_retry=5)
        torque_disabled = True
        for motor, value in DEFAULT_ARM_TORQUE_LIMITS.items():
            robot.bus.write("Torque_Limit", motor, value, normalize=False, num_retry=5)
            print(f"{motor}: Torque_Limit={value}", flush=True)
    except Exception as exc:
        logger.warning("Failed to apply arm torque limits: %s", exc)
    finally:
        if torque_disabled:
            try:
                robot.bus.enable_torque(motor_names, num_retry=5)
            except Exception as exc:
                logger.warning("Failed to re-enable torque after arm torque setup: %s", exc)


class PowerAwareActionLimiter:
    def __init__(self) -> None:
        self.last_targets: dict[str, float] = {}
        self.last_scales = {
            "battery": 1.0,
            "power": 1.0,
        }

    def seed_from_observation(self, observation: dict[str, Any]) -> None:
        for motor in DEFAULT_ARM_MAX_STEP:
            key = f"{motor}.pos"
            if key in observation and isinstance(observation[key], (int, float)):
                self.last_targets[key] = float(observation[key])

    def update(self, action: dict[str, Any]) -> None:
        for key, value in action.items():
            if key.endswith(".pos") and isinstance(value, (int, float)):
                self.last_targets[key] = float(value)

    @staticmethod
    def _sample_float(sample: dict[str, Any] | None, key: str) -> float | None:
        if not sample:
            return None

        value = sample.get(key)
        if value in ("", None):
            return None

        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    def _battery_scale(self, sample: dict[str, Any] | None) -> float:
        percent = self._sample_float(sample, "battery_percent_smoothed_est")
        if percent is None:
            percent = self._sample_float(sample, "battery_percent_est")
        if percent is None:
            return 1.0
        if percent >= DEFAULT_BATTERY_LOW_PERCENT:
            return 1.0
        if percent <= DEFAULT_BATTERY_CRITICAL_PERCENT:
            return DEFAULT_MIN_BATTERY_SCALE

        span = max(DEFAULT_BATTERY_LOW_PERCENT - DEFAULT_BATTERY_CRITICAL_PERCENT, 1e-6)
        ratio = (percent - DEFAULT_BATTERY_CRITICAL_PERCENT) / span
        return DEFAULT_MIN_BATTERY_SCALE + ratio * (1.0 - DEFAULT_MIN_BATTERY_SCALE)

    def _power_scale(self, sample: dict[str, Any] | None) -> float:
        total_power_w = self._sample_float(sample, "total_power_w_est")
        if total_power_w is None or total_power_w <= DEFAULT_TOTAL_POWER_SOFT_LIMIT_W:
            return 1.0
        return max(DEFAULT_MIN_POWER_SCALE, DEFAULT_TOTAL_POWER_SOFT_LIMIT_W / max(total_power_w, 1.0))

    def _servo_scale(self, motor: str, sample: dict[str, Any] | None) -> float:
        current_ma = self._sample_float(sample, f"{motor}.current_ma_est")
        soft_limit_ma = DEFAULT_ARM_CURRENT_SOFT_LIMIT_MA.get(motor)
        if current_ma is None or soft_limit_ma is None or current_ma <= soft_limit_ma:
            return 1.0
        return max(DEFAULT_MIN_SERVO_SCALE, soft_limit_ma / max(current_ma, 1.0))

    def limit(self, action: dict[str, Any], sample: dict[str, Any] | None) -> dict[str, Any]:
        limited = dict(action)
        battery_scale = self._battery_scale(sample)
        power_scale = self._power_scale(sample)
        self.last_scales = {
            "battery": battery_scale,
            "power": power_scale,
        }

        shared_arm_scale = min(battery_scale, power_scale)

        for motor, base_step in DEFAULT_ARM_MAX_STEP.items():
            key = f"{motor}.pos"
            if key not in limited:
                continue

            value = float(limited[key])
            if motor in SAFE_ABSOLUTE_POSITION_LIMITS:
                low, high = SAFE_ABSOLUTE_POSITION_LIMITS[motor]
                value = min(max(value, low), high)

            prev = self.last_targets.get(key)
            if prev is None:
                limited[key] = value
                continue

            servo_scale = self._servo_scale(motor, sample)
            max_step = max(0.25, base_step * shared_arm_scale * servo_scale)
            limited[key] = min(max(value, prev - max_step), prev + max_step)

        return limited


def attach_power_summary(
    observation: dict[str, Any],
    sample: dict[str, Any] | None,
    limiter: PowerAwareActionLimiter,
) -> dict[str, Any]:
    if not sample:
        return observation

    enriched = dict(observation)
    enriched["telemetry.battery_percent_est"] = sample.get("battery_percent_smoothed_est", "")
    enriched["telemetry.battery_voltage_v_est"] = sample.get("battery_voltage_v_est", "")
    enriched["telemetry.total_current_ma_est"] = sample.get("total_current_ma_est", "")
    enriched["telemetry.total_power_w_est"] = sample.get("total_power_w_est", "")
    enriched["telemetry.peak_current_motor"] = sample.get("peak_current_motor", "")
    enriched["telemetry.peak_current_ma_est"] = sample.get("peak_current_ma_est", "")
    enriched["telemetry.hottest_motor"] = sample.get("hottest_motor", "")
    enriched["telemetry.max_temp_c"] = sample.get("max_temp_c", "")
    enriched["telemetry.limit.battery_scale"] = round(limiter.last_scales["battery"], 3)
    enriched["telemetry.limit.power_scale"] = round(limiter.last_scales["power"], 3)
    return enriched


def _shutdown_robot_safely(robot: LeKiwi) -> None:
    try:
        if robot.bus.is_connected:
            try:
                robot.stop_base()
            except Exception:
                logging.exception("Failed to stop base during shutdown")

            try:
                robot.bus.disconnect(robot.config.disable_torque_on_disconnect)
            except Exception:
                logging.exception("Primary bus disconnect failed; attempting emergency torque disable")
                try:
                    robot.bus.disable_torque(num_retry=10)
                except Exception:
                    logging.exception("Emergency torque disable failed")
                try:
                    robot.bus.disconnect(disable_torque=False)
                except Exception:
                    logging.exception("Final bus close failed")

        for cam_name, cam in robot.cameras.items():
            try:
                if cam.is_connected:
                    cam.disconnect()
            except Exception:
                logging.exception("Failed to disconnect camera %s", cam_name)
    except Exception:
        logging.exception("Unexpected error during LeKiwi shutdown")


class LeKiwiHost:
    def __init__(self, config: LeKiwiHostConfig):
        self.zmq_context = zmq.Context()
        self.zmq_cmd_socket = self.zmq_context.socket(zmq.PULL)
        self.zmq_cmd_socket.setsockopt(zmq.CONFLATE, 1)
        self.zmq_cmd_socket.bind(f"tcp://*:{config.port_zmq_cmd}")

        self.zmq_observation_socket = self.zmq_context.socket(zmq.PUSH)
        self.zmq_observation_socket.setsockopt(zmq.CONFLATE, 1)
        self.zmq_observation_socket.bind(f"tcp://*:{config.port_zmq_observations}")

        self.connection_time_s = config.connection_time_s
        self.watchdog_timeout_ms = config.watchdog_timeout_ms
        self.max_loop_freq_hz = config.max_loop_freq_hz

    def disconnect(self):
        self.zmq_observation_socket.close()
        self.zmq_cmd_socket.close()
        self.zmq_context.term()


@draccus.wrap()
def main(cfg: LeKiwiServerConfig):
    logging.info("Configuring LeKiwi")
    robot = LeKiwi(cfg.robot)

    logging.info("Connecting LeKiwi")
    robot.connect()
    apply_arm_torque_limits(robot)

    power_logger = PowerTelemetryLogger(robot, _power_args(), logger, "lekiwi_host")
    power_logger.start()
    power_logger.maybe_sample(force=True)

    action_limiter = PowerAwareActionLimiter()
    try:
        action_limiter.seed_from_observation(robot.get_observation())
    except Exception as exc:
        logger.warning("Failed to seed power-aware action limiter: %s", exc)

    logging.info("Starting HostAgent")
    host = LeKiwiHost(cfg.host)

    last_cmd_time = time.time()
    watchdog_active = False
    no_command_logged = False
    logging.info("Waiting for commands...")
    try:
        start = time.perf_counter()
        duration = 0.0
        while duration < host.connection_time_s:
            loop_start_time = time.perf_counter()
            power_logger.maybe_sample()
            sample = power_logger.last_sample

            try:
                msg = host.zmq_cmd_socket.recv_string(zmq.NOBLOCK)
                data = dict(json.loads(msg))
                limited_action = action_limiter.limit(data, sample)
                action_sent = robot.send_action(limited_action)
                action_limiter.update(action_sent)
                last_cmd_time = time.time()
                watchdog_active = False
                no_command_logged = False
            except zmq.Again:
                if not no_command_logged:
                    logging.warning("No command available")
                    no_command_logged = True
            except Exception as exc:
                logging.error("Message fetching failed: %s", exc)

            now = time.time()
            if (now - last_cmd_time > host.watchdog_timeout_ms / 1000) and not watchdog_active:
                logging.warning(
                    "Command not received for more than %s milliseconds. Stopping the base.",
                    host.watchdog_timeout_ms,
                )
                watchdog_active = True
                robot.stop_base()

            last_observation = robot.get_observation()
            last_observation = attach_power_summary(last_observation, sample, action_limiter)

            for cam_key in robot.cameras:
                ret, buffer = cv2.imencode(
                    ".jpg", last_observation[cam_key], [int(cv2.IMWRITE_JPEG_QUALITY), 90]
                )
                if ret:
                    last_observation[cam_key] = base64.b64encode(buffer).decode("utf-8")
                else:
                    last_observation[cam_key] = ""

            try:
                host.zmq_observation_socket.send_string(json.dumps(last_observation), flags=zmq.NOBLOCK)
            except zmq.Again:
                logging.info("Dropping observation, no client connected")

            elapsed = time.perf_counter() - loop_start_time
            time.sleep(max(1 / host.max_loop_freq_hz - elapsed, 0))
            duration = time.perf_counter() - start
        print("Cycle time reached.")

    except KeyboardInterrupt:
        print("Keyboard interrupt received. Exiting...")
    finally:
        power_logger.maybe_sample(force=True)
        print("Shutting down Lekiwi Host.")
        _shutdown_robot_safely(robot)
        power_logger.close()
        try:
            host.disconnect()
        except Exception:
            logging.exception("Failed to disconnect LeKiwi host sockets")

    logging.info("Finished LeKiwi cleanly")


if __name__ == "__main__":
    main()
