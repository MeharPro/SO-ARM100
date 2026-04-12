#!/usr/bin/env python3

from __future__ import annotations

import argparse
import base64
import json
import logging
import time
from pathlib import Path
from typing import Any

import cv2
import zmq

from lekiwi_power import PowerTelemetryLogger, add_power_monitor_args
from lerobot.robots.lekiwi import LeKiwi, LeKiwiConfig

logging.basicConfig(level=logging.WARNING)
logger = logging.getLogger("lekiwi_sts_power_host")

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
DEFAULT_BASE_CURRENT_SOFT_LIMIT_MA = {
    "base_left_wheel": 1500.0,
    "base_back_wheel": 1500.0,
    "base_right_wheel": 1500.0,
}
DEFAULT_BASE_MAX_DELTA = {
    "x.vel": 0.08,
    "y.vel": 0.08,
    "theta.vel": 30.0,
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


def default_robot_port() -> str:
    by_id_dir = Path("/dev/serial/by-id")
    if by_id_dir.exists():
        for pattern in ("usb-1a86_USB_Single_Serial_*", "*USB_Single_Serial*", "*"):
            matches = sorted(str(path) for path in by_id_dir.glob(pattern))
            if matches:
                return matches[0]

    for pattern in ("ttyACM*", "ttyUSB*"):
        matches = sorted(str(path) for path in Path("/dev").glob(pattern))
        if matches:
            return matches[0]

    return "/dev/ttyACM0"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="LeKiwi host for STS3215 motors with power-aware motion limiting and battery telemetry."
    )
    parser.add_argument("--robot-id", default="follow-mobile")
    parser.add_argument("--robot-port", default=default_robot_port())
    parser.add_argument("--disable-cameras", action="store_true")
    parser.add_argument("--port-zmq-cmd", type=int, default=5555)
    parser.add_argument("--port-zmq-observations", type=int, default=5556)
    parser.add_argument("--connection-time-s", type=int, default=3600)
    parser.add_argument("--watchdog-timeout-ms", type=int, default=500)
    parser.add_argument("--loop-hz", type=float, default=30.0)
    parser.add_argument("--base-max-raw-velocity", type=int, default=6000)
    parser.add_argument("--base-wheel-torque-limit", type=int, default=700)
    parser.add_argument("--disable-arm-torque-limit-setup", action="store_true")
    parser.add_argument("--shoulder-pan-torque-limit", type=int, default=DEFAULT_ARM_TORQUE_LIMITS["arm_shoulder_pan"])
    parser.add_argument(
        "--shoulder-lift-torque-limit",
        type=int,
        default=DEFAULT_ARM_TORQUE_LIMITS["arm_shoulder_lift"],
    )
    parser.add_argument("--elbow-torque-limit", type=int, default=DEFAULT_ARM_TORQUE_LIMITS["arm_elbow_flex"])
    parser.add_argument("--wrist-flex-torque-limit", type=int, default=DEFAULT_ARM_TORQUE_LIMITS["arm_wrist_flex"])
    parser.add_argument("--wrist-roll-torque-limit", type=int, default=DEFAULT_ARM_TORQUE_LIMITS["arm_wrist_roll"])
    parser.add_argument("--gripper-torque-limit", type=int, default=DEFAULT_ARM_TORQUE_LIMITS["arm_gripper"])
    parser.add_argument("--shoulder-max-step", type=float, default=DEFAULT_ARM_MAX_STEP["arm_shoulder_pan"])
    parser.add_argument("--elbow-max-step", type=float, default=DEFAULT_ARM_MAX_STEP["arm_elbow_flex"])
    parser.add_argument("--wrist-max-step", type=float, default=DEFAULT_ARM_MAX_STEP["arm_wrist_flex"])
    parser.add_argument("--wrist-roll-max-step", type=float, default=DEFAULT_ARM_MAX_STEP["arm_wrist_roll"])
    parser.add_argument("--gripper-max-step", type=float, default=DEFAULT_ARM_MAX_STEP["arm_gripper"])
    parser.add_argument(
        "--arm-current-soft-limit-ma",
        type=float,
        default=DEFAULT_ARM_CURRENT_SOFT_LIMIT_MA["arm_shoulder_pan"],
    )
    parser.add_argument(
        "--wrist-current-soft-limit-ma",
        type=float,
        default=DEFAULT_ARM_CURRENT_SOFT_LIMIT_MA["arm_wrist_flex"],
    )
    parser.add_argument(
        "--wrist-roll-current-soft-limit-ma",
        type=float,
        default=DEFAULT_ARM_CURRENT_SOFT_LIMIT_MA["arm_wrist_roll"],
    )
    parser.add_argument(
        "--gripper-current-soft-limit-ma",
        type=float,
        default=DEFAULT_ARM_CURRENT_SOFT_LIMIT_MA["arm_gripper"],
    )
    parser.add_argument("--base-current-soft-limit-ma", type=float, default=DEFAULT_BASE_CURRENT_SOFT_LIMIT_MA["base_left_wheel"])
    parser.add_argument("--total-power-soft-limit-w", type=float, default=DEFAULT_TOTAL_POWER_SOFT_LIMIT_W)
    parser.add_argument("--battery-low-percent", type=float, default=DEFAULT_BATTERY_LOW_PERCENT)
    parser.add_argument("--battery-critical-percent", type=float, default=DEFAULT_BATTERY_CRITICAL_PERCENT)
    parser.add_argument("--min-battery-scale", type=float, default=DEFAULT_MIN_BATTERY_SCALE)
    parser.add_argument("--min-power-scale", type=float, default=DEFAULT_MIN_POWER_SCALE)
    parser.add_argument("--min-servo-scale", type=float, default=DEFAULT_MIN_SERVO_SCALE)
    add_power_monitor_args(parser)
    return parser.parse_args()


def build_arm_torque_limits(args: argparse.Namespace) -> dict[str, int]:
    return {
        "arm_shoulder_pan": args.shoulder_pan_torque_limit,
        "arm_shoulder_lift": args.shoulder_lift_torque_limit,
        "arm_elbow_flex": args.elbow_torque_limit,
        "arm_wrist_flex": args.wrist_flex_torque_limit,
        "arm_wrist_roll": args.wrist_roll_torque_limit,
        "arm_gripper": args.gripper_torque_limit,
    }


def apply_arm_torque_limits(robot: LeKiwi, limits: dict[str, int]) -> None:
    motor_names = list(limits)
    torque_disabled = False
    try:
        robot.bus.disable_torque(motor_names, num_retry=5)
        torque_disabled = True
        for motor, value in limits.items():
            robot.bus.write("Torque_Limit", motor, value, normalize=False, num_retry=5)
            print(f"{motor}: Torque_Limit={value}", flush=True)
    finally:
        if torque_disabled:
            robot.bus.enable_torque(motor_names, num_retry=5)


def encode_observation_images(robot: LeKiwi, observation: dict[str, Any]) -> dict[str, Any]:
    encoded = dict(observation)
    for cam_key in robot.cameras:
        frame = observation.get(cam_key)
        if frame is None:
            encoded[cam_key] = ""
            continue
        ret, buffer = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        encoded[cam_key] = base64.b64encode(buffer).decode("utf-8") if ret else ""
    return encoded


class PowerAwareActionLimiter:
    def __init__(self, args: argparse.Namespace) -> None:
        self.last_targets: dict[str, float] = {}
        self.arm_max_step = {
            "arm_shoulder_pan": args.shoulder_max_step,
            "arm_shoulder_lift": args.shoulder_max_step,
            "arm_elbow_flex": args.elbow_max_step,
            "arm_wrist_flex": args.wrist_max_step,
            "arm_wrist_roll": args.wrist_roll_max_step,
            "arm_gripper": args.gripper_max_step,
        }
        self.arm_current_soft_limit_ma = {
            "arm_shoulder_pan": args.arm_current_soft_limit_ma,
            "arm_shoulder_lift": args.arm_current_soft_limit_ma,
            "arm_elbow_flex": args.arm_current_soft_limit_ma,
            "arm_wrist_flex": args.wrist_current_soft_limit_ma,
            "arm_wrist_roll": args.wrist_roll_current_soft_limit_ma,
            "arm_gripper": args.gripper_current_soft_limit_ma,
        }
        self.base_current_soft_limit_ma = {
            "base_left_wheel": args.base_current_soft_limit_ma,
            "base_back_wheel": args.base_current_soft_limit_ma,
            "base_right_wheel": args.base_current_soft_limit_ma,
        }
        self.total_power_soft_limit_w = max(args.total_power_soft_limit_w, 1.0)
        self.battery_low_percent = args.battery_low_percent
        self.battery_critical_percent = args.battery_critical_percent
        self.min_battery_scale = min(max(args.min_battery_scale, 0.05), 1.0)
        self.min_power_scale = min(max(args.min_power_scale, 0.05), 1.0)
        self.min_servo_scale = min(max(args.min_servo_scale, 0.05), 1.0)
        self.last_scales = {
            "battery": 1.0,
            "power": 1.0,
            "base_current": 1.0,
        }

    def seed_from_observation(self, observation: dict[str, Any]) -> None:
        for motor in self.arm_max_step:
            key = f"{motor}.pos"
            if key in observation and isinstance(observation[key], (int, float)):
                self.last_targets[key] = float(observation[key])
        for key in DEFAULT_BASE_MAX_DELTA:
            if key in observation and isinstance(observation[key], (int, float)):
                self.last_targets[key] = float(observation[key])

    def reset_base(self) -> None:
        for key in DEFAULT_BASE_MAX_DELTA:
            self.last_targets.pop(key, None)

    def update(self, action: dict[str, Any]) -> None:
        for key, value in action.items():
            if isinstance(value, (int, float)):
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
        if percent >= self.battery_low_percent:
            return 1.0
        if percent <= self.battery_critical_percent:
            return self.min_battery_scale
        span = max(self.battery_low_percent - self.battery_critical_percent, 1e-6)
        ratio = (percent - self.battery_critical_percent) / span
        return self.min_battery_scale + ratio * (1.0 - self.min_battery_scale)

    def _power_scale(self, sample: dict[str, Any] | None) -> float:
        total_power_w = self._sample_float(sample, "total_power_w_est")
        if total_power_w is None or total_power_w <= self.total_power_soft_limit_w:
            return 1.0
        return max(self.min_power_scale, self.total_power_soft_limit_w / max(total_power_w, 1.0))

    def _servo_scale(self, motor: str, sample: dict[str, Any] | None) -> float:
        current_ma = self._sample_float(sample, f"{motor}.current_ma_est")
        soft_limit_ma = self.arm_current_soft_limit_ma.get(motor)
        if current_ma is None or soft_limit_ma is None or current_ma <= soft_limit_ma:
            return 1.0
        return max(self.min_servo_scale, soft_limit_ma / max(current_ma, 1.0))

    def _base_current_scale(self, sample: dict[str, Any] | None) -> float:
        scale = 1.0
        for motor, soft_limit_ma in self.base_current_soft_limit_ma.items():
            current_ma = self._sample_float(sample, f"{motor}.current_ma_est")
            if current_ma is None or current_ma <= soft_limit_ma:
                continue
            scale = min(scale, max(self.min_power_scale, soft_limit_ma / max(current_ma, 1.0)))
        return scale

    def limit(self, action: dict[str, Any], sample: dict[str, Any] | None) -> dict[str, Any]:
        limited = dict(action)
        battery_scale = self._battery_scale(sample)
        power_scale = self._power_scale(sample)
        base_current_scale = self._base_current_scale(sample)
        self.last_scales = {
            "battery": battery_scale,
            "power": power_scale,
            "base_current": base_current_scale,
        }

        shared_arm_scale = min(battery_scale, power_scale)
        shared_base_scale = min(battery_scale, power_scale, base_current_scale)

        for motor, base_step in self.arm_max_step.items():
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

        for key, max_delta in DEFAULT_BASE_MAX_DELTA.items():
            if key not in limited:
                continue
            value = float(limited[key]) * shared_base_scale
            prev = self.last_targets.get(key)
            if prev is not None:
                delta = max(0.01, max_delta * shared_base_scale)
                value = min(max(value, prev - delta), prev + delta)
            limited[key] = value

        return limited


def attach_power_telemetry(
    observation: dict[str, Any],
    sample: dict[str, Any] | None,
    limiter: PowerAwareActionLimiter,
    motor_names: list[str],
) -> dict[str, Any]:
    if not sample:
        return observation

    enriched = dict(observation)
    summary_keys = {
        "telemetry.battery_percent_est": sample.get("battery_percent_smoothed_est", ""),
        "telemetry.battery_voltage_v_est": sample.get("battery_voltage_v_est", ""),
        "telemetry.total_current_ma_est": sample.get("total_current_ma_est", ""),
        "telemetry.total_power_w_est": sample.get("total_power_w_est", ""),
        "telemetry.peak_current_motor": sample.get("peak_current_motor", ""),
        "telemetry.peak_current_ma_est": sample.get("peak_current_ma_est", ""),
        "telemetry.hottest_motor": sample.get("hottest_motor", ""),
        "telemetry.max_temp_c": sample.get("max_temp_c", ""),
        "telemetry.limit.battery_scale": round(limiter.last_scales["battery"], 3),
        "telemetry.limit.power_scale": round(limiter.last_scales["power"], 3),
        "telemetry.limit.base_current_scale": round(limiter.last_scales["base_current"], 3),
    }
    enriched.update(summary_keys)

    for motor in motor_names:
        current_key = f"{motor}.current_ma_est"
        power_key = f"{motor}.power_w_est"
        temp_key = f"{motor}.temperature_c"
        current_value = sample.get(current_key, "")
        power_value = sample.get(power_key, "")
        temp_value = sample.get(temp_key, "")
        if current_value != "":
            enriched[f"telemetry.{current_key}"] = current_value
        if power_value != "":
            enriched[f"telemetry.{power_key}"] = power_value
        if temp_value != "":
            enriched[f"telemetry.{temp_key}"] = temp_value

    return enriched


def main() -> None:
    args = parse_args()

    robot_config = LeKiwiConfig(
        id=args.robot_id,
        port=args.robot_port,
        base_max_raw_velocity=args.base_max_raw_velocity,
        base_wheel_torque_limit=args.base_wheel_torque_limit,
    )
    if args.disable_cameras:
        robot_config.cameras = {}

    robot = LeKiwi(robot_config)
    robot.connect(calibrate=False)

    if not args.disable_arm_torque_limit_setup:
        apply_arm_torque_limits(robot, build_arm_torque_limits(args))

    action_limiter = PowerAwareActionLimiter(args)
    try:
        action_limiter.seed_from_observation(robot.get_observation())
    except Exception as exc:
        logger.warning("Failed to seed action limiter from the current robot state: %s", exc)

    power_logger = PowerTelemetryLogger(robot, args, logger, "lekiwi_sts_power_host")
    power_logger.start()
    power_logger.maybe_sample(force=True)

    ctx = zmq.Context()
    cmd_socket = ctx.socket(zmq.PULL)
    cmd_socket.setsockopt(zmq.CONFLATE, 1)
    cmd_socket.bind(f"tcp://*:{args.port_zmq_cmd}")

    obs_socket = ctx.socket(zmq.PUSH)
    obs_socket.setsockopt(zmq.CONFLATE, 1)
    obs_socket.bind(f"tcp://*:{args.port_zmq_observations}")

    last_cmd_time = time.time()
    watchdog_active = False
    no_command_logged = False

    print(
        "STS power host listening on "
        f"tcp://*:{args.port_zmq_cmd} and tcp://*:{args.port_zmq_observations}",
        flush=True,
    )

    start = time.perf_counter()
    try:
        while time.perf_counter() - start < args.connection_time_s:
            loop_start = time.perf_counter()
            power_logger.maybe_sample()
            sample = power_logger.last_sample

            try:
                msg = cmd_socket.recv_string(flags=zmq.NOBLOCK)
                incoming_action = json.loads(msg)
                limited_action = action_limiter.limit(incoming_action, sample)
                sent_action = robot.send_action(limited_action)
                action_limiter.update(sent_action)
                last_cmd_time = time.time()
                watchdog_active = False
                no_command_logged = False
            except zmq.Again:
                if not no_command_logged:
                    logger.warning("No command available")
                    no_command_logged = True
            except Exception as exc:
                logger.warning("Command receive/send failed: %s", exc)

            now = time.time()
            if (now - last_cmd_time > args.watchdog_timeout_ms / 1000.0) and not watchdog_active:
                logger.warning(
                    "Command not received for more than %s milliseconds. Stopping the base.",
                    args.watchdog_timeout_ms,
                )
                try:
                    robot.stop_base()
                except Exception as exc:
                    logger.warning("stop_base failed: %s", exc)
                action_limiter.reset_base()
                watchdog_active = True

            try:
                observation = robot.get_observation()
                observation = attach_power_telemetry(observation, sample, action_limiter, list(robot.bus.motors))
                encoded = encode_observation_images(robot, observation)
                obs_socket.send_string(json.dumps(encoded), flags=zmq.NOBLOCK)
            except zmq.Again:
                pass
            except Exception as exc:
                logger.warning("Observation publish failed: %s", exc)

            elapsed = time.perf_counter() - loop_start
            time.sleep(max(1.0 / args.loop_hz - elapsed, 0.0))
    except KeyboardInterrupt:
        print("\nStopping STS power host", flush=True)
    finally:
        power_logger.maybe_sample(force=True)
        try:
            robot.stop_base()
        except Exception as exc:
            print(f"stop_base: {exc}", flush=True)
        try:
            robot.disconnect()
        except Exception as exc:
            print(f"disconnect: {exc}", flush=True)
        power_logger.close()
        cmd_socket.close()
        obs_socket.close()
        ctx.term()


if __name__ == "__main__":
    main()
