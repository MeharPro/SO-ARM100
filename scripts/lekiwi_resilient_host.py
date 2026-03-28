#!/usr/bin/env python3

import argparse
import base64
import json
import logging
import runpy
import time
from pathlib import Path
from typing import Any

import cv2
import zmq

from lerobot.robots.lekiwi import LeKiwi, LeKiwiConfig

logging.basicConfig(level=logging.WARNING)
logger = logging.getLogger("lekiwi_resilient_host")

if "PowerTelemetryLogger" not in globals():
    helper_path = Path(__file__).with_name("lekiwi_power.py")
    if helper_path.exists():
        _helper_ns = runpy.run_path(str(helper_path))
        PowerTelemetryLogger = _helper_ns["PowerTelemetryLogger"]
        add_power_monitor_args = _helper_ns["add_power_monitor_args"]
    else:
        raise RuntimeError("Power telemetry helper is unavailable.")

DEFAULT_MAX_RELATIVE_TARGET = {
    "arm_shoulder_pan": 8.0,
    "arm_shoulder_lift": 8.0,
    "arm_elbow_flex": 8.0,
    "arm_wrist_flex": 10.0,
    "arm_wrist_roll": 12.0,
}
DEFAULT_GRIPPER_MAX_STEP = 2.0
DEFAULT_GRIPPER_MAX_TORQUE_LIMIT = 400
DEFAULT_GRIPPER_TORQUE_LIMIT = 350
DEFAULT_GRIPPER_PROTECTION_CURRENT = 200
DEFAULT_GRIPPER_OVERLOAD_TORQUE = 20
SAFE_ABSOLUTE_POSITION_LIMITS = {
    "arm_gripper": (0.0, 85.0),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Resilient LeKiwi host with observation fallbacks.")
    parser.add_argument("--robot-id", default="follow")
    parser.add_argument("--robot-port", default="/dev/ttyACM0")
    parser.add_argument("--port-zmq-cmd", type=int, default=5555)
    parser.add_argument("--port-zmq-observations", type=int, default=5556)
    parser.add_argument("--connection-time-s", type=int, default=600)
    parser.add_argument("--watchdog-timeout-ms", type=int, default=500)
    parser.add_argument("--loop-hz", type=float, default=30.0)
    parser.add_argument("--read-retries", type=int, default=2)
    parser.add_argument("--disable-cameras", action="store_true")
    parser.add_argument("--gripper-max-step", type=float, default=DEFAULT_GRIPPER_MAX_STEP)
    parser.add_argument("--gripper-max-torque-limit", type=int, default=DEFAULT_GRIPPER_MAX_TORQUE_LIMIT)
    parser.add_argument("--gripper-torque-limit", type=int, default=DEFAULT_GRIPPER_TORQUE_LIMIT)
    parser.add_argument("--gripper-protection-current", type=int, default=DEFAULT_GRIPPER_PROTECTION_CURRENT)
    parser.add_argument("--gripper-overload-torque", type=int, default=DEFAULT_GRIPPER_OVERLOAD_TORQUE)
    add_power_monitor_args(parser)
    return parser.parse_args()


def apply_robot_safety_defaults(robot: LeKiwi, args: argparse.Namespace) -> None:
    gripper_torque_disabled = False
    try:
        robot.bus.disable_torque("arm_gripper")
        gripper_torque_disabled = True
        robot.bus.write("Max_Torque_Limit", "arm_gripper", args.gripper_max_torque_limit)
        robot.bus.write("Protection_Current", "arm_gripper", args.gripper_protection_current)
        robot.bus.write("Overload_Torque", "arm_gripper", args.gripper_overload_torque)
    except Exception as exc:
        logger.warning("Failed to apply gripper safety settings: %s", exc)
    finally:
        if gripper_torque_disabled:
            try:
                robot.bus.enable_torque("arm_gripper")
                robot.bus.write("Torque_Limit", "arm_gripper", args.gripper_torque_limit)
            except Exception as exc:
                logger.warning("Failed to re-enable gripper torque after safety update: %s", exc)


class ActionLimiter:
    def __init__(self, gripper_max_step: float) -> None:
        self.last_targets: dict[str, float] = {}
        self.max_relative_target = {
            **DEFAULT_MAX_RELATIVE_TARGET,
            "arm_gripper": gripper_max_step,
        }

    def limit(self, action: dict[str, Any]) -> dict[str, Any]:
        limited = dict(action)
        for motor, max_step in self.max_relative_target.items():
            action_key = f"{motor}.pos"
            if action_key not in limited:
                continue

            value = float(limited[action_key])

            if motor in SAFE_ABSOLUTE_POSITION_LIMITS:
                low, high = SAFE_ABSOLUTE_POSITION_LIMITS[motor]
                value = min(max(value, low), high)

            prev = self.last_targets.get(motor)
            if prev is not None:
                value = min(max(value, prev - max_step), prev + max_step)

            limited[action_key] = value

        return limited

    def update(self, action: dict[str, Any]) -> None:
        for motor in self.max_relative_target:
            action_key = f"{motor}.pos"
            if action_key in action:
                self.last_targets[motor] = float(action[action_key])


def release_gripper_torque(robot: LeKiwi) -> bool:
    try:
        robot.bus.disable_torque("arm_gripper")
        return True
    except Exception as exc:
        logger.warning("Failed to release gripper torque: %s", exc)
        return False


class ResilientObservationLoop:
    def __init__(self, robot: LeKiwi, read_retries: int):
        self.robot = robot
        self.read_retries = read_retries
        self.cached_state = dict.fromkeys(robot._state_ft, 0.0)

    def _sequential_read(self, data_name: str, motors: list[str]) -> dict[str, float]:
        values: dict[str, float] = {}
        for motor in motors:
            values[motor] = self.robot.bus.read(data_name, motor, num_retry=self.read_retries)
        return values

    def _read_with_fallback(self, data_name: str, motors: list[str], cache_suffix: str) -> dict[str, float]:
        try:
            return self.robot.bus.sync_read(data_name, motors, num_retry=self.read_retries)
        except Exception as sync_exc:
            logger.warning("sync_read(%s, %s) failed: %s", data_name, motors, sync_exc)

        values: dict[str, float] = {}
        missing: list[str] = []
        for motor in motors:
            try:
                values[motor] = self.robot.bus.read(data_name, motor, num_retry=self.read_retries)
            except Exception as read_exc:
                logger.warning("read(%s, %s) failed: %s", data_name, motor, read_exc)
                missing.append(motor)

        for motor in missing:
            cache_key = f"{motor}{cache_suffix}"
            if cache_key in self.cached_state:
                values[motor] = self.cached_state[cache_key]
            else:
                values[motor] = 0.0

        return values

    def get_observation(self) -> dict[str, Any]:
        arm_pos = self._read_with_fallback("Present_Position", self.robot.arm_motors, ".pos")
        base_wheel_vel = self._read_with_fallback("Present_Velocity", self.robot.base_motors, ".vel")

        base_vel = self.robot._wheel_raw_to_body(
            base_wheel_vel["base_left_wheel"],
            base_wheel_vel["base_back_wheel"],
            base_wheel_vel["base_right_wheel"],
        )

        obs = {f"{motor}.pos": value for motor, value in arm_pos.items()}
        obs.update(base_vel)

        for key in self.cached_state:
            if key in obs:
                self.cached_state[key] = obs[key]

        for cam_key, cam in self.robot.cameras.items():
            try:
                frame = cam.read_latest()
            except Exception as cam_exc:
                logger.warning("camera %s read failed: %s", cam_key, cam_exc)
                frame = None

            if frame is None:
                obs[cam_key] = ""
                continue

            ret, buffer = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
            obs[cam_key] = base64.b64encode(buffer).decode("utf-8") if ret else ""

        return obs


def main() -> None:
    args = parse_args()
    robot_config = LeKiwiConfig(id=args.robot_id, port=args.robot_port)
    if args.disable_cameras:
        robot_config.cameras = {}
    robot = LeKiwi(robot_config)

    robot.connect(calibrate=False)
    apply_robot_safety_defaults(robot, args)
    obs_loop = ResilientObservationLoop(robot, read_retries=args.read_retries)
    action_limiter = ActionLimiter(args.gripper_max_step)
    power_logger = PowerTelemetryLogger(robot, args, logger, "lekiwi_resilient_host")
    power_logger.start()

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
    gripper_torque_released = False

    print(
        f"Resilient LeKiwi host listening on tcp://*:{args.port_zmq_cmd} and tcp://*:{args.port_zmq_observations}",
        flush=True,
    )

    start = time.perf_counter()
    try:
        while time.perf_counter() - start < args.connection_time_s:
            loop_start = time.perf_counter()

            try:
                msg = cmd_socket.recv_string(flags=zmq.NOBLOCK)
                action = json.loads(msg)
                limited_action = action_limiter.limit(action)
                if gripper_torque_released:
                    robot.bus.enable_torque("arm_gripper")
                    gripper_torque_released = False
                robot.send_action(limited_action)
                action_limiter.update(limited_action)
                last_cmd_time = time.time()
                watchdog_active = False
                no_command_logged = False
            except zmq.Again:
                if not no_command_logged:
                    logger.warning("No command available")
                    no_command_logged = True
            except Exception as cmd_exc:
                logger.warning("Command receive/send failed: %s", cmd_exc)

            now = time.time()
            if (now - last_cmd_time > args.watchdog_timeout_ms / 1000) and not watchdog_active:
                logger.warning(
                    "Command not received for more than %s milliseconds. Stopping the base.",
                    args.watchdog_timeout_ms,
                )
                try:
                    robot.stop_base()
                except Exception as stop_exc:
                    logger.warning("stop_base failed: %s", stop_exc)
                gripper_torque_released = release_gripper_torque(robot)
                watchdog_active = True

            try:
                obs = obs_loop.get_observation()
                obs_socket.send_string(json.dumps(obs), flags=zmq.NOBLOCK)
            except zmq.Again:
                pass
            except Exception as obs_exc:
                logger.warning("Observation publish failed: %s", obs_exc)

            power_logger.maybe_sample()

            elapsed = time.perf_counter() - loop_start
            time.sleep(max(1.0 / args.loop_hz - elapsed, 0.0))
    except KeyboardInterrupt:
        print("\nStopping resilient host", flush=True)
    finally:
        try:
            robot.disconnect()
        except Exception as exc:
            print(f"disconnect: {exc}", flush=True)
        power_logger.maybe_sample(force=True)
        power_logger.close()
        cmd_socket.close()
        obs_socket.close()
        ctx.term()


if __name__ == "__main__":
    main()
