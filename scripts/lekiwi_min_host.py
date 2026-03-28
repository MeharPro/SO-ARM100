#!/usr/bin/env python3

import argparse
import base64
import json
import logging
import runpy
import time
from pathlib import Path

import cv2
import zmq

from lerobot.robots.lekiwi import LeKiwi, LeKiwiConfig

logging.basicConfig(level=logging.WARNING)
logger = logging.getLogger("lekiwi_min_host")

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
    parser = argparse.ArgumentParser(description="Minimal command-only LeKiwi host.")
    parser.add_argument("--robot-id", default="follow")
    parser.add_argument("--robot-port", default="/dev/ttyACM0")
    parser.add_argument("--listen-host", default="*")
    parser.add_argument("--port-zmq-cmd", type=int, default=5555)
    parser.add_argument("--port-zmq-observations", type=int, default=5556)
    parser.add_argument("--watchdog-timeout-s", type=float, default=0.5)
    parser.add_argument("--loop-hz", type=float, default=30.0)
    parser.add_argument("--enable-observations", action="store_true")
    parser.add_argument("--enable-cameras", action="store_true")
    parser.add_argument("--gripper-max-step", type=float, default=DEFAULT_GRIPPER_MAX_STEP)
    parser.add_argument("--gripper-max-torque-limit", type=int, default=DEFAULT_GRIPPER_MAX_TORQUE_LIMIT)
    parser.add_argument("--gripper-torque-limit", type=int, default=DEFAULT_GRIPPER_TORQUE_LIMIT)
    parser.add_argument("--gripper-protection-current", type=int, default=DEFAULT_GRIPPER_PROTECTION_CURRENT)
    parser.add_argument("--gripper-overload-torque", type=int, default=DEFAULT_GRIPPER_OVERLOAD_TORQUE)
    add_power_monitor_args(parser)
    return parser.parse_args()


def get_camera_observation(robot: LeKiwi) -> dict[str, str]:
    obs: dict[str, str] = {}
    for cam_key, cam in robot.cameras.items():
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

    def limit(self, action: dict[str, float]) -> dict[str, float]:
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

    def update(self, action: dict[str, float]) -> None:
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


def main() -> None:
    args = parse_args()

    robot_config = LeKiwiConfig(id=args.robot_id, port=args.robot_port)
    if not args.enable_cameras:
        robot_config.cameras = {}
    robot = LeKiwi(robot_config)
    robot.connect(calibrate=False)
    apply_robot_safety_defaults(robot, args)
    action_limiter = ActionLimiter(args.gripper_max_step)
    power_logger = PowerTelemetryLogger(robot, args, logger, "lekiwi_min_host")
    power_logger.start()

    ctx = zmq.Context()
    cmd_socket = ctx.socket(zmq.PULL)
    cmd_socket.setsockopt(zmq.CONFLATE, 1)
    cmd_socket.bind(f"tcp://{args.listen_host}:{args.port_zmq_cmd}")

    obs_socket = None
    if args.enable_observations:
        obs_socket = ctx.socket(zmq.PUSH)
        obs_socket.setsockopt(zmq.CONFLATE, 1)
        obs_socket.bind(f"tcp://{args.listen_host}:{args.port_zmq_observations}")

    last_cmd_time = time.time()
    watchdog_active = False
    gripper_torque_released = False

    print(f"Minimal LeKiwi host listening on tcp://{args.listen_host}:{args.port_zmq_cmd}", flush=True)
    if obs_socket is not None:
        print(
            f"Minimal LeKiwi host publishing observations on tcp://{args.listen_host}:{args.port_zmq_observations}",
            flush=True,
        )

    try:
        while True:
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
            except zmq.Again:
                pass
            except Exception as cmd_exc:
                logger.warning("Command receive/send failed: %s", cmd_exc)

            now = time.time()
            if (now - last_cmd_time > args.watchdog_timeout_s) and not watchdog_active:
                print("Watchdog timeout, stopping base")
                robot.stop_base()
                gripper_torque_released = release_gripper_torque(robot)
                watchdog_active = True

            if obs_socket is not None:
                try:
                    obs_socket.send_string(json.dumps(get_camera_observation(robot)), flags=zmq.NOBLOCK)
                except zmq.Again:
                    pass
                except Exception as obs_exc:
                    logger.warning("Observation publish failed: %s", obs_exc)

            power_logger.maybe_sample()

            elapsed = time.perf_counter() - loop_start
            time.sleep(max(1.0 / args.loop_hz - elapsed, 0.0))
    except KeyboardInterrupt:
        print("\nStopping minimal host", flush=True)
    finally:
        try:
            robot.stop_base()
        except Exception as exc:
            print(f"stop_base: {exc}", flush=True)
        try:
            robot.disconnect()
        except Exception as exc:
            print(f"disconnect: {exc}", flush=True)
        power_logger.maybe_sample(force=True)
        power_logger.close()
        cmd_socket.close()
        if obs_socket is not None:
            obs_socket.close()
        ctx.term()


if __name__ == "__main__":
    main()
