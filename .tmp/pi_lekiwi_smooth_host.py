#!/usr/bin/env python3

import argparse
import base64
import json
import logging
import time
from typing import Any

import cv2
import zmq

from lerobot.robots.lekiwi import LeKiwi, LeKiwiConfig

logging.basicConfig(level=logging.WARNING)
logger = logging.getLogger("lekiwi_smooth_host")

DEFAULT_ARM_ALPHA = 0.35
DEFAULT_GRIPPER_ALPHA = 0.25
DEFAULT_BASE_ALPHA = 0.40
DEFAULT_ARM_DEADBAND = 0.35
DEFAULT_GRIPPER_DEADBAND = 0.15
DEFAULT_BASE_DEADBAND = 0.0
DEFAULT_ARM_P_COEFFICIENT = 24
DEFAULT_ARM_D_COEFFICIENT = 16
DEFAULT_MAX_STEP = {
    "arm_shoulder_pan": 3.0,
    "arm_shoulder_lift": 3.0,
    "arm_elbow_flex": 3.0,
    "arm_wrist_flex": 3.0,
    "arm_wrist_roll": 4.0,
    "arm_gripper": 1.0,
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Smoother LeKiwi host with command filtering.")
    parser.add_argument("--robot-id", default="follow")
    parser.add_argument("--robot-port", default="/dev/ttyACM0")
    parser.add_argument("--disable-cameras", action="store_true")
    parser.add_argument("--port-zmq-cmd", type=int, default=5555)
    parser.add_argument("--port-zmq-observations", type=int, default=5556)
    parser.add_argument("--connection-time-s", type=int, default=3600)
    parser.add_argument("--watchdog-timeout-ms", type=int, default=500)
    parser.add_argument("--loop-hz", type=float, default=10.0)
    parser.add_argument("--robot-max-relative-target", type=float, default=5.0)
    parser.add_argument("--arm-alpha", type=float, default=DEFAULT_ARM_ALPHA)
    parser.add_argument("--gripper-alpha", type=float, default=DEFAULT_GRIPPER_ALPHA)
    parser.add_argument("--base-alpha", type=float, default=DEFAULT_BASE_ALPHA)
    parser.add_argument("--arm-deadband", type=float, default=DEFAULT_ARM_DEADBAND)
    parser.add_argument("--gripper-deadband", type=float, default=DEFAULT_GRIPPER_DEADBAND)
    parser.add_argument("--base-deadband", type=float, default=DEFAULT_BASE_DEADBAND)
    parser.add_argument("--arm-p-coefficient", type=int, default=DEFAULT_ARM_P_COEFFICIENT)
    parser.add_argument("--arm-d-coefficient", type=int, default=DEFAULT_ARM_D_COEFFICIENT)
    parser.add_argument("--arm-i-coefficient", type=int, default=0)
    parser.add_argument("--shoulder-max-step", type=float, default=DEFAULT_MAX_STEP["arm_shoulder_pan"])
    parser.add_argument("--elbow-max-step", type=float, default=DEFAULT_MAX_STEP["arm_elbow_flex"])
    parser.add_argument("--wrist-max-step", type=float, default=DEFAULT_MAX_STEP["arm_wrist_flex"])
    parser.add_argument("--wrist-roll-max-step", type=float, default=DEFAULT_MAX_STEP["arm_wrist_roll"])
    parser.add_argument("--gripper-max-step", type=float, default=DEFAULT_MAX_STEP["arm_gripper"])
    return parser.parse_args()


class SmoothActionFilter:
    def __init__(self, args: argparse.Namespace) -> None:
        self.last_values: dict[str, float] = {}
        self.arm_alpha = args.arm_alpha
        self.gripper_alpha = args.gripper_alpha
        self.base_alpha = args.base_alpha
        self.arm_deadband = args.arm_deadband
        self.gripper_deadband = args.gripper_deadband
        self.base_deadband = args.base_deadband
        self.max_step = {
            "arm_shoulder_pan": args.shoulder_max_step,
            "arm_shoulder_lift": args.shoulder_max_step,
            "arm_elbow_flex": args.elbow_max_step,
            "arm_wrist_flex": args.wrist_max_step,
            "arm_wrist_roll": args.wrist_roll_max_step,
            "arm_gripper": args.gripper_max_step,
        }

    def reset(self) -> None:
        self.last_values.clear()

    @staticmethod
    def _apply_deadband(value: float, previous: float, deadband: float) -> float:
        if abs(value - previous) <= deadband:
            return previous
        return value

    @staticmethod
    def _apply_alpha(value: float, previous: float, alpha: float) -> float:
        return previous + alpha * (value - previous)

    @staticmethod
    def _apply_step_limit(value: float, previous: float, max_step: float) -> float:
        return min(max(value, previous - max_step), previous + max_step)

    def filter(self, action: dict[str, Any]) -> dict[str, Any]:
        filtered = dict(action)
        for key, raw in action.items():
            if not isinstance(raw, (int, float)):
                continue

            value = float(raw)
            previous = self.last_values.get(key)
            if previous is None:
                filtered[key] = value
                continue

            if key.endswith(".pos"):
                motor = key.removesuffix(".pos")
                deadband = self.gripper_deadband if motor == "arm_gripper" else self.arm_deadband
                alpha = self.gripper_alpha if motor == "arm_gripper" else self.arm_alpha
                max_step = self.max_step.get(motor)

                value = self._apply_deadband(value, previous, deadband)
                value = self._apply_alpha(value, previous, alpha)
                if max_step is not None:
                    value = self._apply_step_limit(value, previous, max_step)
                filtered[key] = value
            elif key.endswith(".vel"):
                value = self._apply_deadband(value, previous, self.base_deadband)
                value = self._apply_alpha(value, previous, self.base_alpha)
                filtered[key] = value

        return filtered

    def update(self, action: dict[str, Any]) -> None:
        for key, value in action.items():
            if isinstance(value, (int, float)):
                self.last_values[key] = float(value)


def encode_observation_images(robot: LeKiwi, observation: dict[str, Any]) -> dict[str, Any]:
    encoded = dict(observation)
    for cam_key in robot.cameras:
        ret, buffer = cv2.imencode(".jpg", observation[cam_key], [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        encoded[cam_key] = base64.b64encode(buffer).decode("utf-8") if ret else ""
    return encoded


def apply_arm_pid(robot: LeKiwi, args: argparse.Namespace) -> None:
    try:
        robot.bus.disable_torque(robot.arm_motors, num_retry=10)
        for name in robot.arm_motors:
            robot.bus.write("P_Coefficient", name, args.arm_p_coefficient, num_retry=10)
            robot.bus.write("I_Coefficient", name, args.arm_i_coefficient, num_retry=10)
            robot.bus.write("D_Coefficient", name, args.arm_d_coefficient, num_retry=10)
        robot.bus.enable_torque(robot.arm_motors, num_retry=10)
    except Exception as exc:
        logger.warning("Failed to apply arm PID coefficients: %s", exc)


def main() -> None:
    args = parse_args()

    robot_config = LeKiwiConfig(
        id=args.robot_id,
        port=args.robot_port,
        max_relative_target=args.robot_max_relative_target,
    )
    if args.disable_cameras:
        robot_config.cameras = {}

    robot = LeKiwi(robot_config)
    robot.connect(calibrate=False)
    apply_arm_pid(robot, args)
    action_filter = SmoothActionFilter(args)

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
        "Smooth LeKiwi host listening on "
        f"tcp://*:{args.port_zmq_cmd} and tcp://*:{args.port_zmq_observations}",
        flush=True,
    )
    print(
        "Filtering params: "
        f"arm_alpha={args.arm_alpha}, gripper_alpha={args.gripper_alpha}, base_alpha={args.base_alpha}, "
        f"arm_deadband={args.arm_deadband}, gripper_deadband={args.gripper_deadband}, "
        f"robot_max_relative_target={args.robot_max_relative_target}, "
        f"arm_pid=({args.arm_p_coefficient}, {args.arm_i_coefficient}, {args.arm_d_coefficient})",
        flush=True,
    )

    start = time.perf_counter()
    try:
        while time.perf_counter() - start < args.connection_time_s:
            loop_start = time.perf_counter()
            try:
                msg = cmd_socket.recv_string(flags=zmq.NOBLOCK)
                incoming_action = json.loads(msg)
                filtered_action = action_filter.filter(incoming_action)
                sent_action = robot.send_action(filtered_action)
                action_filter.update(sent_action)
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
                robot.stop_base()
                action_filter.reset()
                watchdog_active = True

            observation = robot.get_observation()
            encoded = encode_observation_images(robot, observation)
            try:
                obs_socket.send_string(json.dumps(encoded), flags=zmq.NOBLOCK)
            except zmq.Again:
                pass
            except Exception as exc:
                logger.warning("Observation publish failed: %s", exc)

            elapsed = time.perf_counter() - loop_start
            time.sleep(max(1.0 / args.loop_hz - elapsed, 0.0))
    except KeyboardInterrupt:
        print("\nStopping smooth LeKiwi host", flush=True)
    finally:
        try:
            robot.disconnect()
        except Exception as exc:
            print(f"disconnect: {exc}", flush=True)
        cmd_socket.close()
        obs_socket.close()
        ctx.term()


if __name__ == "__main__":
    main()
