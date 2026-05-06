#!/usr/bin/env python3

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path


def ensure_lerobot_on_path() -> None:
    candidates: list[Path] = []
    project_dir = os.getenv("LEKIWI_PROJECT_DIR")
    if project_dir:
        candidates.append(Path(project_dir).expanduser() / "src")

    script_path = Path(__file__).resolve()
    candidates.extend(
        [
            script_path.parents[1] / "src",
            script_path.parents[2] / "lerobot" / "src",
            Path("/home/pi/lerobot/src"),
        ]
    )

    for candidate in candidates:
        if (candidate / "lerobot").exists():
            sys.path.insert(0, str(candidate))
            return

    raise SystemExit(
        "Could not find the lerobot Python package. "
        "Run this from ~/lerobot or set LEKIWI_PROJECT_DIR to your lerobot checkout."
    )


ensure_lerobot_on_path()

from lekiwi_runtime import resolve_lekiwi_robot_port
from lerobot.motors.feetech import OperatingMode
from lerobot.robots.lekiwi import LeKiwi, LeKiwiConfig


TARGET_SERVO = "arm_elbow_flex"
TARGET_ID = 3
SAFE_ELBOW_TORQUE_LIMIT = 600


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Assign a replacement STS3215 servo to the LeKiwi elbow slot "
            f"({TARGET_SERVO}, id {TARGET_ID}) on the Pi."
        )
    )
    parser.add_argument("--robot-id", default="follow-mobile")
    parser.add_argument("--robot-port", default="/dev/ttyACM0")
    parser.add_argument(
        "--current-id",
        type=int,
        default=None,
        help="Current ID of the replacement servo. If omitted, the script tries to auto-find a single motor.",
    )
    parser.add_argument(
        "--current-baudrate",
        type=int,
        default=None,
        help="Current baudrate of the replacement servo. If omitted, the script scans supported baudrates.",
    )
    return parser.parse_args()


def build_robot(args: argparse.Namespace) -> LeKiwi:
    fields = getattr(LeKiwiConfig, "__dataclass_fields__", {})
    resolved_port = resolve_lekiwi_robot_port(args.robot_port)
    if resolved_port != args.robot_port:
        print(
            f"Using detected LeKiwi robot port {resolved_port} instead of configured {args.robot_port}.",
            flush=True,
        )
        args.robot_port = resolved_port
    kwargs = {
        "id": args.robot_id,
        "port": args.robot_port,
    }
    optional_values = {
        "use_degrees": True,
        "cameras": {},
        "enable_base": False,
    }
    for name, value in optional_values.items():
        if name in fields:
            kwargs[name] = value

    return LeKiwi(LeKiwiConfig(**kwargs))


def main() -> None:
    args = parse_args()
    robot = build_robot(args)

    print(
        f"Target slot: {TARGET_SERVO} (expected controller id {TARGET_ID}) on {args.robot_port}",
        flush=True,
    )
    if args.current_id is None:
        print(
            "Auto-find mode: only the replacement elbow servo should be connected to the arm bus.",
            flush=True,
        )

    try:
        robot.bus.setup_motor(
            TARGET_SERVO,
            initial_baudrate=args.current_baudrate,
            initial_id=args.current_id,
        )
        model_number = robot.bus.ping(TARGET_SERVO, num_retry=5, raise_on_error=True)

        # Leave the replacement in a known-safe configuration without enabling torque.
        robot.bus.disable_torque(TARGET_SERVO, num_retry=10)
        robot.bus.write("Operating_Mode", TARGET_SERVO, OperatingMode.POSITION.value, num_retry=10)
        robot.bus.write(
            "Torque_Limit",
            TARGET_SERVO,
            SAFE_ELBOW_TORQUE_LIMIT,
            num_retry=10,
        )

        print(
            f"Paired replacement servo as {TARGET_SERVO} with id {TARGET_ID}. "
            f"Model={model_number}. Torque is left disabled.",
            flush=True,
        )
        print(
            "Next step: run single-servo calibration before powering the whole arm:",
            flush=True,
        )
        print(
            f"python scripts/lekiwi_calibrate.py --mode single-servo --servo {TARGET_SERVO} --robot-port {args.robot_port}",
            flush=True,
        )
    finally:
        try:
            robot.bus.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    main()
