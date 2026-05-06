#!/usr/bin/env python3

import argparse
import json
import sys

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

from lekiwi_runtime import resolve_lekiwi_robot_port

DEFAULT_GRIPPER_ID = 6
DEFAULT_MAX_TORQUE_LIMIT = 400
DEFAULT_TORQUE_LIMIT = 350
DEFAULT_PROTECTION_CURRENT = 200
DEFAULT_OVERLOAD_TORQUE = 20


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Recover the LeKiwi gripper after a thermal trip and persist safer Feetech settings."
    )
    parser.add_argument("--robot-port", default="/dev/ttyACM0")
    parser.add_argument("--gripper-id", type=int, default=DEFAULT_GRIPPER_ID)
    parser.add_argument("--num-retry", type=int, default=2)
    parser.add_argument("--max-torque-limit", type=int, default=DEFAULT_MAX_TORQUE_LIMIT)
    parser.add_argument("--torque-limit", type=int, default=DEFAULT_TORQUE_LIMIT)
    parser.add_argument("--protection-current", type=int, default=DEFAULT_PROTECTION_CURRENT)
    parser.add_argument("--overload-torque", type=int, default=DEFAULT_OVERLOAD_TORQUE)
    parser.add_argument("--leave-torque-disabled", action="store_true")
    return parser.parse_args()


def read_registers(bus: FeetechMotorsBus, num_retry: int) -> dict[str, int | str]:
    values: dict[str, int | str] = {}
    for register in (
        "Present_Temperature",
        "Present_Voltage",
        "Present_Current",
        "Present_Position",
        "Max_Torque_Limit",
        "Torque_Limit",
        "Protection_Current",
        "Overload_Torque",
    ):
        try:
            values[register] = bus.read(register, "arm_gripper", normalize=False, num_retry=num_retry)
        except Exception as exc:
            values[register] = f"<read failed: {exc}>"
    return values


def main() -> int:
    args = parse_args()
    resolved_port = resolve_lekiwi_robot_port(args.robot_port)
    if resolved_port != args.robot_port:
        print(
            f"Using detected LeKiwi robot port {resolved_port} instead of configured {args.robot_port}.",
            flush=True,
        )
        args.robot_port = resolved_port
    motors = {
        "arm_gripper": Motor(args.gripper_id, "sts3215", MotorNormMode.RANGE_0_100),
    }
    bus = FeetechMotorsBus(
        port=args.robot_port,
        motors=motors,
    )

    connected = False
    torque_disabled = False
    try:
        bus.connect(handshake=False)
        connected = True

        print("Current gripper registers:")
        print(json.dumps(read_registers(bus, args.num_retry), indent=2, sort_keys=True))

        try:
            bus.disable_torque("arm_gripper", num_retry=args.num_retry)
            torque_disabled = True
        except Exception as exc:
            print(
                "The gripper is still reporting an overheat or write fault.",
                "Let it cool fully, open the jaw by hand, power-cycle the arm, then rerun this script.",
                str(exc),
                sep="\n",
                file=sys.stderr,
            )
            return 1

        settings = {
            "Max_Torque_Limit": args.max_torque_limit,
            "Protection_Current": args.protection_current,
            "Overload_Torque": args.overload_torque,
        }
        for register, value in settings.items():
            bus.write(register, "arm_gripper", value, normalize=False, num_retry=args.num_retry)

        if not args.leave_torque_disabled:
            bus.enable_torque("arm_gripper", num_retry=args.num_retry)
            torque_disabled = False
            bus.write("Torque_Limit", "arm_gripper", args.torque_limit, normalize=False, num_retry=args.num_retry)

        print("\nUpdated gripper registers:")
        print(json.dumps(read_registers(bus, args.num_retry), indent=2, sort_keys=True))
        return 0
    finally:
        if connected:
            bus.disconnect(disable_torque=torque_disabled)


if __name__ == "__main__":
    raise SystemExit(main())
