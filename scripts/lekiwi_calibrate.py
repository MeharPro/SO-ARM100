#!/usr/bin/env python3

from __future__ import annotations

import argparse

from lerobot.motors import MotorCalibration
from lerobot.motors.feetech import OperatingMode
from lerobot.robots.lekiwi import LeKiwi, LeKiwiConfig

ARM_SERVOS = (
    "arm_shoulder_pan",
    "arm_shoulder_lift",
    "arm_elbow_flex",
    "arm_wrist_flex",
    "arm_wrist_roll",
    "arm_gripper",
)
FULL_TURN_MOTORS = {
    "arm_wrist_roll",
    "base_left_wheel",
    "base_back_wheel",
    "base_right_wheel",
}
FULL_TURN_RANGE = (0, 4095)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="LeKiwi follower calibration helper with single-servo support."
    )
    parser.add_argument("--robot-id", default="follow-mobile")
    parser.add_argument("--robot-port", default="/dev/ttyACM0")
    parser.add_argument("--mode", choices=("full-arm", "single-servo"), default="full-arm")
    parser.add_argument("--servo", choices=ARM_SERVOS)
    return parser.parse_args()


def build_robot(args: argparse.Namespace) -> LeKiwi:
    fields = getattr(LeKiwiConfig, "__dataclass_fields__", {})
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

    return LeKiwi(
        LeKiwiConfig(**kwargs)
    )


def prepare_arm_motors(robot: LeKiwi, motors: list[str]) -> None:
    robot.bus.disable_torque(motors, num_retry=10)
    for motor in motors:
        robot.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value, num_retry=10)


def load_existing_calibration(robot: LeKiwi) -> dict[str, MotorCalibration]:
    if robot.calibration:
        return dict(robot.calibration)
    return robot.bus.read_calibration()


def save_calibration(robot: LeKiwi, calibration: dict[str, MotorCalibration]) -> None:
    robot.calibration = calibration
    robot.bus.write_calibration(calibration)
    robot._save_calibration()
    print(f"Calibration saved to {robot.calibration_fpath}", flush=True)


def update_calibration_entry(
    calibration: dict[str, MotorCalibration],
    robot: LeKiwi,
    motor_name: str,
    homing_offset: int,
    range_min: int,
    range_max: int,
) -> None:
    motor = robot.bus.motors[motor_name]
    calibration[motor_name] = MotorCalibration(
        id=motor.id,
        drive_mode=0,
        homing_offset=int(homing_offset),
        range_min=int(range_min),
        range_max=int(range_max),
    )


def maybe_apply_cached_full_calibration(robot: LeKiwi) -> bool:
    if not robot.calibration:
        return False

    user_input = input(
        "Press ENTER to reuse the saved follower calibration, or type 'c' and press ENTER to run a fresh calibration: "
    )
    if user_input.strip().lower() == "c":
        return False

    print(f"Applying cached follower calibration from {robot.calibration_fpath}.", flush=True)
    robot.bus.write_calibration(robot.calibration)
    return True


def calibrate_full_arm(robot: LeKiwi) -> None:
    if maybe_apply_cached_full_calibration(robot):
        return

    motors = [*robot.arm_motors, *robot.base_motors]
    prepare_arm_motors(robot, list(robot.arm_motors))

    input("Move the follower arm to the middle of its range of motion, then press ENTER.")
    homing_offsets = robot.bus.set_half_turn_homings(robot.arm_motors)
    homing_offsets.update(dict.fromkeys(robot.base_motors, 0))

    full_turn_motors = [motor for motor in motors if motor in FULL_TURN_MOTORS]
    limited_range_motors = [motor for motor in motors if motor not in FULL_TURN_MOTORS]

    print(
        f"Move all arm joints except {full_turn_motors} through their entire usable range. Press ENTER to stop.",
        flush=True,
    )
    range_mins, range_maxes = robot.bus.record_ranges_of_motion(limited_range_motors)
    for motor in full_turn_motors:
        range_mins[motor], range_maxes[motor] = FULL_TURN_RANGE

    calibration: dict[str, MotorCalibration] = {}
    for motor in motors:
        update_calibration_entry(
            calibration,
            robot,
            motor,
            homing_offsets[motor],
            range_mins[motor],
            range_maxes[motor],
        )

    save_calibration(robot, calibration)


def calibrate_single_servo(robot: LeKiwi, servo: str) -> None:
    prepare_arm_motors(robot, [servo])

    input(f"Move {servo} to the middle of its usable range, then press ENTER.")
    homing_offsets = robot.bus.set_half_turn_homings([servo])

    if servo in FULL_TURN_MOTORS:
        range_mins = {servo: FULL_TURN_RANGE[0]}
        range_maxes = {servo: FULL_TURN_RANGE[1]}
        print(f"{servo} is treated as a full-turn joint. Using the full 0..4095 range.", flush=True)
    else:
        print(f"Move only {servo} through its entire usable range. Press ENTER to stop.", flush=True)
        range_mins, range_maxes = robot.bus.record_ranges_of_motion([servo])

    calibration = load_existing_calibration(robot)
    update_calibration_entry(
        calibration,
        robot,
        servo,
        homing_offsets[servo],
        range_mins[servo],
        range_maxes[servo],
    )
    save_calibration(robot, calibration)
    print(f"Updated calibration for {servo}.", flush=True)


def main() -> None:
    args = parse_args()
    if args.mode == "single-servo" and not args.servo:
        raise SystemExit("--servo is required when --mode=single-servo.")

    robot = build_robot(args)
    robot.bus.connect()
    try:
        if args.mode == "single-servo":
            calibrate_single_servo(robot, args.servo)
        else:
            calibrate_full_arm(robot)
    finally:
        robot.bus.disconnect()


if __name__ == "__main__":
    main()
