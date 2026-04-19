#!/usr/bin/env python3

import argparse
import json
import logging
import runpy
import time
from pathlib import Path

from lekiwi_runtime import (
    ArmSafetyFilter,
    TorqueLimitFileWatcher,
    add_servo_safety_args,
    add_torque_limit_args,
    apply_torque_limits,
    configure_wrist_roll_mode,
    parse_torque_limits_json,
)
from lerobot.robots.lekiwi import LeKiwi, LeKiwiConfig
from lerobot.utils.robot_utils import precise_sleep

logging.basicConfig(level=logging.WARNING)
logger = logging.getLogger("lekiwi_replay_trajectory")

if "PowerTelemetryLogger" not in globals():
    helper_path = Path(__file__).with_name("lekiwi_power.py")
    if helper_path.exists():
        _helper_ns = runpy.run_path(str(helper_path))
        PowerTelemetryLogger = _helper_ns["PowerTelemetryLogger"]
        add_power_monitor_args = _helper_ns["add_power_monitor_args"]
    else:
        raise RuntimeError("Power telemetry helper is unavailable.")

ARM_STATE_KEYS = (
    "arm_shoulder_pan.pos",
    "arm_shoulder_lift.pos",
    "arm_elbow_flex.pos",
    "arm_wrist_flex.pos",
    "arm_wrist_roll.pos",
    "arm_gripper.pos",
)
BASE_STATE_KEYS = ("x.vel", "y.vel", "theta.vel")


def parse_bool(value: str) -> bool:
    normalized = value.strip().lower()
    if normalized in {"1", "true", "yes", "on"}:
        return True
    if normalized in {"0", "false", "no", "off"}:
        return False
    raise argparse.ArgumentTypeError("expected true or false")


def build_robot_config(args: argparse.Namespace) -> LeKiwiConfig:
    fields = getattr(LeKiwiConfig, "__dataclass_fields__", {})
    kwargs = {
        "id": args.robot_id,
        "port": args.robot_port,
    }
    optional_values = {
        "use_degrees": args.use_degrees,
        "base_max_raw_velocity": args.base_max_raw_velocity,
        "base_wheel_torque_limit": args.base_wheel_torque_limit,
        "enable_base": args.enable_base,
    }
    for name, value in optional_values.items():
        if name in fields:
            kwargs[name] = value

    return LeKiwiConfig(**kwargs)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Pi-side trajectory replayer for recorded follower motion."
    )
    parser.add_argument("--robot-id", "--robot.id", dest="robot_id", default="follow-mobile")
    parser.add_argument("--robot-port", "--robot.port", dest="robot_port", default="/dev/ttyACM0")
    parser.add_argument("--robot-cameras-json", "--robot.cameras", dest="robot_cameras_json", default="{}")
    parser.add_argument("--use-degrees", "--robot.use_degrees", dest="use_degrees", type=parse_bool, default=True)
    parser.add_argument("--base-max-raw-velocity", "--robot.base_max_raw_velocity", dest="base_max_raw_velocity", type=int, default=3000)
    parser.add_argument("--base-wheel-torque-limit", "--robot.base_wheel_torque_limit", dest="base_wheel_torque_limit", type=int, default=None)
    parser.add_argument("--enable-base", "--robot.enable_base", dest="enable_base", type=parse_bool, default=True)
    parser.add_argument("--input", required=True, help="Path to a JSON trajectory file created by lekiwi_record_trajectory.py.")
    parser.add_argument(
        "--speed",
        type=float,
        default=1.0,
        help="Replay speed multiplier. Example: 0.5 is half-speed, 2.0 is double-speed.",
    )
    parser.add_argument(
        "--include-base",
        action="store_true",
        help="Replay recorded base velocities too. Default is arm-only replay with a stationary base.",
    )
    parser.add_argument("--hold-final-s", type=float, default=0.5, help="How long to hold the final arm target.")
    parser.add_argument("--print-every", type=int, default=30, help="Print every N replayed samples.")
    add_servo_safety_args(parser)
    add_torque_limit_args(parser)
    add_power_monitor_args(parser)
    return parser.parse_args()


def configure_cameras(robot_config: LeKiwiConfig, cameras_json: str) -> None:
    value = cameras_json.strip()
    if value in {"", "default", "__default__", "null"}:
        return

    parsed = json.loads(value)
    if parsed == {}:
        robot_config.cameras = {}
        return

    raise SystemExit("Only '{}' or 'default' are currently supported for --robot-cameras-json.")


def load_trajectory(path_str: str) -> tuple[Path, list[dict[str, object]]]:
    path = Path(path_str).expanduser()
    if not path.exists():
        raise SystemExit(f"Trajectory file does not exist: {path}")

    payload = json.loads(path.read_text())
    if payload.get("format") != "lekiwi-follower-trajectory":
        raise SystemExit(f"Unsupported trajectory format in {path}")

    samples = payload.get("samples")
    if not isinstance(samples, list) or not samples:
        raise SystemExit(f"No samples found in {path}")

    return path, samples


def build_action(state: dict[str, float], include_base: bool) -> dict[str, float]:
    action = {key: float(state[key]) for key in ARM_STATE_KEYS}
    if include_base:
        action.update({key: float(state[key]) for key in BASE_STATE_KEYS})
    else:
        action.update(dict.fromkeys(BASE_STATE_KEYS, 0.0))
    return action


def summarize_arm_action(action: dict[str, float]) -> str:
    return " ".join(f"{key}={action[key]:7.2f}" for key in ARM_STATE_KEYS)


def main() -> None:
    args = parse_args()
    if args.speed <= 0:
        raise SystemExit("--speed must be greater than 0.")

    trajectory_path, samples = load_trajectory(args.input)
    robot_config = build_robot_config(args)
    configure_cameras(robot_config, args.robot_cameras_json)
    robot = LeKiwi(robot_config)
    power_logger = None

    print(f"Connecting to LeKiwi on {args.robot_port}")
    robot.connect(calibrate=False)
    configure_wrist_roll_mode(robot, continuous=args.safer_servo_mode)
    apply_torque_limits(robot, parse_torque_limits_json(args.torque_limits_json))
    torque_watcher = TorqueLimitFileWatcher(args.torque_limits_path)
    torque_watcher.poll(robot, force=True)
    safety_filter = ArmSafetyFilter(enabled=args.safer_servo_mode)
    try:
        safety_filter.seed_from_observation(robot.get_observation())
    except Exception as exc:
        logger.warning("Failed to seed servo safety filter from the current robot state: %s", exc)
    power_logger = PowerTelemetryLogger(robot, args, logger, "lekiwi_replay_trajectory")
    power_logger.start()
    print(f"Replaying {len(samples)} samples from {trajectory_path}")
    if args.include_base:
        print("Base replay: enabled")
    else:
        print("Base replay: disabled (base held at zero velocity)")
    if args.safer_servo_mode:
        print("Safer servo mode enabled for replay.")

    start = time.perf_counter()
    last_action: dict[str, float] | None = None

    try:
        for idx, sample in enumerate(samples, start=1):
            state = sample["state"]
            target_t = float(sample["t_s"]) / args.speed
            precise_sleep(max(target_t - (time.perf_counter() - start), 0.0))

            torque_watcher.poll(robot)
            action = safety_filter.normalize(build_action(state, include_base=args.include_base))
            sent_action = robot.send_action(action)
            safety_filter.update(sent_action)
            last_action = sent_action
            power_logger.maybe_sample()

            if idx == 1 or idx % args.print_every == 0 or idx == len(samples):
                print(f"[{idx:05d}/{len(samples):05d}] t={target_t:7.3f}s {summarize_arm_action(action)}")

        if last_action is not None and args.hold_final_s > 0:
            precise_sleep(args.hold_final_s)
    except KeyboardInterrupt:
        print("\nReplay interrupted.")
    finally:
        try:
            if last_action is not None:
                robot.send_action(
                    {
                        **{key: last_action[key] for key in ARM_STATE_KEYS},
                        **dict.fromkeys(BASE_STATE_KEYS, 0.0),
                    }
                )
            else:
                robot.stop_base()
        except Exception:
            pass
        if power_logger is not None:
            try:
                power_logger.maybe_sample(force=True)
            except Exception:
                pass
        robot.disconnect()
        if power_logger is not None:
            power_logger.close()


if __name__ == "__main__":
    main()
