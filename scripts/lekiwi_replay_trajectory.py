#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import logging
import runpy
import time
from pathlib import Path

from lekiwi_runtime import (
    ArmSafetyFilter,
    LiveRobotSensorStatusEmitter,
    ResilientObservationReader,
    ServoProtectionSupervisor,
    TorqueLimitFileWatcher,
    apply_robot_action,
    add_servo_safety_args,
    add_torque_limit_args,
    apply_torque_limits,
    build_normalized_arm_position_limits,
    configure_wrist_roll_mode,
    disconnect_robot,
    parse_torque_limits_json,
    stop_robot_base,
)
from lekiwi_sensor_replay import (
    SENSOR_PREPOSITION_TIMEOUT_S,
    SensorAwareReplayState,
    preposition_vex_base_to_recorded_state,
    recorded_state_has_sensor_reference,
)
from vex_base_bridge import (
    VexBaseBridge,
    VexBaseTelemetryManager,
    add_vex_base_args,
    ensure_vex_command_stream,
    normalize_vex_control_config,
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
VEX_POSITION_LOG_PREFIX = "[vex-position]"


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
    parser.add_argument(
        "--vex-replay-mode",
        choices=("drive", "ecu"),
        default="ecu",
        help="How the VEX base replay program should apply recorded motion.",
    )
    parser.add_argument("--hold-final-s", type=float, default=0.5, help="How long to hold the final arm target.")
    parser.add_argument("--print-every", type=int, default=30, help="Print every N replayed samples.")
    add_servo_safety_args(parser)
    add_torque_limit_args(parser)
    add_vex_base_args(parser)
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


def print_vex_position_status(
    status: str,
    *,
    reason: str | None = None,
    timeout_s: float = SENSOR_PREPOSITION_TIMEOUT_S,
    message: str | None = None,
    arm_replay: str = "continuing",
) -> None:
    payload: dict[str, object] = {
        "status": status,
        "timeout_s": round(float(timeout_s), 3) if float(timeout_s) > 0 else None,
        "arm_replay": arm_replay,
    }
    if reason:
        payload["reason"] = reason
    if message:
        payload["message"] = message
    print(f"{VEX_POSITION_LOG_PREFIX} {json.dumps(payload, separators=(',', ':'))}", flush=True)


def main() -> None:
    args = parse_args()
    if args.speed <= 0:
        raise SystemExit("--speed must be greater than 0.")

    trajectory_path, samples = load_trajectory(args.input)
    robot_config = build_robot_config(args)
    configure_cameras(robot_config, args.robot_cameras_json)
    robot = LeKiwi(robot_config)
    power_logger = None
    observation_reader = None
    sensor_status_emitter = None
    vex_base_bridge = None
    vex_base_telemetry = None
    vex_control_config = normalize_vex_control_config(json.loads(args.vex_control_config_json))

    print(f"Connecting to LeKiwi on {args.robot_port}")
    robot.connect(calibrate=False)
    configure_wrist_roll_mode(robot, continuous=args.safer_servo_mode)
    apply_torque_limits(robot, parse_torque_limits_json(args.torque_limits_json))
    torque_watcher = TorqueLimitFileWatcher(args.torque_limits_path)
    torque_watcher.poll(robot, force=True)
    absolute_position_limits = build_normalized_arm_position_limits(
        robot,
        preserve_continuous_wrist_roll=args.safer_servo_mode,
    )
    safety_filter = ArmSafetyFilter(
        enabled=args.safer_servo_mode,
        absolute_position_limits=absolute_position_limits,
    )
    servo_protection = ServoProtectionSupervisor(robot, logger)
    observation_reader = ResilientObservationReader(robot, logger)
    sensor_status_emitter = LiveRobotSensorStatusEmitter()
    try:
        observation = observation_reader.get_observation()
        safety_filter.seed_from_observation(observation)
        servo_protection.seed_from_observation(observation)
    except Exception as exc:
        logger.warning("Failed to seed servo safety filter from the current robot state: %s", exc)
    power_logger = PowerTelemetryLogger(robot, args, logger, "lekiwi_replay_trajectory")
    power_logger.start()
    vex_base_bridge = VexBaseBridge(
        requested_port=args.vex_base_port,
        baudrate=args.vex_base_baudrate,
        stale_after_s=args.vex_base_stale_after_s,
        command_timeout_s=args.vex_base_command_timeout_s,
        logger=logger,
    )
    vex_base_bridge.connect()
    start_state = samples[0]["state"] if samples and isinstance(samples[0], dict) else None
    replay_to_vex_base = args.include_base and not args.enable_base
    auto_preposition_base = (
        not args.enable_base
        and isinstance(start_state, dict)
        and recorded_state_has_sensor_reference(start_state)
    )
    vex_base_telemetry = VexBaseTelemetryManager(
        requested_vexcom_path=args.vex_vexcom_path,
        telemetry_slot=args.vex_telemetry_slot,
        cache_dir=args.vex_program_cache_dir,
        logger=logger,
    )
    prepare_vex_base = replay_to_vex_base or auto_preposition_base
    if prepare_vex_base:
        if not ensure_vex_command_stream(
            vex_base_bridge,
            vex_base_telemetry,
            vex_control_config,
            program_name=args.vex_telemetry_program_name,
            logger=logger,
        ):
            if replay_to_vex_base:
                raise SystemExit("VEX base replay requested, but the Brain did not accept live USB commands.")
            if auto_preposition_base:
                print(
                    "Warning: recorded ultrasonic/gyro start alignment is available, but this VEX Brain did not accept live Pi commands.",
                    flush=True,
                )
                print_vex_position_status(
                    "skipped",
                    reason="command-stream-unavailable",
                    message="VEX start positioning skipped because the Brain did not accept live USB commands.",
                )
                auto_preposition_base = False
            prepare_vex_base = False
    print(vex_base_bridge.status_message, flush=True)
    print(f"Replaying {len(samples)} samples from {trajectory_path}")
    if args.include_base:
        print(f"Base replay: enabled ({args.vex_replay_mode} mode)")
    elif auto_preposition_base:
        print("Base replay: arm-only after recorded ultrasonic/gyro start alignment")
    else:
        print("Base replay: disabled (base held at zero velocity)")
    if args.safer_servo_mode:
        print("Safer servo mode enabled for replay.")

    start = time.perf_counter()
    last_action: dict[str, float] | None = None
    replay_state = None
    last_base_command_at: float | None = None
    vex_base_control_used = False
    if prepare_vex_base:
        replay_state = SensorAwareReplayState(
            samples,
            replay_mode=args.vex_replay_mode,
            speed=args.speed,
            control_config=vex_control_config,
        )
        if isinstance(start_state, dict):
            if auto_preposition_base:
                print("Aligning VEX base to recorded ultrasonic/gyro start pose.", flush=True)
            prepositioned = preposition_vex_base_to_recorded_state(
                vex_base_bridge,
                observation_reader,
                replay_state,
                start_state,
            )
            vex_base_control_used = True
            if not prepositioned:
                failure_reason = getattr(prepositioned, "reason", None) or "not-aligned"
                failure_axis = getattr(prepositioned, "axis", None)
                failure_detail = getattr(prepositioned, "detail", None)
                failure_message = "VEX start positioning stopped"
                if failure_axis:
                    failure_message += f" on {failure_axis}"
                if failure_detail:
                    failure_message += f": {failure_detail}"
                failure_message += "; arm replay was aborted."
                print(
                    "Warning: stopped recentering the VEX base before the recorded X/Y/gyro start was aligned.",
                    flush=True,
                )
                print_vex_position_status(
                    "skipped",
                    reason=failure_reason,
                    message=failure_message,
                    arm_replay="aborted",
                )
                raise SystemExit(1)
            elif auto_preposition_base:
                print_vex_position_status(
                    "aligned",
                    message="VEX start positioning completed before arm replay.",
                )
        replay_state.prepare()

    try:
        for idx, sample in enumerate(samples, start=1):
            state = sample["state"]
            target_t = float(sample["t_s"]) / args.speed
            precise_sleep(max(target_t - (time.perf_counter() - start), 0.0))

            torque_watcher.poll(robot)
            action = safety_filter.normalize(build_action(state, include_base=args.include_base))
            sent_action = apply_robot_action(
                robot,
                action,
                allow_legacy_base=args.enable_base,
            )
            safety_filter.update(sent_action)
            servo_protection.record_command(sent_action)
            last_action = sent_action
            power_logger.maybe_sample()
            if replay_to_vex_base and replay_state is not None:
                command_now = time.perf_counter()
                command_dt_s = (
                    command_now - last_base_command_at
                    if last_base_command_at is not None
                    else max(0.02, target_t)
                )
                base_command = replay_state.build_replay_command(
                    state,
                    command_dt_s=command_dt_s,
                )
                if base_command.mode == "ecu" and base_command.ecu_targets is not None:
                    vex_base_bridge.send_ecu_targets(
                        base_command.ecu_targets,
                        motion=base_command.motion,
                        command_dt_ms=base_command.command_dt_ms,
                    )
                else:
                    vex_base_bridge.send_motion(base_command.motion)
                last_base_command_at = command_now
            observation = observation_reader.get_observation()
            observation = vex_base_bridge.merge_observation(observation)
            observation = servo_protection.enrich_observation(observation)
            sensor_status_emitter.emit(
                observation_reader,
                observation,
                source="replay",
                vex_base_bridge=vex_base_bridge,
            )
            if servo_protection.observe(observation, power_logger.last_sample):
                stop_robot_base(robot, allow_legacy_base=args.enable_base)
                print("Replay stopped because the arm safety latch tripped.", flush=True)
                break

            if idx == 1 or idx % args.print_every == 0 or idx == len(samples):
                print(f"[{idx:05d}/{len(samples):05d}] t={target_t:7.3f}s {summarize_arm_action(action)}")

        if last_action is not None and args.hold_final_s > 0:
            hold_until = time.perf_counter() + args.hold_final_s
            if replay_to_vex_base:
                vex_base_bridge.send_hold()
            while time.perf_counter() < hold_until:
                power_logger.maybe_sample()
                observation = observation_reader.get_observation()
                observation = vex_base_bridge.merge_observation(observation)
                observation = servo_protection.enrich_observation(observation)
                sensor_status_emitter.emit(
                    observation_reader,
                    observation,
                    source="replay",
                    vex_base_bridge=vex_base_bridge,
                )
                if servo_protection.observe(observation, power_logger.last_sample):
                    stop_robot_base(robot, allow_legacy_base=args.enable_base)
                    print("Replay stopped because the arm safety latch tripped.", flush=True)
                    break
                precise_sleep(min(0.05, max(hold_until - time.perf_counter(), 0.0)))
    except KeyboardInterrupt:
        print("\nReplay interrupted.")
    finally:
        try:
            if last_action is not None and args.enable_base:
                robot.send_action(
                    {
                        **{key: last_action[key] for key in ARM_STATE_KEYS},
                        **dict.fromkeys(BASE_STATE_KEYS, 0.0),
                    }
                )
            else:
                stop_robot_base(robot, allow_legacy_base=args.enable_base)
        except Exception:
            pass
        if power_logger is not None:
            try:
                power_logger.maybe_sample(force=True)
            except Exception:
                pass
        if replay_to_vex_base or vex_base_control_used:
            vex_base_bridge.send_hold()
        disconnect_robot(robot, allow_legacy_base=args.enable_base)
        if vex_base_bridge is not None:
            vex_base_bridge.close()
        if power_logger is not None:
            power_logger.close()


if __name__ == "__main__":
    main()
