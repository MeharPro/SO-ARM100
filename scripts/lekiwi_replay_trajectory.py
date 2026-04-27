#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import logging
import math
import runpy
import time
from datetime import datetime, timezone
from pathlib import Path

from lekiwi_runtime import (
    ArmSafetyFilter,
    DEFAULT_SAFER_ARM_MAX_STEP,
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
    ULTRASONIC_MAX_DISTANCE_M,
    ULTRASONIC_MIN_DISTANCE_M,
    ULTRASONIC_STATE_KEYS,
)
from lekiwi_sensor_replay import (
    PREPOSITION_HEADING_CROSS_TRIM_DEG,
    PREPOSITION_LINEAR_CROSS_TRIM_M,
    SENSOR_PREPOSITION_TIMEOUT_S,
    THETA_TRACK_TOLERANCE_DEG,
    SensorAwareReplayState,
    XY_TRACK_TOLERANCE_M,
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
GRIPPER_STATE_KEY = "arm_gripper.pos"
ARM_HOME_MOTION_KEYS = tuple(key for key in ARM_STATE_KEYS if key != GRIPPER_STATE_KEY)
BASE_STATE_KEYS = ("x.vel", "y.vel", "theta.vel")
VEX_POSITION_LOG_PREFIX = "[vex-position]"
HOME_POSITION_TOLERANCE_DEG = 3.0
HOME_SETTLE_TIMEOUT_S = 5.0
ULTRASONIC_X_DISTANCE_KEY = ULTRASONIC_STATE_KEYS[0]
ULTRASONIC_Y_DISTANCE_KEY = ULTRASONIC_STATE_KEYS[1]
ULTRASONIC_X_POSITION_KEY = "ultrasonic_sensor_1.position_m"
ULTRASONIC_Y_POSITION_KEY = "ultrasonic_sensor_2.position_m"
ULTRASONIC_POSITION_KEYS = (
    ULTRASONIC_X_POSITION_KEY,
    ULTRASONIC_Y_POSITION_KEY,
)


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
        "--auto-vex-positioning",
        type=parse_bool,
        default=True,
        help="Automatically align the VEX base to the recording start using gyro/ultrasonic references.",
    )
    parser.add_argument(
        "--vex-positioning-timeout-s",
        type=float,
        default=SENSOR_PREPOSITION_TIMEOUT_S,
        help="Maximum seconds to spend aligning the VEX base before arm replay.",
    )
    parser.add_argument(
        "--vex-positioning-xy-tolerance-m",
        type=float,
        default=XY_TRACK_TOLERANCE_M,
        help="X/Y error in meters considered aligned during VEX start positioning.",
    )
    parser.add_argument(
        "--vex-positioning-heading-tolerance-deg",
        type=float,
        default=THETA_TRACK_TOLERANCE_DEG,
        help="Heading error in degrees considered aligned during VEX start positioning.",
    )
    parser.add_argument(
        "--vex-positioning-xy-trim-tolerance-m",
        type=float,
        default=PREPOSITION_LINEAR_CROSS_TRIM_M,
        help="Maximum X/Y overshoot in meters allowed after crossing the start-position target.",
    )
    parser.add_argument(
        "--vex-positioning-heading-trim-tolerance-deg",
        type=float,
        default=PREPOSITION_HEADING_CROSS_TRIM_DEG,
        help="Maximum heading overshoot in degrees allowed after crossing the start-position target.",
    )
    parser.add_argument(
        "--vex-replay-mode",
        choices=("drive", "ecu"),
        default="ecu",
        help="How the VEX base replay program should apply recorded motion.",
    )
    parser.add_argument("--hold-final-s", type=float, default=0.5, help="How long to hold the final arm target.")
    parser.add_argument(
        "--home-mode",
        choices=("none", "start", "end", "both"),
        default="none",
        help="Move the follower arm to the saved home position before replay, after replay, or both.",
    )
    parser.add_argument(
        "--home-position-json",
        default="",
        help="JSON object containing saved follower arm joint positions for --home-mode.",
    )
    parser.add_argument(
        "--recapture-ultrasonic-stream",
        action="store_true",
        help="Replay the full recording and overwrite its X/Y ultrasonic stream from live sensor readings.",
    )
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


def load_trajectory(path_str: str) -> tuple[Path, dict[str, object], list[dict[str, object]]]:
    path = Path(path_str).expanduser()
    if not path.exists():
        raise SystemExit(f"Trajectory file does not exist: {path}")

    payload = json.loads(path.read_text())
    if payload.get("format") != "lekiwi-follower-trajectory":
        raise SystemExit(f"Unsupported trajectory format in {path}")

    samples = payload.get("samples")
    if not isinstance(samples, list) or not samples:
        raise SystemExit(f"No samples found in {path}")

    return path, payload, samples


def finite_number(value: object) -> float | None:
    if isinstance(value, (int, float)):
        parsed = float(value)
        if math.isfinite(parsed):
            return parsed
    return None


def valid_ultrasonic_distance(value: object) -> float | None:
    parsed = finite_number(value)
    if parsed is None:
        return None
    if ULTRASONIC_MIN_DISTANCE_M <= parsed <= ULTRASONIC_MAX_DISTANCE_M:
        return parsed
    return None


def clear_recaptured_ultrasonic_state(state: dict[str, object]) -> None:
    for key in (
        ULTRASONIC_X_DISTANCE_KEY,
        ULTRASONIC_Y_DISTANCE_KEY,
        ULTRASONIC_X_POSITION_KEY,
        ULTRASONIC_Y_POSITION_KEY,
    ):
        state.pop(key, None)


def recapture_ultrasonic_sample(
    sample: dict[str, object],
    observation: dict[str, object],
    reference: dict[str, object] | None,
) -> tuple[dict[str, object] | None, bool]:
    state = sample.get("state")
    if not isinstance(state, dict):
        return reference, False

    x_m = valid_ultrasonic_distance(observation.get(ULTRASONIC_X_DISTANCE_KEY))
    y_m = valid_ultrasonic_distance(observation.get(ULTRASONIC_Y_DISTANCE_KEY))
    if x_m is None or y_m is None:
        clear_recaptured_ultrasonic_state(state)
        return reference, False

    t_s = finite_number(sample.get("t_s"))
    if reference is None:
        reference = {
            "type": "replay-recapture",
            "x_distance_key": ULTRASONIC_X_DISTANCE_KEY,
            "y_distance_key": ULTRASONIC_Y_DISTANCE_KEY,
            "x_position_key": ULTRASONIC_X_POSITION_KEY,
            "y_position_key": ULTRASONIC_Y_POSITION_KEY,
            "x_distance_m": round(x_m, 6),
            "y_distance_m": round(y_m, 6),
            "reference_t_s": round(t_s if t_s is not None else 0.0, 6),
            "marked_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        }

    reference_x_m = finite_number(reference.get("x_distance_m"))
    reference_y_m = finite_number(reference.get("y_distance_m"))
    if reference_x_m is None or reference_y_m is None:
        clear_recaptured_ultrasonic_state(state)
        return reference, False

    state[ULTRASONIC_X_DISTANCE_KEY] = round(x_m, 6)
    state[ULTRASONIC_Y_DISTANCE_KEY] = round(y_m, 6)
    state[ULTRASONIC_X_POSITION_KEY] = round(x_m - reference_x_m, 6)
    state[ULTRASONIC_Y_POSITION_KEY] = round(y_m - reference_y_m, 6)
    return reference, True


def append_unique_sensor_keys(payload: dict[str, object]) -> None:
    existing = payload.get("sensor_state_keys")
    if not isinstance(existing, list):
        existing = []
    normalized = [item for item in existing if isinstance(item, str)]
    for key in (*ULTRASONIC_STATE_KEYS, *ULTRASONIC_POSITION_KEYS):
        if key not in normalized:
            normalized.append(key)
    payload["sensor_state_keys"] = normalized


def write_recaptured_ultrasonic_stream(
    path: Path,
    payload: dict[str, object],
    samples: list[dict[str, object]],
    reference: dict[str, object],
) -> None:
    payload["samples"] = samples
    append_unique_sensor_keys(payload)
    payload["ultrasonic_position_reference"] = reference
    sample_times = [
        parsed
        for parsed in (
            finite_number(sample.get("t_s")) for sample in samples if isinstance(sample, dict)
        )
        if parsed is not None
    ]
    if sample_times:
        payload["duration_s"] = round(max(sample_times), 6)

    tmp_path = path.with_name(f".{path.name}.tmp")
    tmp_path.write_text(json.dumps(payload, indent=2) + "\n")
    tmp_path.replace(path)


def build_action(state: dict[str, float], include_base: bool) -> dict[str, float]:
    action = {key: float(state[key]) for key in ARM_STATE_KEYS}
    if include_base:
        action.update({key: float(state[key]) for key in BASE_STATE_KEYS})
    else:
        action.update(dict.fromkeys(BASE_STATE_KEYS, 0.0))
    return action


def validate_home_position(value: object) -> dict[str, float]:
    if not isinstance(value, dict):
        raise ValueError("home position must be a JSON object")
    home_position: dict[str, float] = {}
    for key in ARM_STATE_KEYS:
        raw_value = value.get(key)
        if not isinstance(raw_value, (int, float)):
            raise ValueError(f"home position is missing numeric {key}")
        home_position[key] = float(raw_value)
    return home_position


def load_home_position(args: argparse.Namespace) -> dict[str, float] | None:
    if args.home_mode == "none":
        return None
    if not args.home_position_json.strip():
        raise SystemExit("--home-position-json is required when --home-mode is not none.")
    try:
        return validate_home_position(json.loads(args.home_position_json))
    except Exception as exc:
        raise SystemExit(f"Invalid --home-position-json: {exc}") from exc


def extract_home_position(observation: dict[str, object]) -> dict[str, float]:
    return validate_home_position({key: observation.get(key) for key in ARM_STATE_KEYS})


def home_motion_errors(
    observation: dict[str, object],
    home_position: dict[str, float],
) -> dict[str, float]:
    errors: dict[str, float] = {}
    for key in ARM_HOME_MOTION_KEYS:
        value = observation.get(key)
        if isinstance(value, (int, float)) and math.isfinite(float(value)):
            errors[key] = abs(float(value) - home_position[key])
        else:
            errors[key] = float("inf")
    return errors


def max_home_motion_error(errors: dict[str, float]) -> float:
    finite_errors = [error for error in errors.values() if math.isfinite(error)]
    return max(finite_errors, default=float("inf"))


def format_home_motion_errors(errors: dict[str, float]) -> str:
    if not errors:
        return "no joint observations"
    return ", ".join(
        f"{key}={error:.2f}"
        if math.isfinite(error)
        else f"{key}=unavailable"
        for key, error in errors.items()
    )


def move_arm_to_home(
    home_position: dict[str, float],
    robot: LeKiwi,
    observation_reader: ResilientObservationReader,
    sensor_status_emitter: LiveRobotSensorStatusEmitter,
    servo_protection: ServoProtectionSupervisor,
    torque_watcher: TorqueLimitFileWatcher,
    safety_filter: ArmSafetyFilter,
    power_logger: object,
    vex_base_bridge: VexBaseBridge,
    *,
    allow_legacy_base: bool,
    loop_hz: float = 30.0,
) -> dict[str, float] | None:
    if servo_protection.latched:
        print("Go-home skipped because the arm safety latch is active.", flush=True)
        return None

    observation = observation_reader.get_observation()
    current_position = extract_home_position(observation)
    safety_filter.seed_from_observation(observation)
    max_steps = 1
    for key in ARM_HOME_MOTION_KEYS:
        target = home_position[key]
        step_limit = DEFAULT_SAFER_ARM_MAX_STEP.get(key, 8.0)
        if step_limit > 0:
            max_steps = max(max_steps, math.ceil(abs(target - current_position[key]) / step_limit))
    max_steps = max(1, min(600, max_steps))
    sleep_step_s = 1.0 / max(loop_hz, 1.0)
    last_action: dict[str, float] | None = None
    final_errors: dict[str, float] = {}

    for step_index in range(1, max_steps + 1):
        progress = step_index / max_steps
        target_action = {
            key: current_position[key] + (home_position[key] - current_position[key]) * progress
            for key in ARM_HOME_MOTION_KEYS
        }
        target_action.update(dict.fromkeys(BASE_STATE_KEYS, 0.0))
        torque_watcher.poll(robot)
        action = safety_filter.normalize(target_action)
        sent_action = apply_robot_action(
            robot,
            action,
            allow_legacy_base=allow_legacy_base,
        )
        safety_filter.update(sent_action)
        servo_protection.record_command(sent_action)
        last_action = sent_action
        vex_base_bridge.send_hold()
        if hasattr(power_logger, "maybe_sample"):
            power_logger.maybe_sample()
        observation = observation_reader.get_observation()
        observation = vex_base_bridge.merge_observation(observation)
        observation = servo_protection.enrich_observation(observation)
        sensor_status_emitter.emit(
            observation_reader,
            observation,
            source="replay-home",
            vex_base_bridge=vex_base_bridge,
        )
        if servo_protection.observe(observation, getattr(power_logger, "last_sample", None)):
            print("Go-home stopped because the arm safety latch tripped.", flush=True)
            return None
        precise_sleep(sleep_step_s)

    settle_deadline = time.perf_counter() + HOME_SETTLE_TIMEOUT_S
    while True:
        final_errors = home_motion_errors(observation, home_position)
        if max_home_motion_error(final_errors) <= HOME_POSITION_TOLERANCE_DEG:
            break

        if time.perf_counter() >= settle_deadline:
            print(
                "Go-home target was commanded but not reached within "
                f"{HOME_SETTLE_TIMEOUT_S:.1f}s; errors: {format_home_motion_errors(final_errors)}",
                flush=True,
            )
            return None

        target_action = {key: home_position[key] for key in ARM_HOME_MOTION_KEYS}
        target_action.update(dict.fromkeys(BASE_STATE_KEYS, 0.0))
        torque_watcher.poll(robot)
        action = safety_filter.normalize(target_action)
        sent_action = apply_robot_action(
            robot,
            action,
            allow_legacy_base=allow_legacy_base,
        )
        safety_filter.update(sent_action)
        servo_protection.record_command(sent_action)
        last_action = sent_action
        vex_base_bridge.send_hold()
        if hasattr(power_logger, "maybe_sample"):
            power_logger.maybe_sample()
        observation = observation_reader.get_observation()
        observation = vex_base_bridge.merge_observation(observation)
        observation = servo_protection.enrich_observation(observation)
        sensor_status_emitter.emit(
            observation_reader,
            observation,
            source="replay-home",
            vex_base_bridge=vex_base_bridge,
        )
        if servo_protection.observe(observation, getattr(power_logger, "last_sample", None)):
            print("Go-home stopped because the arm safety latch tripped.", flush=True)
            return None
        precise_sleep(sleep_step_s)

    print(
        f"Go-home reached saved arm pose without commanding the gripper (max joint error {max_home_motion_error(final_errors):.2f}deg).",
        flush=True,
    )
    return last_action


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
    if args.vex_positioning_timeout_s <= 0:
        raise SystemExit("--vex-positioning-timeout-s must be greater than 0.")
    if args.vex_positioning_xy_tolerance_m <= 0:
        raise SystemExit("--vex-positioning-xy-tolerance-m must be greater than 0.")
    if args.vex_positioning_heading_tolerance_deg <= 0:
        raise SystemExit("--vex-positioning-heading-tolerance-deg must be greater than 0.")
    if args.vex_positioning_xy_trim_tolerance_m <= 0:
        raise SystemExit("--vex-positioning-xy-trim-tolerance-m must be greater than 0.")
    if args.vex_positioning_heading_trim_tolerance_deg <= 0:
        raise SystemExit("--vex-positioning-heading-trim-tolerance-deg must be greater than 0.")
    home_position = load_home_position(args)

    trajectory_path, trajectory_payload, samples = load_trajectory(args.input)
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
    auto_positioning_available = (
        not args.enable_base
        and isinstance(start_state, dict)
        and recorded_state_has_sensor_reference(start_state)
        and not args.recapture_ultrasonic_stream
    )
    auto_preposition_base = args.auto_vex_positioning and auto_positioning_available
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
                    timeout_s=args.vex_positioning_timeout_s,
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
    if auto_positioning_available and not args.auto_vex_positioning:
        print_vex_position_status(
            "skipped",
            reason="disabled",
            message="VEX start positioning is disabled for this replay.",
            timeout_s=args.vex_positioning_timeout_s,
        )
    if args.recapture_ultrasonic_stream:
        print("X/Y ultrasonic stream recapture: enabled; recording will be overwritten after full replay.")
    if args.safer_servo_mode:
        print("Safer servo mode enabled for replay.")
    if args.home_mode != "none":
        print(f"Go-home mode: {args.home_mode}")

    last_action: dict[str, float] | None = None
    replay_state = None
    last_base_command_at: float | None = None
    vex_base_control_used = False
    if home_position is not None and args.home_mode in {"start", "both"}:
        print("Moving arm to saved home before replay.", flush=True)
        home_action = move_arm_to_home(
            home_position,
            robot,
            observation_reader,
            sensor_status_emitter,
            servo_protection,
            torque_watcher,
            safety_filter,
            power_logger,
            vex_base_bridge,
            allow_legacy_base=args.enable_base,
        )
        if home_action is None:
            raise SystemExit("Failed to move arm home before replay.")
        last_action = home_action

    if prepare_vex_base:
        replay_state = SensorAwareReplayState(
            samples,
            replay_mode=args.vex_replay_mode,
            speed=args.speed,
            control_config=vex_control_config,
        )
        if (
            args.auto_vex_positioning
            and isinstance(start_state, dict)
            and not args.recapture_ultrasonic_stream
        ):
            if auto_preposition_base:
                print("Aligning VEX base to recorded ultrasonic/gyro start pose.", flush=True)
            prepositioned = preposition_vex_base_to_recorded_state(
                vex_base_bridge,
                observation_reader,
                replay_state,
                start_state,
                timeout_s=args.vex_positioning_timeout_s,
                xy_tolerance_m=args.vex_positioning_xy_tolerance_m,
                heading_tolerance_deg=args.vex_positioning_heading_tolerance_deg,
                xy_trim_tolerance_m=args.vex_positioning_xy_trim_tolerance_m,
                heading_trim_tolerance_deg=args.vex_positioning_heading_trim_tolerance_deg,
                sensor_status_emitter=sensor_status_emitter,
                sensor_status_source="replay-preposition",
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
                    timeout_s=args.vex_positioning_timeout_s,
                    arm_replay="aborted",
                )
                raise SystemExit(1)
            elif auto_preposition_base:
                print_vex_position_status(
                    "aligned",
                    message="VEX start positioning completed before arm replay.",
                    timeout_s=args.vex_positioning_timeout_s,
                )
        replay_state.prepare()

    start = time.perf_counter()
    recaptured_reference: dict[str, object] | None = None
    recaptured_sample_count = 0
    recapture_missing_count = 0
    replay_completed = False
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
            if args.recapture_ultrasonic_stream:
                recaptured_reference, sample_recaptured = recapture_ultrasonic_sample(
                    sample,
                    observation,
                    recaptured_reference,
                )
                if sample_recaptured:
                    recaptured_sample_count += 1
                else:
                    recapture_missing_count += 1
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
        else:
            replay_completed = True

        if args.recapture_ultrasonic_stream:
            if not replay_completed:
                print("Ultrasonic stream recapture skipped because replay did not complete.", flush=True)
            elif recaptured_reference is None or recaptured_sample_count <= 0:
                raise RuntimeError("No valid live X/Y ultrasonic samples were captured during replay.")
            else:
                recaptured_reference["samples_recaptured"] = recaptured_sample_count
                recaptured_reference["samples_missing"] = recapture_missing_count
                recaptured_reference["sample_count"] = len(samples)
                write_recaptured_ultrasonic_stream(
                    trajectory_path,
                    trajectory_payload,
                    samples,
                    recaptured_reference,
                )
                print(
                    "Overwrote recording X/Y ultrasonic stream "
                    f"with {recaptured_sample_count}/{len(samples)} replay samples.",
                    flush=True,
                )

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
        if home_position is not None and args.home_mode in {"end", "both"}:
            print("Moving arm to saved home after replay.", flush=True)
            home_action = move_arm_to_home(
                home_position,
                robot,
                observation_reader,
                sensor_status_emitter,
                servo_protection,
                torque_watcher,
                safety_filter,
                power_logger,
                vex_base_bridge,
                allow_legacy_base=args.enable_base,
            )
            if home_action is not None:
                last_action = home_action
            else:
                print("Warning: failed to move arm home after replay.", flush=True)
    except KeyboardInterrupt:
        print("\nReplay interrupted.")
    finally:
        try:
            if last_action is not None and args.enable_base:
                robot.send_action(
                    {
                        **{
                            key: value
                            for key, value in last_action.items()
                            if key.endswith(".pos") and isinstance(value, (int, float))
                        },
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
