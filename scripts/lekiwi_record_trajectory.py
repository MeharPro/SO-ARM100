#!/usr/bin/env python3

from __future__ import annotations

import argparse
import base64
import json
import logging
import runpy
import time
from datetime import datetime
from pathlib import Path

import cv2
import zmq

from lekiwi_runtime import (
    ArmSafetyFilter,
    LiveRobotSensorStatusEmitter,
    ULTRASONIC_STATE_KEYS,
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
from vex_base_bridge import (
    VEX_INERTIAL_STATE_KEYS,
    VEX_MOTOR_STATE_KEYS,
    VEX_POSE_STATE_KEYS,
    VexBaseBridge,
    VexBaseTelemetryManager,
    add_vex_base_args,
    ensure_vex_command_stream,
    normalize_vex_control_config,
)
from lerobot.robots.lekiwi import LeKiwi, LeKiwiConfig

logging.basicConfig(level=logging.WARNING)
logger = logging.getLogger("lekiwi_record_trajectory")

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
DEFAULT_RECORD_DIR = Path("~/lekiwi-trajectories").expanduser()


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
        description=(
            "Pi-side LeKiwi recording host. It behaves like the normal ZMQ host, "
            "but it also records the follower's observed state to a trajectory file."
        )
    )
    parser.add_argument("--robot-id", "--robot.id", dest="robot_id", default="follow-mobile")
    parser.add_argument("--robot-port", "--robot.port", dest="robot_port", default="/dev/ttyACM0")
    parser.add_argument("--robot-cameras-json", "--robot.cameras", dest="robot_cameras_json", default="{}")
    parser.add_argument("--use-degrees", "--robot.use_degrees", dest="use_degrees", type=parse_bool, default=True)
    parser.add_argument("--base-max-raw-velocity", "--robot.base_max_raw_velocity", dest="base_max_raw_velocity", type=int, default=3000)
    parser.add_argument("--base-wheel-torque-limit", "--robot.base_wheel_torque_limit", dest="base_wheel_torque_limit", type=int, default=None)
    parser.add_argument("--enable-base", "--robot.enable_base", dest="enable_base", type=parse_bool, default=True)
    parser.add_argument("--port-zmq-cmd", "--host.port_zmq_cmd", dest="port_zmq_cmd", type=int, default=5555)
    parser.add_argument(
        "--port-zmq-observations", "--host.port_zmq_observations", dest="port_zmq_observations", type=int, default=5556
    )
    parser.add_argument("--connection-time-s", "--host.connection_time_s", dest="connection_time_s", type=int, default=600)
    parser.add_argument(
        "--watchdog-timeout-ms", "--host.watchdog_timeout_ms", dest="watchdog_timeout_ms", type=int, default=500
    )
    parser.add_argument("--loop-hz", "--host.max_loop_freq_hz", dest="loop_hz", type=float, default=30.0)
    parser.add_argument("--output", default=None, help="Trajectory JSON output path. Defaults to ~/lekiwi-trajectories.")
    parser.add_argument("--label", default="", help="Optional label stored in the trajectory metadata.")
    parser.add_argument(
        "--recording-mode",
        choices=("leader", "keyboard", "free-teach"),
        default="leader",
        help=(
            "leader/keyboard wait for the first external command before recording. "
            "free-teach disables follower arm torque and records observed arm poses immediately."
        ),
    )
    parser.add_argument("--print-every", type=int, default=30, help="Print every N recorded samples.")
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


def build_output_path(requested: str | None) -> Path:
    if requested:
        path = Path(requested).expanduser()
    else:
        DEFAULT_RECORD_DIR.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        path = DEFAULT_RECORD_DIR / f"trajectory-{timestamp}.json"

    path.parent.mkdir(parents=True, exist_ok=True)
    return path


def build_sample(observation: dict[str, float], t_s: float) -> dict[str, object]:
    state = {key: float(observation.get(key, 0.0)) for key in ARM_STATE_KEYS + BASE_STATE_KEYS}
    for key in VEX_MOTOR_STATE_KEYS:
        value = observation.get(key)
        if isinstance(value, (int, float)):
            state[key] = float(value)
    for key in VEX_INERTIAL_STATE_KEYS:
        value = observation.get(key)
        if isinstance(value, (int, float)):
            state[key] = float(value)
    for key in VEX_POSE_STATE_KEYS:
        value = observation.get(key)
        if isinstance(value, (int, float)):
            state[key] = float(value)
    for key in ULTRASONIC_STATE_KEYS:
        value = observation.get(key)
        if isinstance(value, (int, float)):
            state[key] = float(value)
    return {
        "t_s": round(t_s, 6),
        "state": state,
    }


def build_command_sample(action: dict[str, float], t_s: float) -> dict[str, object]:
    return {
        "t_s": round(t_s, 6),
        "action": {key: float(action.get(key, 0.0)) for key in ARM_STATE_KEYS + BASE_STATE_KEYS},
    }


def summarize_sample(sample: dict[str, object]) -> str:
    state = sample["state"]
    return " ".join(f"{key}={state[key]:7.2f}" for key in ARM_STATE_KEYS)


def write_trajectory(
    path: Path,
    args: argparse.Namespace,
    samples: list[dict[str, object]],
    command_samples: list[dict[str, object]],
) -> None:
    payload = {
        "format": "lekiwi-follower-trajectory",
        "version": 4,
        "created_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "recorded_on": "pi",
        "robot_id": args.robot_id,
        "robot_port": args.robot_port,
        "loop_hz": args.loop_hz,
        "label": args.label,
        "recording_mode": args.recording_mode,
        "duration_s": round(float(samples[-1]["t_s"]), 6) if samples else 0.0,
        "arm_state_keys": list(ARM_STATE_KEYS),
        "base_state_keys": list(BASE_STATE_KEYS),
        "vex_state_keys": list(VEX_MOTOR_STATE_KEYS + VEX_INERTIAL_STATE_KEYS + VEX_POSE_STATE_KEYS),
        "vex_motor_state_keys": list(VEX_MOTOR_STATE_KEYS),
        "vex_inertial_state_keys": list(VEX_INERTIAL_STATE_KEYS),
        "vex_pose_state_keys": list(VEX_POSE_STATE_KEYS),
        "sensor_state_keys": list(ULTRASONIC_STATE_KEYS),
        "samples": samples,
        "command_samples": command_samples,
    }
    path.write_text(json.dumps(payload, indent=2) + "\n")


def restore_free_teach_arm_torque(robot: LeKiwi, observation_reader: ResilientObservationReader) -> None:
    observation = observation_reader.get_observation()
    hold_action = {
        key: float(observation[key])
        for key in ARM_STATE_KEYS
        if isinstance(observation.get(key), (int, float))
    }
    if hold_action:
        apply_robot_action(
            robot,
            {**hold_action, **dict.fromkeys(BASE_STATE_KEYS, 0.0)},
            allow_legacy_base=False,
        )
    robot.bus.enable_torque(robot.arm_motors, num_retry=10)
    print("Follower arm torque restored at the recorded stop pose.", flush=True)


def main() -> None:
    args = parse_args()
    output_path = build_output_path(args.output)

    robot_config = build_robot_config(args)
    configure_cameras(robot_config, args.robot_cameras_json)
    robot = LeKiwi(robot_config)
    power_logger = None
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
        map_wrist_to_follower_start=args.recording_mode != "free-teach",
        absolute_position_limits=absolute_position_limits,
    )
    is_free_teach = args.recording_mode == "free-teach"
    servo_protection = ServoProtectionSupervisor(
        robot,
        logger,
        stall_detection_enabled=not is_free_teach,
    )
    observation_reader = ResilientObservationReader(robot, logger)
    sensor_status_emitter = LiveRobotSensorStatusEmitter()
    try:
        observation = observation_reader.get_observation()
        safety_filter.seed_from_observation(observation)
        servo_protection.seed_from_observation(observation)
    except Exception as exc:
        logger.warning("Failed to seed servo safety filter from the current robot state: %s", exc)
    if is_free_teach:
        robot.bus.disable_torque(robot.arm_motors, num_retry=10)
        print("Follower arm torque disabled for hand-guided recording.", flush=True)
    power_logger = PowerTelemetryLogger(robot, args, logger, "lekiwi_record_trajectory")
    power_logger.start()
    vex_control_config = normalize_vex_control_config(json.loads(args.vex_control_config_json))
    vex_base_bridge = VexBaseBridge(
        requested_port=args.vex_base_port,
        baudrate=args.vex_base_baudrate,
        stale_after_s=args.vex_base_stale_after_s,
        command_timeout_s=args.vex_base_command_timeout_s,
        logger=logger,
    )
    vex_base_bridge.connect()
    vex_base_telemetry = VexBaseTelemetryManager(
        requested_vexcom_path=args.vex_vexcom_path,
        telemetry_slot=args.vex_telemetry_slot,
        cache_dir=args.vex_program_cache_dir,
        logger=logger,
    )
    if args.recording_mode != "free-teach" and vex_base_telemetry.enabled:
        vex_base_telemetry.install_and_run(
            vex_control_config,
            program_name=args.vex_telemetry_program_name,
            run_after_install=True,
        )
        time.sleep(1.0)
        vex_base_bridge.connect()
    vex_command_stream_ready = False
    if args.recording_mode != "free-teach":
        vex_command_stream_ready = ensure_vex_command_stream(
            vex_base_bridge,
            vex_base_telemetry,
            vex_control_config,
            program_name=args.vex_telemetry_program_name,
            logger=logger,
        )
    if args.recording_mode != "free-teach" and not vex_command_stream_ready:
        print(
            "Warning: VEX USB command stream is unavailable; recording will store live sensors but cannot reset a stable inertial origin.",
            flush=True,
        )

    ctx = zmq.Context()
    cmd_socket = ctx.socket(zmq.PULL)
    cmd_socket.setsockopt(zmq.CONFLATE, 1)
    cmd_socket.bind(f"tcp://*:{args.port_zmq_cmd}")

    obs_socket = ctx.socket(zmq.PUSH)
    obs_socket.setsockopt(zmq.CONFLATE, 1)
    obs_socket.bind(f"tcp://*:{args.port_zmq_observations}")

    last_cmd_time = time.time()
    watchdog_active = False
    samples: list[dict[str, object]] = []
    command_samples: list[dict[str, object]] = []

    print(
        f"LeKiwi recording host listening on tcp://*:{args.port_zmq_cmd} and tcp://*:{args.port_zmq_observations}",
        flush=True,
    )
    print(vex_base_bridge.status_message, flush=True)
    print(f"Recording exact follower motion to {output_path}", flush=True)
    if args.recording_mode == "free-teach":
        print("Free-teach recording starts immediately. Move the follower arm by hand, then stop recording.", flush=True)
    else:
        print("Waiting for the first control command before recording samples.", flush=True)
    if args.safer_servo_mode:
        print("Safer servo mode enabled for recording.", flush=True)
    if is_free_teach:
        print("Powered-servo stall detection is bypassed while follower arm torque is disabled.", flush=True)

    session_started_at = time.perf_counter()
    recording_started_at = None
    if args.recording_mode == "free-teach":
        initial_observation = vex_base_bridge.merge_observation(observation_reader.get_observation())
        initial_observation = servo_protection.enrich_observation(initial_observation)
        recording_started_at = time.perf_counter()
        samples.append(build_sample(initial_observation, 0.0))
        print("Recording started.", flush=True)
        print("[00001] t=  0.000s captured initial hand-guided state.", flush=True)
    try:
        while time.perf_counter() - session_started_at < args.connection_time_s:
            loop_start = time.time()

            if args.recording_mode != "free-teach":
                try:
                    msg = cmd_socket.recv_string(zmq.NOBLOCK)
                    action = safety_filter.normalize(dict(json.loads(msg)))
                    if recording_started_at is None:
                        if vex_command_stream_ready:
                            if vex_base_bridge.set_pose_origin(ttl_ms=1200, timeout_s=1.5):
                                print("VEX pose origin reset for this recording.", flush=True)
                            else:
                                print(
                                    "Warning: failed to reset the VEX pose origin; recorded inertial angle may be stale.",
                                    flush=True,
                                )
                        initial_observation = vex_base_bridge.merge_observation(observation_reader.get_observation())
                        initial_observation = servo_protection.enrich_observation(initial_observation)
                        recording_started_at = time.perf_counter()
                        samples.append(build_sample(initial_observation, 0.0))
                        print("First control command received. Recording started.", flush=True)
                        print("[00001] t=  0.000s captured initial pre-move state.", flush=True)
                        command_t_s = 0.0
                    else:
                        command_t_s = time.perf_counter() - recording_started_at
                    sent_action = apply_robot_action(
                        robot,
                        action,
                        allow_legacy_base=args.enable_base,
                    )
                    safety_filter.update(sent_action)
                    servo_protection.record_command(sent_action)
                    command_samples.append(
                        build_command_sample(vex_base_bridge.merge_action(sent_action), command_t_s)
                    )
                    last_cmd_time = time.time()
                    watchdog_active = False
                except zmq.Again:
                    if not watchdog_active:
                        logger.warning("No command available")
                except Exception as exc:
                    logger.warning("Message fetching failed: %s", exc)

            now = time.time()
            if (
                args.recording_mode != "free-teach"
                and (now - last_cmd_time > args.watchdog_timeout_ms / 1000.0)
                and not watchdog_active
            ):
                logger.warning(
                    "Command not received for more than %s milliseconds. Stopping the base.",
                    args.watchdog_timeout_ms,
                )
                stop_robot_base(robot, allow_legacy_base=args.enable_base)
                watchdog_active = True

            if args.recording_mode != "free-teach":
                torque_watcher.poll(robot)

            observation = vex_base_bridge.merge_observation(observation_reader.get_observation())
            observation = servo_protection.enrich_observation(observation)
            sensor_status_emitter.emit(
                observation_reader,
                observation,
                source="recording",
                vex_base_bridge=vex_base_bridge,
            )
            if recording_started_at is not None:
                sample = build_sample(observation, time.perf_counter() - recording_started_at)
                samples.append(sample)
                if len(samples) % args.print_every == 0:
                    print(f"[{len(samples):05d}] t={sample['t_s']:7.3f}s {summarize_sample(sample)}", flush=True)
            power_logger.maybe_sample()
            if servo_protection.observe(observation, power_logger.last_sample):
                stop_robot_base(robot, allow_legacy_base=args.enable_base)
                print("Recording stopped because the arm safety latch tripped.", flush=True)
                break

            encoded_observation = dict(observation)
            for cam_key in robot.cameras:
                frame = encoded_observation.get(cam_key)
                if isinstance(frame, str) or frame is None:
                    encoded_observation[cam_key] = ""
                    continue
                ret, buffer = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
                encoded_observation[cam_key] = base64.b64encode(buffer).decode("utf-8") if ret else ""

            try:
                obs_socket.send_string(json.dumps(encoded_observation), flags=zmq.NOBLOCK)
            except zmq.Again:
                pass
            except Exception as exc:
                logger.warning("Observation publish failed: %s", exc)

            elapsed = time.time() - loop_start
            time.sleep(max(1.0 / args.loop_hz - elapsed, 0.0))
    except KeyboardInterrupt:
        print("\nStopping LeKiwi recording host", flush=True)
    finally:
        if power_logger is not None:
            try:
                power_logger.maybe_sample(force=True)
            except Exception as exc:
                print(f"power telemetry: {exc}", flush=True)
        if samples:
            try:
                write_trajectory(output_path, args, samples, command_samples)
                print(f"Saved {len(samples)} samples to {output_path}", flush=True)
            except Exception as exc:
                print(f"save trajectory: {exc}", flush=True)
        else:
            print("No motion was recorded, so nothing was saved.", flush=True)
        if args.recording_mode == "free-teach" and not servo_protection.latched:
            try:
                restore_free_teach_arm_torque(robot, observation_reader)
            except Exception as exc:
                print(f"restore arm torque: {exc}", flush=True)
        try:
            disconnect_robot(robot, allow_legacy_base=args.enable_base)
        except Exception as exc:
            print(f"disconnect: {exc}", flush=True)
        vex_base_bridge.close()
        if power_logger is not None:
            try:
                power_logger.close()
            except Exception as exc:
                print(f"power telemetry close: {exc}", flush=True)
        cmd_socket.close()
        obs_socket.close()
        ctx.term()


if __name__ == "__main__":
    main()
