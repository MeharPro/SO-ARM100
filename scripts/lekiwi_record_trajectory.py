#!/usr/bin/env python3

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
    TorqueLimitFileWatcher,
    add_servo_safety_args,
    add_torque_limit_args,
    apply_torque_limits,
    build_safer_max_relative_target,
    configure_wrist_roll_mode,
    parse_torque_limits_json,
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
STATE_KEYS = ARM_STATE_KEYS + BASE_STATE_KEYS
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
    if args.safer_servo_mode:
        optional_values["max_relative_target"] = build_safer_max_relative_target()
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
    parser.add_argument("--print-every", type=int, default=30, help="Print every N recorded samples.")
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
    return {
        "t_s": round(t_s, 6),
        "state": {key: float(observation.get(key, 0.0)) for key in STATE_KEYS},
    }


def build_command_sample(action: dict[str, float], t_s: float) -> dict[str, object]:
    return {
        "t_s": round(t_s, 6),
        "action": {key: float(action.get(key, 0.0)) for key in STATE_KEYS},
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
        "version": 2,
        "created_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "recorded_on": "pi",
        "robot_id": args.robot_id,
        "robot_port": args.robot_port,
        "loop_hz": args.loop_hz,
        "label": args.label,
        "duration_s": round(float(samples[-1]["t_s"]), 6) if samples else 0.0,
        "arm_state_keys": list(ARM_STATE_KEYS),
        "base_state_keys": list(BASE_STATE_KEYS),
        "samples": samples,
        "command_samples": command_samples,
    }
    path.write_text(json.dumps(payload, indent=2) + "\n")


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
    safety_filter = ArmSafetyFilter(enabled=args.safer_servo_mode)
    try:
        safety_filter.seed_from_observation(robot.get_observation())
    except Exception as exc:
        logger.warning("Failed to seed servo safety filter from the current robot state: %s", exc)
    power_logger = PowerTelemetryLogger(robot, args, logger, "lekiwi_record_trajectory")
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
    samples: list[dict[str, object]] = []
    command_samples: list[dict[str, object]] = []

    print(
        f"LeKiwi recording host listening on tcp://*:{args.port_zmq_cmd} and tcp://*:{args.port_zmq_observations}",
        flush=True,
    )
    print(f"Recording exact follower motion to {output_path}", flush=True)
    print("Waiting for the first leader command before recording samples.", flush=True)
    if args.safer_servo_mode:
        print("Safer servo mode enabled for recording.", flush=True)

    session_started_at = time.perf_counter()
    recording_started_at = None
    try:
        while time.perf_counter() - session_started_at < args.connection_time_s:
            loop_start = time.time()

            try:
                msg = cmd_socket.recv_string(zmq.NOBLOCK)
                action = safety_filter.normalize(dict(json.loads(msg)))
                if recording_started_at is None:
                    recording_started_at = time.perf_counter()
                    print("First leader command received. Recording started.", flush=True)
                    command_t_s = 0.0
                else:
                    command_t_s = time.perf_counter() - recording_started_at
                sent_action = robot.send_action(action)
                safety_filter.update(sent_action)
                command_samples.append(build_command_sample(sent_action, command_t_s))
                last_cmd_time = time.time()
                watchdog_active = False
            except zmq.Again:
                if not watchdog_active:
                    logger.warning("No command available")
            except Exception as exc:
                logger.warning("Message fetching failed: %s", exc)

            now = time.time()
            if (now - last_cmd_time > args.watchdog_timeout_ms / 1000.0) and not watchdog_active:
                logger.warning(
                    "Command not received for more than %s milliseconds. Stopping the base.",
                    args.watchdog_timeout_ms,
                )
                robot.stop_base()
                watchdog_active = True

            torque_watcher.poll(robot)

            observation = robot.get_observation()
            if recording_started_at is not None:
                sample = build_sample(observation, time.perf_counter() - recording_started_at)
                samples.append(sample)
                if len(samples) == 1 or len(samples) % args.print_every == 0:
                    print(f"[{len(samples):05d}] t={sample['t_s']:7.3f}s {summarize_sample(sample)}", flush=True)
            power_logger.maybe_sample()

            encoded_observation = dict(observation)
            for cam_key in robot.cameras:
                ret, buffer = cv2.imencode(".jpg", encoded_observation[cam_key], [int(cv2.IMWRITE_JPEG_QUALITY), 90])
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
            print("No leader-driven motion was recorded, so nothing was saved.", flush=True)
        try:
            robot.disconnect()
        except Exception as exc:
            print(f"disconnect: {exc}", flush=True)
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
