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

from lekiwi_runtime import (
    ArmSafetyFilter,
    TorqueLimitFileWatcher,
    add_servo_safety_args,
    add_torque_limit_args,
    apply_torque_limits,
    configure_wrist_roll_mode,
    parse_torque_limits_json,
)
from lerobot.datasets.feature_utils import build_dataset_frame, hw_to_dataset_features
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.processor import make_default_processors
from lerobot.robots.lekiwi import LeKiwi, LeKiwiConfig
from lerobot.utils.constants import ACTION, OBS_STR

logging.basicConfig(level=logging.WARNING)
logger = logging.getLogger("lekiwi_record_dataset_host")

if "PowerTelemetryLogger" not in globals():
    helper_path = Path(__file__).with_name("lekiwi_power.py")
    if helper_path.exists():
        _helper_ns = runpy.run_path(str(helper_path))
        PowerTelemetryLogger = _helper_ns["PowerTelemetryLogger"]
        add_power_monitor_args = _helper_ns["add_power_monitor_args"]
    else:
        raise RuntimeError("Power telemetry helper is unavailable.")

BASE_ACTION_KEYS = ("x.vel", "y.vel", "theta.vel")
ARM_ACTION_KEYS = (
    "arm_shoulder_pan.pos",
    "arm_shoulder_lift.pos",
    "arm_elbow_flex.pos",
    "arm_wrist_flex.pos",
    "arm_wrist_roll.pos",
    "arm_gripper.pos",
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


def configure_cameras(robot_config: LeKiwiConfig, cameras_json: str) -> None:
    value = cameras_json.strip()
    if value in {"", "default", "__default__", "null"}:
        return

    parsed = json.loads(value)
    if parsed == {}:
        robot_config.cameras = {}
        return

    raise SystemExit("Only '{}' or 'default' are currently supported for --robot-cameras-json.")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Pi-side LeKiwi dataset capture host. It records a local LeRobot dataset while "
            "accepting leader-arm actions from the Mac over ZMQ, or free-teaching directly on the Pi."
        )
    )
    parser.add_argument("--robot-id", "--robot.id", dest="robot_id", default="follow-mobile")
    parser.add_argument("--robot-port", "--robot.port", dest="robot_port", default="/dev/ttyACM0")
    parser.add_argument("--robot-cameras-json", "--robot.cameras", dest="robot_cameras_json", default="default")
    parser.add_argument("--use-degrees", "--robot.use_degrees", dest="use_degrees", type=parse_bool, default=True)
    parser.add_argument("--base-max-raw-velocity", "--robot.base_max_raw_velocity", dest="base_max_raw_velocity", type=int, default=3000)
    parser.add_argument("--base-wheel-torque-limit", "--robot.base_wheel_torque_limit", dest="base_wheel_torque_limit", type=int, default=None)
    parser.add_argument("--enable-base", "--robot.enable_base", dest="enable_base", type=parse_bool, default=False)
    parser.add_argument("--port-zmq-cmd", "--host.port_zmq_cmd", dest="port_zmq_cmd", type=int, default=5555)
    parser.add_argument("--port-zmq-observations", "--host.port_zmq_observations", dest="port_zmq_observations", type=int, default=5556)
    parser.add_argument("--connection-time-s", "--host.connection_time_s", dest="connection_time_s", type=int, default=3600)
    parser.add_argument("--watchdog-timeout-ms", "--host.watchdog_timeout_ms", dest="watchdog_timeout_ms", type=int, default=500)
    parser.add_argument("--loop-hz", "--host.max_loop_freq_hz", dest="loop_hz", type=float, default=30.0)
    parser.add_argument("--capture-mode", choices=("leader", "free-teach"), default="leader")
    parser.add_argument("--dataset-repo-id", required=True)
    parser.add_argument("--dataset-root", required=True)
    parser.add_argument("--task", required=True)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--num-episodes", type=int, default=50)
    parser.add_argument("--episode-time-s", type=int, default=20)
    parser.add_argument("--reset-time-s", type=int, default=10)
    parser.add_argument("--dataset-vcodec", default="h264")
    parser.add_argument("--dataset-streaming-encoding", type=parse_bool, default=False)
    parser.add_argument("--encoder-threads", type=int, default=2)
    parser.add_argument("--image-writer-threads", type=int, default=4)
    add_servo_safety_args(parser)
    add_torque_limit_args(parser)
    add_power_monitor_args(parser)
    return parser.parse_args()


def make_zero_action() -> dict[str, float]:
    return {
        **{key: 0.0 for key in ARM_ACTION_KEYS},
        **{key: 0.0 for key in BASE_ACTION_KEYS},
    }


def make_free_teach_action(observation: dict[str, float]) -> dict[str, float]:
    return {
        **{key: float(observation.get(key, 0.0)) for key in ARM_ACTION_KEYS},
        **{key: 0.0 for key in BASE_ACTION_KEYS},
    }


def load_or_create_dataset(args: argparse.Namespace, robot: LeKiwi) -> LeRobotDataset:
    dataset_root = Path(args.dataset_root).expanduser()
    dataset_root.parent.mkdir(parents=True, exist_ok=True)

    action_features = hw_to_dataset_features(robot.action_features, ACTION)
    obs_features = hw_to_dataset_features(robot.observation_features, OBS_STR)
    dataset_features = {**action_features, **obs_features}
    meta_info = dataset_root / "meta" / "info.json"

    if meta_info.exists():
        return LeRobotDataset.resume(
            repo_id=args.dataset_repo_id,
            root=dataset_root,
            vcodec=args.dataset_vcodec,
            streaming_encoding=args.dataset_streaming_encoding,
            image_writer_threads=args.image_writer_threads,
            encoder_threads=args.encoder_threads,
        )

    return LeRobotDataset.create(
        repo_id=args.dataset_repo_id,
        root=dataset_root,
        fps=args.fps,
        features=dataset_features,
        robot_type=robot.name,
        use_videos=True,
        image_writer_threads=args.image_writer_threads,
        vcodec=args.dataset_vcodec,
        streaming_encoding=args.dataset_streaming_encoding,
        encoder_threads=args.encoder_threads,
    )


def main() -> None:
    args = parse_args()

    robot_config = build_robot_config(args)
    configure_cameras(robot_config, args.robot_cameras_json)
    robot = LeKiwi(robot_config)
    robot.connect(calibrate=False)
    configure_wrist_roll_mode(robot, continuous=args.safer_servo_mode)
    apply_torque_limits(robot, parse_torque_limits_json(args.torque_limits_json))
    torque_watcher = TorqueLimitFileWatcher(args.torque_limits_path)
    torque_watcher.poll(robot, force=True)
    safety_filter = ArmSafetyFilter(
        enabled=args.safer_servo_mode,
        map_wrist_to_follower_start=args.capture_mode == "leader",
    )
    try:
        safety_filter.seed_from_observation(robot.get_observation())
    except Exception as exc:
        logger.warning("Failed to seed servo safety filter from the current robot state: %s", exc)
    if args.capture_mode == "free-teach":
        robot.bus.disable_torque(robot.arm_motors, num_retry=10)

    dataset = load_or_create_dataset(args, robot)
    _, _, robot_observation_processor = make_default_processors()

    power_logger = PowerTelemetryLogger(robot, args, logger, "lekiwi_record_dataset_host")
    power_logger.start()

    ctx = zmq.Context()
    cmd_socket = ctx.socket(zmq.PULL)
    cmd_socket.setsockopt(zmq.CONFLATE, 1)
    cmd_socket.bind(f"tcp://*:{args.port_zmq_cmd}")

    obs_socket = ctx.socket(zmq.PUSH)
    obs_socket.setsockopt(zmq.CONFLATE, 1)
    obs_socket.bind(f"tcp://*:{args.port_zmq_observations}")

    print(
        f"LeKiwi dataset capture host listening on tcp://*:{args.port_zmq_cmd} and tcp://*:{args.port_zmq_observations}",
        flush=True,
    )
    print(f"Capture mode: {args.capture_mode}", flush=True)
    print(f"Dataset root: {Path(args.dataset_root).expanduser()}", flush=True)
    print(f"Target episodes: {args.num_episodes} at {args.fps} fps", flush=True)
    if args.capture_mode == "leader":
        print("Waiting for leader commands from the Mac before starting episode 1.", flush=True)
    else:
        print("Free-teach mode: recording starts immediately and arm torque stays disabled.", flush=True)
    if args.safer_servo_mode:
        print("Safer servo mode enabled for dataset capture.", flush=True)

    current_action = make_zero_action()
    episodes_recorded = dataset.num_episodes
    if episodes_recorded >= args.num_episodes:
        print(
            f"Dataset already contains {episodes_recorded} episode(s), which meets the target. Nothing to record.",
            flush=True,
        )
        dataset.finalize()
        robot.disconnect()
        power_logger.close()
        cmd_socket.close()
        obs_socket.close()
        ctx.term()
        return
    episode_start = time.perf_counter()
    reset_start = time.perf_counter()
    phase = "armed" if args.capture_mode == "leader" and episodes_recorded == 0 else "recording"
    start = time.perf_counter()
    first_command_seen = args.capture_mode == "free-teach"
    last_cmd_time = time.time()
    watchdog_active = False

    try:
        while time.perf_counter() - start < args.connection_time_s:
            loop_start = time.perf_counter()

            try:
                raw = cmd_socket.recv_string(zmq.NOBLOCK)
                leader_action = safety_filter.normalize(dict(json.loads(raw)))
                current_action = {
                    **make_zero_action(),
                    **leader_action,
                }
                last_cmd_time = time.time()
                watchdog_active = False
                if args.capture_mode == "leader" and not first_command_seen:
                    first_command_seen = True
                    phase = "recording"
                    episode_start = time.perf_counter()
                    print(
                        f"Episode {episodes_recorded + 1}/{args.num_episodes} started after the first leader command.",
                        flush=True,
                    )
            except zmq.Again:
                pass
            except Exception as exc:
                logger.warning("Command fetch failed: %s", exc)

            observation = robot.get_observation()
            torque_watcher.poll(robot)

            now = time.time()
            if (now - last_cmd_time > args.watchdog_timeout_ms / 1000.0) and not watchdog_active:
                robot.stop_base()
                watchdog_active = True

            if args.capture_mode == "free-teach":
                current_action = make_free_teach_action(observation)

            if phase == "recording":
                sent_action = robot.send_action(safety_filter.normalize(current_action))
                safety_filter.update(sent_action)
                processed_obs = robot_observation_processor(observation)
                frame = {
                    **build_dataset_frame(dataset.features, processed_obs, prefix=OBS_STR),
                    **build_dataset_frame(dataset.features, sent_action, prefix=ACTION),
                    "task": args.task,
                }
                dataset.add_frame(frame)

                elapsed_episode = time.perf_counter() - episode_start
                if elapsed_episode >= args.episode_time_s:
                    dataset.save_episode()
                    episodes_recorded += 1
                    print(
                        f"Saved episode {episodes_recorded}/{args.num_episodes} at {elapsed_episode:.2f}s.",
                        flush=True,
                    )
                    if episodes_recorded >= args.num_episodes:
                        break
                    phase = "reset"
                    reset_start = time.perf_counter()
                    print(
                        f"Reset window started for {args.reset_time_s}s before episode {episodes_recorded + 1}.",
                        flush=True,
                    )
            else:
                sent_action = robot.send_action(safety_filter.normalize(current_action))
                safety_filter.update(sent_action)
                if time.perf_counter() - reset_start >= args.reset_time_s:
                    phase = "recording"
                    episode_start = time.perf_counter()
                    print(
                        f"Episode {episodes_recorded + 1}/{args.num_episodes} started.",
                        flush=True,
                    )

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

            power_logger.maybe_sample()
            remaining = max(1.0 / args.loop_hz - (time.perf_counter() - loop_start), 0.0)
            if remaining > 0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        print("\nStopping dataset capture host.", flush=True)
    finally:
        try:
            power_logger.maybe_sample(force=True)
        except Exception:
            pass
        try:
            dataset.finalize()
        except Exception as exc:
            print(f"dataset finalize: {exc}", flush=True)
        try:
            robot.disconnect()
        except Exception as exc:
            print(f"disconnect: {exc}", flush=True)
        power_logger.close()
        cmd_socket.close()
        obs_socket.close()
        ctx.term()

    print(f"Dataset capture complete. Episodes now stored: {dataset.num_episodes}", flush=True)


if __name__ == "__main__":
    main()
