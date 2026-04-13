#!/usr/bin/env python3

import argparse
import json
import time
from pathlib import Path

from lerobot.datasets.feature_utils import hw_to_dataset_features
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.policies.factory import make_pre_post_processors
from lerobot.processor import make_default_processors
from lerobot.robots.lekiwi import LeKiwi, LeKiwiConfig
from lerobot.scripts.lerobot_record import record_loop
from lerobot.utils.constants import ACTION, OBS_STR


def parse_bool(value: str) -> bool:
    normalized = value.strip().lower()
    if normalized in {"1", "true", "yes", "on"}:
        return True
    if normalized in {"0", "false", "no", "off"}:
        return False
    raise argparse.ArgumentTypeError("expected true or false")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run a deployed ACT policy directly on the Pi and save eval episodes.")
    parser.add_argument("--robot-id", "--robot.id", dest="robot_id", default="follow-mobile")
    parser.add_argument("--robot-port", "--robot.port", dest="robot_port", default="/dev/ttyACM0")
    parser.add_argument("--robot-cameras-json", "--robot.cameras", dest="robot_cameras_json", default="default")
    parser.add_argument("--use-degrees", "--robot.use_degrees", dest="use_degrees", type=parse_bool, default=True)
    parser.add_argument("--enable-base", "--robot.enable_base", dest="enable_base", type=parse_bool, default=False)
    parser.add_argument("--policy-path", required=True)
    parser.add_argument("--dataset-repo-id", required=True)
    parser.add_argument("--dataset-root", required=True)
    parser.add_argument("--task", required=True)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--num-episodes", type=int, default=10)
    parser.add_argument("--episode-time-s", type=int, default=20)
    parser.add_argument("--reset-time-s", type=int, default=10)
    parser.add_argument("--dataset-vcodec", default="h264")
    parser.add_argument("--dataset-streaming-encoding", type=parse_bool, default=False)
    parser.add_argument("--encoder-threads", type=int, default=2)
    parser.add_argument("--image-writer-threads", type=int, default=4)
    parser.add_argument("--display-data", type=parse_bool, default=False)
    parser.add_argument("--target-effective-fps", type=float, default=0.0)
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


def main() -> None:
    args = parse_args()

    dataset_root = Path(args.dataset_root).expanduser()
    dataset_root.parent.mkdir(parents=True, exist_ok=True)

    robot_config = LeKiwiConfig(
        id=args.robot_id,
        port=args.robot_port,
        use_degrees=args.use_degrees,
        enable_base=args.enable_base,
    )
    configure_cameras(robot_config, args.robot_cameras_json)
    robot = LeKiwi(robot_config)

    policy = ACTPolicy.from_pretrained(args.policy_path)
    preprocessor, postprocessor = make_pre_post_processors(
        policy_cfg=policy,
        pretrained_path=args.policy_path,
        preprocessor_overrides={"device_processor": {"device": str(policy.config.device)}},
    )
    teleop_action_processor, robot_action_processor, robot_observation_processor = make_default_processors()

    action_features = hw_to_dataset_features(robot.action_features, ACTION)
    obs_features = hw_to_dataset_features(robot.observation_features, OBS_STR)
    dataset_features = {**action_features, **obs_features}

    dataset = LeRobotDataset.create(
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

    robot.connect()
    print(f"Running policy from {args.policy_path}", flush=True)
    print(f"Saving evaluation episodes to {dataset_root}", flush=True)
    if args.target_effective_fps > 0:
        print(f"Benchmark gate effective fps: {args.target_effective_fps:.3f}", flush=True)

    events = {
        "stop_recording": False,
        "rerecord_episode": False,
        "exit_early": False,
    }

    try:
        for episode_idx in range(args.num_episodes):
            print(f"Starting eval episode {episode_idx + 1}/{args.num_episodes}.", flush=True)
            episode_start = time.perf_counter()
            record_loop(
                robot=robot,
                events=events,
                fps=args.fps,
                teleop_action_processor=teleop_action_processor,
                robot_action_processor=robot_action_processor,
                robot_observation_processor=robot_observation_processor,
                dataset=dataset,
                policy=policy,
                preprocessor=preprocessor,
                postprocessor=postprocessor,
                control_time_s=args.episode_time_s,
                single_task=args.task,
                display_data=args.display_data,
            )
            dataset.save_episode()
            print(
                f"Saved eval episode {episode_idx + 1}/{args.num_episodes} after "
                f"{time.perf_counter() - episode_start:.2f}s.",
                flush=True,
            )
            if episode_idx < args.num_episodes - 1 and args.reset_time_s > 0:
                print(f"Reset window: {args.reset_time_s}s before the next eval episode.", flush=True)
                time.sleep(args.reset_time_s)
    finally:
        dataset.finalize()
        if robot.is_connected:
            robot.disconnect()

    print(f"Saved {dataset.num_episodes} eval episodes at {dataset_root}", flush=True)


if __name__ == "__main__":
    main()
