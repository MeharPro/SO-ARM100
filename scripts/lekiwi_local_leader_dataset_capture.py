#!/usr/bin/env python3

import argparse
import time
from pathlib import Path

from lerobot.datasets.feature_utils import build_dataset_frame, hw_to_dataset_features
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from lerobot.utils.constants import ACTION, OBS_STR

ARM_KEY_MAP = (
    ("shoulder_pan.pos", "arm_shoulder_pan.pos"),
    ("shoulder_lift.pos", "arm_shoulder_lift.pos"),
    ("elbow_flex.pos", "arm_elbow_flex.pos"),
    ("wrist_flex.pos", "arm_wrist_flex.pos"),
    ("wrist_roll.pos", "arm_wrist_roll.pos"),
    ("gripper.pos", "arm_gripper.pos"),
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Record a local arm-only LeRobot dataset on the Mac by using the leader arm as a "
            "free-taught follower with LeKiwi-compatible joint keys."
        )
    )
    parser.add_argument("--robot-id", default="leader")
    parser.add_argument("--robot-port", required=True)
    parser.add_argument("--dataset-repo-id", required=True)
    parser.add_argument("--dataset-root", required=True)
    parser.add_argument("--task", required=True)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--num-episodes", type=int, default=50)
    parser.add_argument("--episode-time-s", type=int, default=20)
    parser.add_argument("--reset-time-s", type=int, default=10)
    return parser.parse_args()


def load_or_create_dataset(args: argparse.Namespace) -> LeRobotDataset:
    dataset_root = Path(args.dataset_root).expanduser()
    dataset_root.parent.mkdir(parents=True, exist_ok=True)

    arm_features = {key: float for _, key in ARM_KEY_MAP}
    dataset_features = {
        **hw_to_dataset_features(arm_features, ACTION),
        **hw_to_dataset_features(arm_features, OBS_STR),
    }

    if (dataset_root / "meta" / "info.json").exists():
        return LeRobotDataset.resume(repo_id=args.dataset_repo_id, root=dataset_root)

    return LeRobotDataset.create(
        repo_id=args.dataset_repo_id,
        root=dataset_root,
        fps=args.fps,
        features=dataset_features,
        robot_type="lekiwi",
        use_videos=False,
    )


def read_arm_observation(robot: SO101Follower) -> dict[str, float]:
    raw_observation = robot.get_observation()
    return {
        target_key: float(raw_observation[source_key])
        for source_key, target_key in ARM_KEY_MAP
    }


def add_frame(dataset: LeRobotDataset, observation: dict[str, float]) -> None:
    action = dict(observation)
    frame = {
        **build_dataset_frame(dataset.features, observation, prefix=OBS_STR),
        **build_dataset_frame(dataset.features, action, prefix=ACTION),
    }
    dataset.add_frame(frame)


def main() -> None:
    args = parse_args()
    if args.fps <= 0:
        raise SystemExit("--fps must be greater than 0.")

    dataset = load_or_create_dataset(args)
    robot = SO101Follower(
        SO101FollowerConfig(
            id=args.robot_id,
            port=args.robot_port,
            cameras={},
            use_degrees=True,
        )
    )

    print(f"Connecting to leader arm on {args.robot_port}", flush=True)
    robot.connect()
    robot.bus.disable_torque()

    print(f"Recording local leader-as-follower dataset to {Path(args.dataset_root).expanduser()}", flush=True)
    print(f"Task: {args.task}", flush=True)
    print("Torque is disabled. Manually move the leader arm to demonstrate each episode.", flush=True)
    print(f"Target episodes: {args.num_episodes} at {args.fps} fps", flush=True)

    episodes_recorded = dataset.num_episodes
    if episodes_recorded >= args.num_episodes:
        print(
            f"Dataset already contains {episodes_recorded} episode(s), which meets the target. Nothing to record.",
            flush=True,
        )
        dataset.finalize()
        robot.disconnect()
        return

    try:
        while episodes_recorded < args.num_episodes:
            episode_index = episodes_recorded + 1
            print(f"Starting episode {episode_index}/{args.num_episodes}.", flush=True)
            episode_started_at = time.perf_counter()
            sample_interval_s = 1.0 / args.fps
            next_sample_at = episode_started_at

            while time.perf_counter() - episode_started_at < args.episode_time_s:
                now = time.perf_counter()
                if now < next_sample_at:
                    time.sleep(min(next_sample_at - now, 0.01))
                    continue

                observation = read_arm_observation(robot)
                add_frame(dataset, observation)
                next_sample_at += sample_interval_s

            dataset.save_episode()
            episodes_recorded += 1
            print(
                f"Saved episode {episodes_recorded}/{args.num_episodes} after {args.episode_time_s:.2f}s.",
                flush=True,
            )

            if episodes_recorded < args.num_episodes and args.reset_time_s > 0:
                print(
                    f"Reset window started for {args.reset_time_s}s before episode {episodes_recorded + 1}.",
                    flush=True,
                )
                reset_deadline = time.perf_counter() + args.reset_time_s
                while time.perf_counter() < reset_deadline:
                    time.sleep(min(reset_deadline - time.perf_counter(), 0.1))
    except KeyboardInterrupt:
        print("\nStopping local leader-as-follower capture.", flush=True)
    finally:
        dataset.finalize()
        if robot.is_connected:
            robot.disconnect()

    print(f"Saved {dataset.num_episodes} episode(s) at {Path(args.dataset_root).expanduser()}", flush=True)


if __name__ == "__main__":
    main()
