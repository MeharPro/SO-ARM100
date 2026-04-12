#!/usr/bin/env python3

import argparse
from pathlib import Path

from lerobot.datasets.feature_utils import hw_to_dataset_features
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.policies.factory import make_pre_post_processors
from lerobot.processor import make_default_processors
from lerobot.robots.lekiwi import LeKiwiClient, LeKiwiClientConfig
from lerobot.scripts.lerobot_record import record_loop
from lerobot.utils.constants import ACTION, OBS_STR
from lerobot.utils.control_utils import init_keyboard_listener
from lerobot.utils.utils import log_say
from lerobot.utils.visualization_utils import init_rerun


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run a trained ACT policy on LeKiwi for the wooden-piece pickup task and "
            "optionally save evaluation episodes locally."
        )
    )
    parser.add_argument("--remote-host", default="192.168.40.135")
    parser.add_argument("--robot-id", default="follow-mobile")
    parser.add_argument("--policy-path", required=True, help="Local ACT checkpoint directory or HF repo id.")
    parser.add_argument("--dataset-repo-id", default="meharkhanna/eval_lekiwi_wood_pick")
    parser.add_argument(
        "--dataset-root",
        default=str(Path("~/lerobot-datasets/eval_lekiwi_wood_pick").expanduser()),
        help="Local evaluation dataset directory.",
    )
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--episode-time-s", type=int, default=20)
    parser.add_argument("--num-episodes", type=int, default=10)
    parser.add_argument(
        "--task",
        default="Pick up the wooden piece from the table, close the gripper around it, and lift it.",
    )
    parser.add_argument("--push-to-hub", action="store_true")
    parser.add_argument("--private", action="store_true")
    parser.add_argument("--display-data", action=argparse.BooleanOptionalAction, default=True)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    dataset_root = Path(args.dataset_root).expanduser()
    dataset_root.parent.mkdir(parents=True, exist_ok=True)

    robot = LeKiwiClient(LeKiwiClientConfig(remote_ip=args.remote_host, id=args.robot_id))
    policy = ACTPolicy.from_pretrained(args.policy_path)

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
        image_writer_threads=4,
        streaming_encoding=True,
        encoder_threads=2,
    )

    preprocessor, postprocessor = make_pre_post_processors(
        policy_cfg=policy,
        pretrained_path=args.policy_path,
        dataset_stats=dataset.meta.stats,
        preprocessor_overrides={"device_processor": {"device": str(policy.config.device)}},
    )

    teleop_action_processor, robot_action_processor, robot_observation_processor = make_default_processors()

    print(f"Connecting to LeKiwi host at {args.remote_host}")
    print(f"Evaluating policy from {args.policy_path}")
    print(f"Saving eval episodes to {dataset_root}")

    robot.connect()
    listener, events = init_keyboard_listener()
    if args.display_data:
        init_rerun(session_name="lekiwi_wood_pick_eval")

    try:
        if not robot.is_connected:
            raise ValueError("Robot is not connected.")

        completed_episodes = 0
        while completed_episodes < args.num_episodes and not events["stop_recording"]:

            input(
                f"\nPlace the wooden piece for eval episode {completed_episodes + 1}/{args.num_episodes}, "
                "then press ENTER to start policy rollout."
            )
            log_say(f"Running wooden-pick policy episode {completed_episodes + 1} of {args.num_episodes}")

            record_loop(
                robot=robot,
                events=events,
                fps=args.fps,
                policy=policy,
                preprocessor=preprocessor,
                postprocessor=postprocessor,
                dataset=dataset,
                control_time_s=args.episode_time_s,
                single_task=args.task,
                display_data=args.display_data,
                teleop_action_processor=teleop_action_processor,
                robot_action_processor=robot_action_processor,
                robot_observation_processor=robot_observation_processor,
            )

            if events["rerecord_episode"]:
                log_say("Discarding eval episode and retrying")
                events["rerecord_episode"] = False
                events["exit_early"] = False
                dataset.clear_episode_buffer()
                continue

            dataset.save_episode()
            completed_episodes += 1
    finally:
        log_say("Stop evaluation")
        if robot.is_connected:
            robot.disconnect()
        listener.stop()

        dataset.finalize()
        if args.push_to_hub:
            dataset.push_to_hub(private=args.private)

    print(f"Saved {dataset.num_episodes} eval episodes at {dataset_root}")


if __name__ == "__main__":
    main()
