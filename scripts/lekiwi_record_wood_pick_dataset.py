#!/usr/bin/env python3

import argparse
import glob
import os
import shutil
import time
from pathlib import Path

from lerobot.datasets.feature_utils import build_dataset_frame
from lerobot.datasets.feature_utils import hw_to_dataset_features
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.processor import make_default_processors
from lerobot.robots.lekiwi import LeKiwiClient, LeKiwiClientConfig
from lerobot.scripts.lerobot_record import record_loop
from lerobot.teleoperators.keyboard import KeyboardTeleop, KeyboardTeleopConfig
from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig
from lerobot.utils.constants import ACTION, OBS_STR
from lerobot.utils.control_utils import init_keyboard_listener
from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.utils import log_say
from lerobot.utils.visualization_utils import init_rerun, log_rerun_data

ARM_ACTION_KEYS = (
    "arm_shoulder_pan.pos",
    "arm_shoulder_lift.pos",
    "arm_elbow_flex.pos",
    "arm_wrist_flex.pos",
    "arm_wrist_roll.pos",
    "arm_gripper.pos",
)
BASE_ACTION_KEYS = ("x.vel", "y.vel", "theta.vel")


def is_incomplete_empty_dataset_root(dataset_root: Path) -> bool:
    meta_info_path = dataset_root / "meta" / "info.json"
    meta_tasks_path = dataset_root / "meta" / "tasks.parquet"
    episodes_dir = dataset_root / "meta" / "episodes"

    if not meta_info_path.exists():
        return False

    if meta_tasks_path.exists():
        return False

    if episodes_dir.exists():
        return False

    all_files = [path for path in dataset_root.rglob("*") if path.is_file()]
    return all_files == [meta_info_path]


def detect_leader_port() -> str:
    candidates = sorted(
        set(glob.glob("/dev/tty.usbmodem*"))
        | set(glob.glob("/dev/tty.usbserial*"))
        | set(glob.glob("/dev/cu.usbmodem*"))
        | set(glob.glob("/dev/cu.usbserial*"))
    )
    if not candidates:
        raise SystemExit(
            "No leader-arm serial port found. Plug the leader arm into the Mac and rerun, "
            "or pass --leader-port explicitly."
        )

    grouped: dict[str, list[str]] = {}
    for path in candidates:
        name = os.path.basename(path)
        if name.startswith("tty."):
            key = name.removeprefix("tty.")
        elif name.startswith("cu."):
            key = name.removeprefix("cu.")
        else:
            key = name
        grouped.setdefault(key, []).append(path)

    deduped = []
    for key in sorted(grouped):
        paths = sorted(grouped[key])
        tty_path = next((p for p in paths if os.path.basename(p).startswith("tty.")), None)
        deduped.append(tty_path or paths[0])

    if len(deduped) == 1:
        return deduped[0]

    raise SystemExit(
        "More than one leader-arm serial port was found. Re-run with --leader-port set to one of:\n"
        + "\n".join(deduped)
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Record LeKiwi camera-based demonstrations for a wooden-piece pickup task. "
            "Run the Pi host first, then use the leader arm to demonstrate the task."
        )
    )
    parser.add_argument("--remote-host", default=os.getenv("LEKIWI_PI_HOST", "192.168.40.135"))
    parser.add_argument("--robot-id", default=os.getenv("LEKIWI_ROBOT_ID", "follow-mobile"))
    parser.add_argument("--leader-port", default=os.getenv("LEKIWI_LEADER_PORT"))
    parser.add_argument("--leader-id", default=os.getenv("LEKIWI_LEADER_ID", "leader"))
    parser.add_argument("--keyboard-id", default=os.getenv("LEKIWI_KEYBOARD_ID", "my_laptop_keyboard"))
    parser.add_argument("--dataset-repo-id", default="meharkhanna/lekiwi_wood_pick")
    parser.add_argument(
        "--dataset-root",
        default=str(Path("~/lerobot-datasets/lekiwi_wood_pick").expanduser()),
        help="Local dataset directory.",
    )
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--episode-time-s", type=int, default=20)
    parser.add_argument("--reset-time-s", type=int, default=10)
    parser.add_argument("--num-episodes", type=int, default=30)
    parser.add_argument(
        "--task",
        default="Pick up the wooden piece from the table, close the gripper around it, and lift it.",
    )
    parser.add_argument(
        "--arm-free-teach",
        action="store_true",
        help=(
            "Do not use the leader arm. Instead, record the current follower arm pose as the demonstration "
            "while you hand-guide the arm with torque disabled on the Pi host. The keyboard still controls "
            "the base in this mode."
        ),
    )
    parser.add_argument("--push-to-hub", action="store_true")
    parser.add_argument("--private", action="store_true")
    parser.add_argument(
        "--overwrite-existing",
        action="store_true",
        help="Delete an existing local dataset root before starting a fresh recording run.",
    )
    parser.add_argument("--display-data", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--streaming-encoding", action=argparse.BooleanOptionalAction, default=True)
    return parser.parse_args()


def free_teach_record_loop(
    *,
    robot: LeKiwiClient,
    keyboard: KeyboardTeleop,
    events: dict,
    dataset: LeRobotDataset | None,
    fps: int,
    control_time_s: int,
    single_task: str,
    display_data: bool,
    robot_observation_processor,
) -> None:
    start_episode_t = time.perf_counter()
    timestamp = 0.0
    while timestamp < control_time_s:
        start_loop_t = time.perf_counter()

        if events["exit_early"]:
            events["exit_early"] = False
            break

        obs = robot.get_observation()
        obs_processed = robot_observation_processor(obs)
        missing_image_keys = [
            key.removeprefix(f"{OBS_STR}.images.")
            for key, ft in dataset.features.items()
            if dataset is not None
            and key.startswith(f"{OBS_STR}.images.")
            and ft["dtype"] in ["image", "video"]
            and key.removeprefix(f"{OBS_STR}.images.") not in obs_processed
        ]
        if missing_image_keys:
            raise RuntimeError(
                "The Pi host is not publishing the expected camera observations: "
                + ", ".join(sorted(missing_image_keys))
                + ". Start the Pi free-teach host with cameras enabled, e.g. "
                + "`python scripts/lekiwi_free_teach_host.py --robot.id=follow-mobile "
                + "--robot.cameras=default --host.connection_time_s=3600`."
            )

        keyboard_keys = keyboard.get_action()
        base_action = robot._from_keyboard_to_base_action(keyboard_keys)
        if len(base_action) == 0:
            base_action = dict.fromkeys(BASE_ACTION_KEYS, 0.0)

        arm_action = {key: float(obs_processed[key]) for key in ARM_ACTION_KEYS}
        action_values = {**arm_action, **base_action}

        robot.send_action(action_values)

        if dataset is not None:
            observation_frame = build_dataset_frame(dataset.features, obs_processed, prefix=OBS_STR)
            action_frame = build_dataset_frame(dataset.features, action_values, prefix=ACTION)
            frame = {**observation_frame, **action_frame, "task": single_task}
            dataset.add_frame(frame)

        if display_data:
            log_rerun_data(observation=obs_processed, action=action_values)

        precise_sleep(max(1.0 / fps - (time.perf_counter() - start_loop_t), 0.0))
        timestamp = time.perf_counter() - start_episode_t


def main() -> None:
    args = parse_args()
    leader_port = None if args.arm_free_teach else (args.leader_port or detect_leader_port())
    dataset_root = Path(args.dataset_root).expanduser()
    dataset_root.parent.mkdir(parents=True, exist_ok=True)

    robot = LeKiwiClient(LeKiwiClientConfig(remote_ip=args.remote_host, id=args.robot_id))
    leader_arm = (
        None
        if args.arm_free_teach
        else SO101Leader(SO101LeaderConfig(port=leader_port, id=args.leader_id, use_degrees=False))
    )
    keyboard = KeyboardTeleop(KeyboardTeleopConfig(id=args.keyboard_id))

    teleop_action_processor, robot_action_processor, robot_observation_processor = make_default_processors()

    action_features = hw_to_dataset_features(robot.action_features, ACTION)
    obs_features = hw_to_dataset_features(robot.observation_features, OBS_STR)
    dataset_features = {**action_features, **obs_features}

    dataset = None
    existing_episodes = 0
    meta_info_path = dataset_root / "meta" / "info.json"

    if dataset_root.exists():
        if args.overwrite_existing:
            shutil.rmtree(dataset_root)
        elif is_incomplete_empty_dataset_root(dataset_root):
            print(
                f"Found an incomplete empty dataset stub at {dataset_root}. "
                "Removing it and starting a fresh local dataset."
            )
            shutil.rmtree(dataset_root)
        elif meta_info_path.exists():
            try:
                dataset = LeRobotDataset.resume(
                    repo_id=args.dataset_repo_id,
                    root=dataset_root,
                    streaming_encoding=args.streaming_encoding,
                    image_writer_threads=4,
                    encoder_threads=2,
                )
                existing_episodes = dataset.num_episodes
            except Exception as exc:
                raise SystemExit(
                    f"Dataset root exists at {dataset_root} but cannot be resumed: {exc}\n"
                    "If this was a failed first run with no saved episodes, rerun with "
                    "--overwrite-existing to recreate it."
                ) from exc
        else:
            raise SystemExit(
                f"Dataset root already exists at {dataset_root} but does not look like a resumable LeRobot dataset. "
                "Remove it manually or rerun with --overwrite-existing."
            )

    if dataset is None:
        dataset = LeRobotDataset.create(
            repo_id=args.dataset_repo_id,
            root=dataset_root,
            fps=args.fps,
            features=dataset_features,
            robot_type=robot.name,
            use_videos=True,
            image_writer_threads=4,
            streaming_encoding=args.streaming_encoding,
            encoder_threads=2,
        )
    elif dataset.fps != args.fps:
        raise SystemExit(
            f"Existing dataset at {dataset_root} was created at {dataset.fps} fps, "
            f"but you requested --fps {args.fps}. "
            "Resume with the same fps, or rerun with --overwrite-existing to start a new dataset."
        )

    print(f"Connecting to LeKiwi host at {args.remote_host}")
    if leader_port is not None:
        print(f"Using leader port: {leader_port}")
    print(f"Recording dataset to {dataset_root}")
    if existing_episodes > 0:
        print(
            f"Resuming existing dataset with {existing_episodes} recorded episode"
            f"{'' if existing_episodes == 1 else 's'}."
        )
    if args.arm_free_teach:
        print("Free-teach mode: hand-guide the follower arm while watching the cameras.")
        print("Use the keyboard only for the base. Start the Pi with lekiwi_free_teach_host.py.")
    else:
        print("Demonstrate the task by watching the cameras, not by looking directly at the robot.")
        print("Normal teleop mode: the leader arm controls the arm and the keyboard controls the base.")

    robot.connect()
    if leader_arm is not None:
        leader_arm.connect()
    keyboard.connect()

    listener, events = init_keyboard_listener()
    if args.display_data:
        init_rerun(session_name="lekiwi_wood_pick_record")

    try:
        if not robot.is_connected or not keyboard.is_connected or (leader_arm is not None and not leader_arm.is_connected):
            raise ValueError("Robot or teleop is not connected.")

        if existing_episodes >= args.num_episodes:
            raise SystemExit(
                f"Dataset already contains {existing_episodes} episodes, which meets or exceeds "
                f"--num-episodes={args.num_episodes}. Use a larger --num-episodes or --overwrite-existing."
            )

        recorded_episodes = existing_episodes
        while recorded_episodes < args.num_episodes and not events["stop_recording"]:
            log_say(f"Recording wooden-pick episode {recorded_episodes + 1} of {args.num_episodes}")
            if args.arm_free_teach:
                free_teach_record_loop(
                    robot=robot,
                    keyboard=keyboard,
                    events=events,
                    dataset=dataset,
                    fps=args.fps,
                    control_time_s=args.episode_time_s,
                    single_task=args.task,
                    display_data=args.display_data,
                    robot_observation_processor=robot_observation_processor,
                )
            else:
                record_loop(
                    robot=robot,
                    events=events,
                    fps=args.fps,
                    dataset=dataset,
                    teleop=[leader_arm, keyboard],
                    control_time_s=args.episode_time_s,
                    single_task=args.task,
                    display_data=args.display_data,
                    teleop_action_processor=teleop_action_processor,
                    robot_action_processor=robot_action_processor,
                    robot_observation_processor=robot_observation_processor,
                )

            if events["rerecord_episode"]:
                log_say("Re-record episode")
                events["rerecord_episode"] = False
                events["exit_early"] = False
                dataset.clear_episode_buffer()
                continue

            dataset.save_episode()
            recorded_episodes += 1

            if recorded_episodes < args.num_episodes and not events["stop_recording"]:
                log_say("Reset the wooden piece for the next episode")
                if args.arm_free_teach:
                    free_teach_record_loop(
                        robot=robot,
                        keyboard=keyboard,
                        events=events,
                        dataset=None,
                        fps=args.fps,
                        control_time_s=args.reset_time_s,
                        single_task=args.task,
                        display_data=args.display_data,
                        robot_observation_processor=robot_observation_processor,
                    )
                else:
                    record_loop(
                        robot=robot,
                        events=events,
                        fps=args.fps,
                        teleop=[leader_arm, keyboard],
                        control_time_s=args.reset_time_s,
                        single_task=args.task,
                        display_data=args.display_data,
                        teleop_action_processor=teleop_action_processor,
                        robot_action_processor=robot_action_processor,
                        robot_observation_processor=robot_observation_processor,
                    )
    finally:
        log_say("Stop recording")
        if robot.is_connected:
            robot.disconnect()
        if leader_arm is not None and leader_arm.is_connected:
            leader_arm.disconnect()
        if keyboard.is_connected:
            keyboard.disconnect()
        listener.stop()

        dataset.finalize()
        if args.push_to_hub:
            dataset.push_to_hub(private=args.private)

    print(f"Saved {dataset.num_episodes} episodes at {dataset_root}")


if __name__ == "__main__":
    main()
