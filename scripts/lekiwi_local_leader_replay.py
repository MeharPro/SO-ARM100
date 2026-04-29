#!/usr/bin/env python3

import argparse
import json
import math
import time
from pathlib import Path

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from lerobot.utils.robot_utils import precise_sleep

ARM_KEY_MAP = (
    ("arm_shoulder_pan.pos", "shoulder_pan.pos"),
    ("arm_shoulder_lift.pos", "shoulder_lift.pos"),
    ("arm_elbow_flex.pos", "elbow_flex.pos"),
    ("arm_wrist_flex.pos", "wrist_flex.pos"),
    ("arm_wrist_roll.pos", "wrist_roll.pos"),
    ("arm_gripper.pos", "gripper.pos"),
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Replay a saved LeKiwi follower trajectory on the Mac by driving the leader arm "
            "as if it were a follower."
        )
    )
    parser.add_argument("--robot-id", default="leader")
    parser.add_argument("--robot-port", required=True)
    parser.add_argument("--input", required=True)
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--hold-final-s", type=float, default=0.5)
    parser.add_argument("--start-time-s", type=float, default=0.0)
    parser.add_argument("--print-every", type=int, default=30)
    return parser.parse_args()


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


def build_action(state: dict[str, float]) -> dict[str, float]:
    return {target_key: float(state[source_key]) for source_key, target_key in ARM_KEY_MAP}


def replay_start_index(samples: list[dict[str, object]], start_time_s: float) -> int:
    if start_time_s <= 0:
        return 0

    for index, sample in enumerate(samples):
        if not isinstance(sample, dict):
            continue
        raw_t_s = sample.get("t_s")
        if not isinstance(raw_t_s, (int, float)):
            continue
        t_s = float(raw_t_s)
        if math.isfinite(t_s) and t_s >= start_time_s:
            return index
    return max(len(samples) - 1, 0)


def summarize_action(action: dict[str, float]) -> str:
    return " ".join(
        f"{source_key}={action[target_key]:7.2f}"
        for source_key, target_key in ARM_KEY_MAP
    )


def main() -> None:
    args = parse_args()
    if args.speed <= 0:
        raise SystemExit("--speed must be greater than 0.")
    if not math.isfinite(float(args.start_time_s)) or args.start_time_s < 0:
        raise SystemExit("--start-time-s must be 0 or greater.")
    start_time_s = float(args.start_time_s)

    trajectory_path, samples = load_trajectory(args.input)
    start_index = replay_start_index(samples, start_time_s)
    replay_samples = samples[start_index:]
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
    print(f"Replaying {len(replay_samples)} of {len(samples)} samples from {trajectory_path}", flush=True)
    if start_time_s > 0:
        print(
            f"Replay resume: starting at t={start_time_s:.3f}s "
            f"(sample {start_index + 1}/{len(samples)}).",
            flush=True,
        )

    last_action: dict[str, float] | None = None
    started_at = time.perf_counter()

    try:
        for index, sample in enumerate(replay_samples, start=start_index + 1):
            state = sample.get("state")
            if not isinstance(state, dict):
                raise SystemExit(f"Sample {index} is missing a valid state payload.")

            raw_t_s = float(sample["t_s"])
            target_t = max(raw_t_s - start_time_s, 0.0) / args.speed
            precise_sleep(max(target_t - (time.perf_counter() - started_at), 0.0))

            action = build_action(state)
            robot.send_action(action)
            last_action = action

            if index == start_index + 1 or index % args.print_every == 0 or index == len(samples):
                print(f"[{index:05d}/{len(samples):05d}] t={raw_t_s:7.3f}s {summarize_action(action)}", flush=True)

        if last_action is not None and args.hold_final_s > 0:
            precise_sleep(args.hold_final_s)
    except KeyboardInterrupt:
        print("\nLeader replay interrupted.", flush=True)
    finally:
        if robot.is_connected:
            robot.disconnect()


if __name__ == "__main__":
    main()
