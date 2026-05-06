#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import math
import time
from typing import Any

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from lerobot.utils.robot_utils import precise_sleep

from lekiwi_leader_support import connect_leader_noninteractive

ARM_KEY_MAP = (
    ("arm_shoulder_pan.pos", "shoulder_pan.pos"),
    ("arm_shoulder_lift.pos", "shoulder_lift.pos"),
    ("arm_elbow_flex.pos", "elbow_flex.pos"),
    ("arm_wrist_flex.pos", "wrist_flex.pos"),
    ("arm_wrist_roll.pos", "wrist_roll.pos"),
    ("arm_gripper.pos", "gripper.pos"),
)
DEFAULT_MAX_STEP_DEG = 8.0
DEFAULT_TOLERANCE_DEG = 3.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Move the Mac-connected leader arm to a held follower pose, then release torque."
    )
    parser.add_argument("--robot-id", default="leader")
    parser.add_argument("--robot-port", required=True)
    parser.add_argument("--home-position-json", required=True)
    parser.add_argument("--fps", type=float, default=60.0)
    parser.add_argument("--settle-s", type=float, default=0.5)
    parser.add_argument("--timeout-s", type=float, default=12.0)
    parser.add_argument("--max-step-deg", type=float, default=DEFAULT_MAX_STEP_DEG)
    parser.add_argument("--tolerance-deg", type=float, default=DEFAULT_TOLERANCE_DEG)
    return parser.parse_args()


def parse_home_position(raw_json: str) -> dict[str, float]:
    payload = json.loads(raw_json)
    if not isinstance(payload, dict):
        raise SystemExit("--home-position-json must be a JSON object.")

    positions: dict[str, float] = {}
    for follower_key, leader_key in ARM_KEY_MAP:
        raw_value = payload.get(follower_key)
        if not isinstance(raw_value, (int, float)) or not math.isfinite(float(raw_value)):
            raise SystemExit(f"--home-position-json is missing numeric {follower_key}.")
        positions[leader_key] = float(raw_value)
    return positions


def read_leader_position(robot: SO101Follower, fallback: dict[str, float]) -> dict[str, float]:
    try:
        observation = robot.get_observation()
    except Exception:
        return dict(fallback)

    positions: dict[str, float] = {}
    for _follower_key, leader_key in ARM_KEY_MAP:
        value = observation.get(leader_key)
        positions[leader_key] = (
            float(value)
            if isinstance(value, (int, float)) and math.isfinite(float(value))
            else fallback[leader_key]
        )
    return positions


def dual_ended_joint_batches() -> tuple[tuple[str, ...], ...]:
    leader_keys = tuple(leader_key for _follower_key, leader_key in ARM_KEY_MAP)
    batches: list[tuple[str, ...]] = []
    left = 0
    right = len(leader_keys) - 1
    while left <= right:
        if left == right:
            batches.append((leader_keys[left],))
        else:
            batches.append((leader_keys[left], leader_keys[right]))
        left += 1
        right -= 1
    return tuple(batches)


def max_error(observation: dict[str, float], target: dict[str, float]) -> float:
    errors = [
        abs(float(observation[key]) - float(target[key]))
        for _follower_key, key in ARM_KEY_MAP
    ]
    return max(errors, default=float("inf"))


def move_leader_to_pose(
    robot: SO101Follower,
    target: dict[str, float],
    *,
    fps: float,
    max_step_deg: float,
) -> dict[str, float]:
    sleep_step_s = 1.0 / max(float(fps), 1.0)
    motion_position = read_leader_position(robot, target)

    for joint_batch in dual_ended_joint_batches():
        batch_starts = {
            joint_key: motion_position[joint_key]
            for joint_key in joint_batch
        }
        batch_steps = []
        for joint_key in joint_batch:
            step_limit = max(float(max_step_deg), 0.1)
            batch_steps.append(math.ceil(abs(target[joint_key] - batch_starts[joint_key]) / step_limit))
        max_steps = max(1, min(600, max(batch_steps, default=1)))

        for step_index in range(1, max_steps + 1):
            progress = step_index / max_steps
            for joint_key in joint_batch:
                start_value = batch_starts[joint_key]
                motion_position[joint_key] = start_value + (target[joint_key] - start_value) * progress
            robot.send_action(dict(motion_position))
            precise_sleep(sleep_step_s)

    return dict(motion_position)


def disable_torque(robot: SO101Follower) -> None:
    try:
        robot.bus.disable_torque(num_retry=10)
    except TypeError:
        robot.bus.disable_torque()


def main() -> None:
    args = parse_args()
    target = parse_home_position(args.home_position_json)
    robot = SO101Follower(
        SO101FollowerConfig(
            id=args.robot_id,
            port=args.robot_port,
            cameras={},
            use_degrees=True,
        )
    )

    print(f"Connecting to leader arm on {args.robot_port}", flush=True)
    connect_leader_noninteractive(
        robot,
        "Use the dashboard calibration controls to calibrate the leader arm, then try Leader Mimic again.",
    )
    try:
        try:
            robot.bus.enable_torque(num_retry=10)
        except TypeError:
            robot.bus.enable_torque()

        print("Moving leader arm to active hold pose.", flush=True)
        move_leader_to_pose(
            robot,
            target,
            fps=args.fps,
            max_step_deg=args.max_step_deg,
        )

        deadline = time.perf_counter() + max(float(args.timeout_s), 0.1)
        observation = read_leader_position(robot, target)
        while max_error(observation, target) > float(args.tolerance_deg):
            if time.perf_counter() >= deadline:
                raise SystemExit(
                    f"Leader arm did not reach hold pose within {args.timeout_s:.1f}s "
                    f"(max error {max_error(observation, target):.2f}deg)."
                )
            robot.send_action(dict(target))
            precise_sleep(1.0 / max(float(args.fps), 1.0))
            observation = read_leader_position(robot, observation)

        if args.settle_s > 0:
            precise_sleep(float(args.settle_s))
        print(f"Leader arm reached hold pose; max error {max_error(observation, target):.2f}deg.", flush=True)
    finally:
        if robot.is_connected:
            disable_torque(robot)
            robot.disconnect()
            print("Leader arm torque released.", flush=True)


if __name__ == "__main__":
    main()
