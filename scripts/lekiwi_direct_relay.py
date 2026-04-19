#!/usr/bin/env python3

import argparse
import os
import time

from lekiwi_leader_support import (
    connect_leader_noninteractive,
    detect_leader_port,
    disconnect_device_safely,
)
from lerobot.robots.lekiwi import LeKiwiClient, LeKiwiClientConfig
from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig
from lerobot.utils.robot_utils import precise_sleep

FPS = 30


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Direct LeKiwi relay with no keyboard teleop layer. Use --leader-only to test the leader arm."
    )
    parser.add_argument("--remote-host", default=os.getenv("LEKIWI_PI_HOST"))
    parser.add_argument("--robot-id", default=os.getenv("LEKIWI_ROBOT_ID", "follow"))
    parser.add_argument("--leader-port", default=os.getenv("LEKIWI_LEADER_PORT"))
    parser.add_argument("--leader-id", default=os.getenv("LEKIWI_LEADER_ID", "lead"))
    parser.add_argument("--fps", type=int, default=int(os.getenv("LEKIWI_FPS", FPS)))
    parser.add_argument("--print-every", type=int, default=10)
    parser.add_argument("--connect-timeout-s", type=int, default=10)
    parser.add_argument("--leader-only", action="store_true")
    return parser.parse_args()


def summarize_action(action: dict[str, float]) -> str:
    ordered_keys = (
        "shoulder_pan.pos",
        "shoulder_lift.pos",
        "elbow_flex.pos",
        "wrist_flex.pos",
        "wrist_roll.pos",
        "gripper.pos",
    )
    return " ".join(f"{key}={action[key]:7.2f}" for key in ordered_keys)


def align_degrees_near_reference(value: float, reference: float) -> float:
    return float(reference) + (((float(value) - float(reference)) + 180.0) % 360.0) - 180.0


def main() -> None:
    args = parse_args()
    leader_port = detect_leader_port(args.leader_port)

    leader = SO101Leader(SO101LeaderConfig(port=leader_port, id=args.leader_id, use_degrees=False))
    robot = None

    print(f"Using leader port: {leader_port}")
    connect_leader_noninteractive(
        leader,
        calibrate_hint=(
            "Run the dashboard's Mac calibration, or run "
            "`lerobot-calibrate --teleop.type=so101_leader --teleop.port <port> --teleop.id "
            f"{args.leader_id}`."
        ),
    )

    if not args.leader_only:
        if not args.remote_host:
            raise SystemExit("--remote-host or LEKIWI_PI_HOST is required unless --leader-only is used.")
        robot = LeKiwiClient(
            LeKiwiClientConfig(
                remote_ip=args.remote_host,
                id=args.robot_id,
                connect_timeout_s=args.connect_timeout_s,
            )
        )
        print(f"Connecting to LeKiwi host at {args.remote_host}")
        robot.connect()

    print("Starting direct relay loop...")
    loop_idx = 0
    last_wrist_roll: float | None = None
    try:
        while True:
            t0 = time.perf_counter()
            leader_action = leader.get_action()
            wrist_key = "wrist_roll.pos"
            if wrist_key in leader_action:
                wrist_value = float(leader_action[wrist_key])
                if last_wrist_roll is not None:
                    wrist_value = align_degrees_near_reference(wrist_value, last_wrist_roll)
                last_wrist_roll = wrist_value
                leader_action[wrist_key] = wrist_value

            if loop_idx % args.print_every == 0:
                print(summarize_action(leader_action))

            if robot is not None:
                robot.send_action(
                    {
                        **{f"arm_{key}": value for key, value in leader_action.items()},
                        "x.vel": 0.0,
                        "y.vel": 0.0,
                        "theta.vel": 0.0,
                    }
                )

            loop_idx += 1
            precise_sleep(max(1.0 / args.fps - (time.perf_counter() - t0), 0.0))
    except KeyboardInterrupt:
        print("\nStopping direct relay.")
    finally:
        disconnect_device_safely(robot, "LeKiwi client")
        disconnect_device_safely(leader, "leader arm")


if __name__ == "__main__":
    main()
