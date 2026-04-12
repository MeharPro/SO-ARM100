#!/usr/bin/env python3

import argparse
import glob
import os
import time

from lerobot.robots.lekiwi import LeKiwiClient, LeKiwiClientConfig
from lerobot.teleoperators.so_leader import SO100Leader, SO100LeaderConfig
from lerobot.utils.robot_utils import precise_sleep

FPS = 30


def detect_leader_port(requested_port: str | None) -> str:
    if requested_port and requested_port.lower() != "auto":
        return requested_port

    candidates = sorted(
        set(glob.glob("/dev/tty.usbmodem*"))
        | set(glob.glob("/dev/tty.usbserial*"))
        | set(glob.glob("/dev/cu.usbmodem*"))
        | set(glob.glob("/dev/cu.usbserial*"))
    )
    if not candidates:
        raise SystemExit(
            "No leader-arm serial port found. Plug in the leader arm and rerun start control."
        )

    grouped: dict[str, list[str]] = {}
    for port in candidates:
        name = os.path.basename(port)
        if name.startswith("tty."):
            key = name.removeprefix("tty.")
        elif name.startswith("cu."):
            key = name.removeprefix("cu.")
        else:
            key = name
        grouped.setdefault(key, []).append(port)

    deduped = []
    for key in sorted(grouped):
        options = sorted(grouped[key])
        tty_port = next((item for item in options if os.path.basename(item).startswith("tty.")), None)
        deduped.append(tty_port or options[0])

    if len(deduped) == 1:
        return deduped[0]

    raise SystemExit(
        "More than one leader-arm serial port was found. Pass --leader-port explicitly.\n"
        + "\n".join(deduped)
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Headless UI teleop relay for the LeKiwi follower using the SO100 leader arm."
    )
    parser.add_argument("--remote-host", default=os.getenv("LEKIWI_PI_HOST"))
    parser.add_argument("--robot-id", default=os.getenv("LEKIWI_ROBOT_ID", "follow-mobile"))
    parser.add_argument("--leader-port", default=os.getenv("LEKIWI_LEADER_PORT", "auto"))
    parser.add_argument("--leader-id", default=os.getenv("LEKIWI_LEADER_ID", "leader"))
    parser.add_argument("--fps", type=int, default=int(os.getenv("LEKIWI_FPS", FPS)))
    parser.add_argument("--print-every", type=int, default=10)
    parser.add_argument("--connect-timeout-s", type=int, default=10)
    return parser.parse_args()


def summarize_action(action: dict[str, float]) -> str:
    keys = (
        "shoulder_pan.pos",
        "shoulder_lift.pos",
        "elbow_flex.pos",
        "wrist_flex.pos",
        "wrist_roll.pos",
        "gripper.pos",
    )
    return " ".join(f"{key}={action[key]:7.2f}" for key in keys)


def wrap_degrees(value: float) -> float:
    return ((float(value) + 180.0) % 360.0) - 180.0


def build_robot_action(leader_action: dict[str, float]) -> dict[str, float]:
    arm_action = {f"arm_{key}": value for key, value in leader_action.items()}
    wrist_key = "arm_wrist_roll.pos"
    if wrist_key in arm_action:
        arm_action[wrist_key] = wrap_degrees(arm_action[wrist_key])

    return {
        **arm_action,
        "x.vel": 0.0,
        "y.vel": 0.0,
        "theta.vel": 0.0,
    }


def summarize_robot_action(action: dict[str, float]) -> str:
    return " ".join(
        f"{key.removeprefix('arm_')}={action[key]:7.2f}"
        for key in (
            "arm_shoulder_pan.pos",
            "arm_shoulder_lift.pos",
            "arm_elbow_flex.pos",
            "arm_wrist_flex.pos",
            "arm_wrist_roll.pos",
            "arm_gripper.pos",
        )
    )


def main() -> None:
    args = parse_args()
    if not args.remote_host:
        raise SystemExit("--remote-host or LEKIWI_PI_HOST is required.")

    leader_port = detect_leader_port(args.leader_port)

    robot = LeKiwiClient(
        LeKiwiClientConfig(
            remote_ip=args.remote_host,
            id=args.robot_id,
            connect_timeout_s=args.connect_timeout_s,
        )
    )
    leader = SO100Leader(SO100LeaderConfig(port=leader_port, id=args.leader_id, use_degrees=True))

    print(f"Using leader port: {leader_port}", flush=True)
    print(f"Connecting to LeKiwi host at {args.remote_host}", flush=True)
    print("Starting UI teleop relay. Base keyboard control is disabled in this helper.", flush=True)

    loop_idx = 0
    try:
        robot.connect()
        leader.connect()

        while True:
            t0 = time.perf_counter()
            leader_action = leader.get_action()
            robot_action = build_robot_action(leader_action)

            if loop_idx % args.print_every == 0:
                print(summarize_robot_action(robot_action), flush=True)

            robot.send_action(robot_action)

            loop_idx += 1
            precise_sleep(max(1.0 / args.fps - (time.perf_counter() - t0), 0.0))
    except KeyboardInterrupt:
        print("\nStopping UI teleop relay.", flush=True)
    finally:
        if robot.is_connected:
            robot.disconnect()
        if leader.is_connected:
            leader.disconnect()


if __name__ == "__main__":
    main()
