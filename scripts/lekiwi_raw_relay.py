#!/usr/bin/env python3

import argparse
import json
import os
import time

import zmq

from lekiwi_leader_support import (
    connect_leader_noninteractive,
    detect_leader_port,
    disconnect_device_safely,
)
from lerobot.robots.lekiwi import LeKiwiClient, LeKiwiClientConfig
from lerobot.teleoperators.keyboard.teleop_keyboard import KeyboardTeleop, KeyboardTeleopConfig
from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig
from lerobot.utils.robot_utils import precise_sleep

FPS = 30


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Raw ZMQ leader-to-LeKiwi relay with no observation dependency.")
    parser.add_argument("--remote-host", default=os.getenv("LEKIWI_PI_HOST"))
    parser.add_argument("--port-zmq-cmd", type=int, default=5555)
    parser.add_argument("--leader-port", default=os.getenv("LEKIWI_LEADER_PORT"))
    parser.add_argument("--leader-id", default=os.getenv("LEKIWI_LEADER_ID", "lead"))
    parser.add_argument("--keyboard-id", default=os.getenv("LEKIWI_KEYBOARD_ID", "my_laptop_keyboard"))
    parser.add_argument("--fps", type=int, default=int(os.getenv("LEKIWI_FPS", FPS)))
    parser.add_argument("--print-every", type=int, default=10)
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


def to_grid_base_action(stock_action: dict[str, float]) -> dict[str, float]:
    return {
        "x.vel": float(stock_action.get("y.vel", 0.0) or 0.0),
        "y.vel": float(stock_action.get("x.vel", 0.0) or 0.0),
        "theta.vel": float(stock_action.get("theta.vel", 0.0) or 0.0),
    }


def main() -> None:
    args = parse_args()
    if not args.remote_host:
        raise SystemExit("--remote-host or LEKIWI_PI_HOST is required.")

    leader_port = detect_leader_port(args.leader_port)
    leader = SO101Leader(SO101LeaderConfig(port=leader_port, id=args.leader_id, use_degrees=False))
    # Reuse the stock LeKiwi keyboard mapping and speed-step logic without depending on the network client path.
    base_mapper = LeKiwiClient(LeKiwiClientConfig(remote_ip=args.remote_host, id="follow"))
    keyboard = KeyboardTeleop(KeyboardTeleopConfig(id=args.keyboard_id))

    connect_leader_noninteractive(
        leader,
        calibrate_hint=(
            "Run the dashboard's Mac calibration, or run "
            "`lerobot-calibrate --teleop.type=so101_leader --teleop.port <port> --teleop.id "
            f"{args.leader_id}`."
        ),
    )
    keyboard.connect()

    ctx = zmq.Context()
    sock = ctx.socket(zmq.PUSH)
    sock.setsockopt(zmq.CONFLATE, 1)
    sock.connect(f"tcp://{args.remote_host}:{args.port_zmq_cmd}")

    print(f"Using leader port: {leader_port}")
    print(f"Relaying leader arm to tcp://{args.remote_host}:{args.port_zmq_cmd}")
    print("Base controls: W/S forward/back (Y), A/D left-right (X), Z/X rotate, R/F speed up/down")

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
            keyboard_keys = keyboard.get_action()
            base_action = to_grid_base_action(base_mapper._from_keyboard_to_base_action(keyboard_keys))

            if loop_idx % args.print_every == 0:
                print(summarize_action(leader_action))

            action = {f"arm_{key}": value for key, value in leader_action.items()}
            action.update(base_action)
            sock.send_string(json.dumps(action))

            loop_idx += 1
            precise_sleep(max(1.0 / args.fps - (time.perf_counter() - t0), 0.0))
    except KeyboardInterrupt:
        print("\nStopping raw relay")
    finally:
        disconnect_device_safely(keyboard, "keyboard teleop")
        disconnect_device_safely(leader, "leader arm")
        sock.close()
        ctx.term()


if __name__ == "__main__":
    main()
