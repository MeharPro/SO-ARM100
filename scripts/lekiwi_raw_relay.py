#!/usr/bin/env python3

import argparse
import glob
import json
import os
import time

import zmq

from lerobot.robots.lekiwi import LeKiwiClient, LeKiwiClientConfig
from lerobot.teleoperators.keyboard.teleop_keyboard import KeyboardTeleop, KeyboardTeleopConfig
from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig
from lerobot.utils.robot_utils import precise_sleep

FPS = 30


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


def main() -> None:
    args = parse_args()
    if not args.remote_host:
        raise SystemExit("--remote-host or LEKIWI_PI_HOST is required.")

    leader_port = args.leader_port or detect_leader_port()
    leader = SO101Leader(SO101LeaderConfig(port=leader_port, id=args.leader_id, use_degrees=False))
    # Reuse the stock LeKiwi keyboard mapping and speed-step logic without depending on the network client path.
    base_mapper = LeKiwiClient(LeKiwiClientConfig(remote_ip=args.remote_host, id="follow"))
    keyboard = KeyboardTeleop(KeyboardTeleopConfig(id=args.keyboard_id))

    leader.connect()
    keyboard.connect()

    ctx = zmq.Context()
    sock = ctx.socket(zmq.PUSH)
    sock.setsockopt(zmq.CONFLATE, 1)
    sock.connect(f"tcp://{args.remote_host}:{args.port_zmq_cmd}")

    print(f"Using leader port: {leader_port}")
    print(f"Relaying leader arm to tcp://{args.remote_host}:{args.port_zmq_cmd}")
    print("Base controls: W/S forward/back, A/D strafe, Z/X rotate, R/F speed up/down")

    loop_idx = 0
    try:
        while True:
            t0 = time.perf_counter()
            leader_action = leader.get_action()
            keyboard_keys = keyboard.get_action()
            base_action = base_mapper._from_keyboard_to_base_action(keyboard_keys)

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
        keyboard.disconnect()
        leader.disconnect()
        sock.close()
        ctx.term()


if __name__ == "__main__":
    main()
