#!/usr/bin/env python3

import argparse
import json
import os
import time

import zmq

from lerobot.robots.lekiwi import LeKiwiClient, LeKiwiClientConfig
from lerobot.teleoperators.keyboard.teleop_keyboard import KeyboardTeleop, KeyboardTeleopConfig
from lerobot.utils.robot_utils import precise_sleep

FPS = 30


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Keyboard-only LeKiwi base relay with no leader arm.")
    parser.add_argument("--remote-host", default=os.getenv("LEKIWI_PI_HOST"))
    parser.add_argument("--robot-id", default=os.getenv("LEKIWI_ROBOT_ID", "follow"))
    parser.add_argument("--keyboard-id", default=os.getenv("LEKIWI_KEYBOARD_ID", "my_laptop_keyboard"))
    parser.add_argument("--port-zmq-cmd", type=int, default=int(os.getenv("LEKIWI_PORT_ZMQ_CMD", "5555")))
    parser.add_argument("--fps", type=int, default=int(os.getenv("LEKIWI_FPS", FPS)))
    parser.add_argument("--print-every", type=int, default=10)
    return parser.parse_args()


def summarize_action(action: dict[str, float]) -> str:
    return " ".join(
        (
            f"x.vel={action['x.vel']:6.2f}",
            f"y.vel={action['y.vel']:6.2f}",
            f"theta.vel={action['theta.vel']:6.2f}",
        )
    )


def main() -> None:
    args = parse_args()
    if not args.remote_host:
        raise SystemExit("--remote-host or LEKIWI_PI_HOST is required.")

    # Reuse the stock LeKiwi keyboard mapping and speed-step logic without connecting a leader arm.
    base_mapper = LeKiwiClient(LeKiwiClientConfig(remote_ip=args.remote_host, id=args.robot_id))
    keyboard = KeyboardTeleop(KeyboardTeleopConfig(id=args.keyboard_id))
    keyboard.connect()
    if not keyboard.is_connected:
        raise SystemExit(
            "Keyboard listener did not start. On macOS, grant Input Monitoring/Accessibility access to the terminal."
        )

    ctx = zmq.Context()
    sock = ctx.socket(zmq.PUSH)
    sock.setsockopt(zmq.CONFLATE, 1)
    sock.connect(f"tcp://{args.remote_host}:{args.port_zmq_cmd}")

    print(f"Keyboard-only base relay to tcp://{args.remote_host}:{args.port_zmq_cmd}")
    print("Base controls: W/S forward/back, A/D strafe, Z/X rotate, R/F speed up/down")

    loop_idx = 0
    last_action = {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}
    try:
        while True:
            t0 = time.perf_counter()
            keyboard_keys = keyboard.get_action()
            last_action = base_mapper._from_keyboard_to_base_action(keyboard_keys)
            sock.send_string(json.dumps(last_action))

            if loop_idx % args.print_every == 0:
                print(summarize_action(last_action))

            loop_idx += 1
            precise_sleep(max(1.0 / args.fps - (time.perf_counter() - t0), 0.0))
    except KeyboardInterrupt:
        print("\nStopping keyboard-only relay.")
    finally:
        try:
            sock.send_string(json.dumps({"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}))
        except Exception:
            pass
        if keyboard.is_connected:
            keyboard.disconnect()
        sock.close()
        ctx.term()


if __name__ == "__main__":
    main()
