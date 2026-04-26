#!/usr/bin/env python3

import argparse
import json
import os

import zmq


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Send a control command to the warm LeKiwi host.")
    parser.add_argument("--remote-host", default=os.getenv("LEKIWI_PI_HOST"))
    parser.add_argument("--port-zmq-cmd", type=int, default=int(os.getenv("LEKIWI_PORT_ZMQ_CMD", "5555")))
    parser.add_argument("--command", choices=("replay", "stop-replay"), required=True)
    parser.add_argument("--trajectory-path", default="")
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--hold-final-s", type=float, default=0.5)
    parser.add_argument("--include-base", action="store_true")
    parser.add_argument("--vex-replay-mode", choices=("drive", "ecu"), default="ecu")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if not args.remote_host:
        raise SystemExit("--remote-host or LEKIWI_PI_HOST is required.")
    if args.command == "replay" and not args.trajectory_path.strip():
        raise SystemExit("--trajectory-path is required for replay commands.")

    payload: dict[str, object] = {"command": args.command}
    if args.command == "replay":
        payload.update(
            {
                "trajectory_path": args.trajectory_path,
                "speed": args.speed,
                "hold_final_s": args.hold_final_s,
                "include_base": args.include_base,
                "vex_replay_mode": args.vex_replay_mode,
            }
        )

    ctx = zmq.Context()
    sock = ctx.socket(zmq.PUSH)
    sock.setsockopt(zmq.CONFLATE, 1)
    sock.connect(f"tcp://{args.remote_host}:{args.port_zmq_cmd}")

    try:
        sock.send_string(json.dumps(payload))
        print(json.dumps(payload), flush=True)
    finally:
        sock.close()
        ctx.term()


if __name__ == "__main__":
    main()
