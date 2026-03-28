#!/usr/bin/env python3

import argparse
import os
import time

import cv2

from lerobot.robots.lekiwi import LeKiwiClient, LeKiwiClientConfig


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="View LeKiwi camera streams from the observation socket.")
    parser.add_argument("--remote-host", default=os.getenv("LEKIWI_PI_HOST"))
    parser.add_argument("--robot-id", default=os.getenv("LEKIWI_ROBOT_ID", "follow"))
    parser.add_argument("--fps", type=float, default=float(os.getenv("LEKIWI_VIEW_FPS", "30")))
    parser.add_argument("--front-only", action="store_true")
    parser.add_argument("--wrist-only", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if not args.remote_host:
        raise SystemExit("--remote-host or LEKIWI_PI_HOST is required.")
    if args.front_only and args.wrist_only:
        raise SystemExit("Use at most one of --front-only or --wrist-only.")

    robot = LeKiwiClient(LeKiwiClientConfig(remote_ip=args.remote_host, id=args.robot_id))
    print(f"Connecting to LeKiwi host at {args.remote_host}", flush=True)
    robot.connect()

    show_front = not args.wrist_only
    show_wrist = not args.front_only

    try:
        while True:
            t0 = time.perf_counter()
            obs = robot.get_observation()

            if show_front and "front" in obs:
                cv2.imshow("LeKiwi Front", obs["front"])
            if show_wrist and "wrist" in obs:
                cv2.imshow("LeKiwi Wrist", obs["wrist"])

            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break

            remaining = 1.0 / args.fps - (time.perf_counter() - t0)
            if remaining > 0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        if robot.is_connected:
            robot.disconnect()


if __name__ == "__main__":
    main()
