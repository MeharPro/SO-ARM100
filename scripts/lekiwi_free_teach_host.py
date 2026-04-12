#!/usr/bin/env python3

import argparse
import base64
import json
import logging
import runpy
import time
from pathlib import Path

import cv2
import zmq

from lerobot.robots.lekiwi import LeKiwi, LeKiwiConfig

logging.basicConfig(level=logging.WARNING)
logger = logging.getLogger("lekiwi_free_teach_host")

BASE_ACTION_KEYS = ("x.vel", "y.vel", "theta.vel")

if "PowerTelemetryLogger" not in globals():
    helper_path = Path(__file__).with_name("lekiwi_power.py")
    if helper_path.exists():
        _helper_ns = runpy.run_path(str(helper_path))
        PowerTelemetryLogger = _helper_ns["PowerTelemetryLogger"]
        add_power_monitor_args = _helper_ns["add_power_monitor_args"]
    else:
        raise RuntimeError("Power telemetry helper is unavailable.")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="LeKiwi host with arm torque disabled for free-teach recording."
    )
    parser.add_argument("--robot-id", "--robot.id", dest="robot_id", default="follow")
    parser.add_argument("--robot-port", "--robot.port", dest="robot_port", default="/dev/ttyACM0")
    parser.add_argument(
        "--robot-cameras-json",
        "--robot.cameras",
        dest="robot_cameras_json",
        default="default",
        help="Use 'default' to keep the built-in front and wrist cameras enabled, or '{}' to disable them.",
    )
    parser.add_argument("--port-zmq-cmd", "--host.port_zmq_cmd", dest="port_zmq_cmd", type=int, default=5555)
    parser.add_argument(
        "--port-zmq-observations", "--host.port_zmq_observations", dest="port_zmq_observations", type=int, default=5556
    )
    parser.add_argument("--connection-time-s", "--host.connection_time_s", dest="connection_time_s", type=int, default=600)
    parser.add_argument(
        "--watchdog-timeout-ms", "--host.watchdog_timeout_ms", dest="watchdog_timeout_ms", type=int, default=500
    )
    parser.add_argument("--loop-hz", "--host.max_loop_freq_hz", dest="loop_hz", type=float, default=30.0)
    parser.add_argument("--disable-base-torque", action="store_true", help="Also disable base wheel torque.")
    add_power_monitor_args(parser)
    return parser.parse_args()


def configure_cameras(robot_config: LeKiwiConfig, cameras_json: str) -> None:
    value = cameras_json.strip()
    if value in {"", "default", "__default__", "null"}:
        return

    parsed = json.loads(value)
    if parsed == {}:
        robot_config.cameras = {}
        return

    raise SystemExit("Only '{}' or 'default' are currently supported for --robot-cameras-json.")


def extract_base_action(action: dict[str, float]) -> dict[str, float]:
    return {key: float(action.get(key, 0.0)) for key in BASE_ACTION_KEYS}


def main() -> None:
    args = parse_args()

    robot_config = LeKiwiConfig(id=args.robot_id, port=args.robot_port)
    configure_cameras(robot_config, args.robot_cameras_json)
    robot = LeKiwi(robot_config)
    robot.connect()

    passive_motors = list(robot.arm_motors)
    if args.disable_base_torque:
        passive_motors.extend(robot.base_motors)
    robot.bus.disable_torque(passive_motors, num_retry=10)

    power_logger = PowerTelemetryLogger(robot, args, logger, "lekiwi_free_teach_host")
    power_logger.start()

    ctx = zmq.Context()
    cmd_socket = ctx.socket(zmq.PULL)
    cmd_socket.setsockopt(zmq.CONFLATE, 1)
    cmd_socket.bind(f"tcp://*:{args.port_zmq_cmd}")

    obs_socket = ctx.socket(zmq.PUSH)
    obs_socket.setsockopt(zmq.CONFLATE, 1)
    obs_socket.bind(f"tcp://*:{args.port_zmq_observations}")

    last_cmd_time = time.time()
    watchdog_active = False

    print(
        f"LeKiwi free-teach host listening on tcp://*:{args.port_zmq_cmd} and tcp://*:{args.port_zmq_observations}",
        flush=True,
    )
    print("Arm torque is disabled. You can hand-guide the arm directly.", flush=True)
    if args.disable_base_torque:
        print("Base torque is also disabled. Wheels will not respond to commands.", flush=True)

    start = time.perf_counter()
    try:
        while time.perf_counter() - start < args.connection_time_s:
            loop_start = time.time()

            try:
                msg = cmd_socket.recv_string(zmq.NOBLOCK)
                action = dict(json.loads(msg))
                base_action = extract_base_action(action)
                robot.send_action(base_action)
                last_cmd_time = time.time()
                watchdog_active = False
            except zmq.Again:
                if not watchdog_active:
                    logger.warning("No command available")
            except Exception as exc:
                logger.warning("Message fetching failed: %s", exc)

            now = time.time()
            if (now - last_cmd_time > args.watchdog_timeout_ms / 1000.0) and not watchdog_active:
                logger.warning(
                    "Command not received for more than %s milliseconds. Stopping the base.",
                    args.watchdog_timeout_ms,
                )
                robot.stop_base()
                watchdog_active = True

            observation = robot.get_observation()
            for cam_key in robot.cameras:
                ret, buffer = cv2.imencode(".jpg", observation[cam_key], [int(cv2.IMWRITE_JPEG_QUALITY), 90])
                observation[cam_key] = base64.b64encode(buffer).decode("utf-8") if ret else ""

            try:
                obs_socket.send_string(json.dumps(observation), flags=zmq.NOBLOCK)
            except zmq.Again:
                pass
            except Exception as exc:
                logger.warning("Observation publish failed: %s", exc)

            power_logger.maybe_sample()

            elapsed = time.time() - loop_start
            time.sleep(max(1.0 / args.loop_hz - elapsed, 0.0))
    except KeyboardInterrupt:
        print("\nStopping LeKiwi free-teach host", flush=True)
    finally:
        try:
            robot.disconnect()
        except Exception as exc:
            print(f"disconnect: {exc}", flush=True)
        power_logger.maybe_sample(force=True)
        power_logger.close()
        cmd_socket.close()
        obs_socket.close()
        ctx.term()


if __name__ == "__main__":
    main()
