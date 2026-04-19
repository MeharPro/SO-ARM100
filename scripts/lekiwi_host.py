#!/usr/bin/env python3

import argparse
import base64
import json
import logging
import runpy
import time
from pathlib import Path
from typing import Any

import cv2
import zmq

from lekiwi_runtime import (
    ArmSafetyFilter,
    TorqueLimitFileWatcher,
    add_servo_safety_args,
    add_torque_limit_args,
    apply_torque_limits,
    configure_wrist_roll_mode,
    parse_torque_limits_json,
)
from lerobot.robots.lekiwi import LeKiwi, LeKiwiConfig
from lerobot.utils.robot_utils import precise_sleep

logging.basicConfig(level=logging.WARNING)
logger = logging.getLogger("lekiwi_host")

ARM_STATE_KEYS = (
    "arm_shoulder_pan.pos",
    "arm_shoulder_lift.pos",
    "arm_elbow_flex.pos",
    "arm_wrist_flex.pos",
    "arm_wrist_roll.pos",
    "arm_gripper.pos",
)
BASE_STATE_KEYS = ("x.vel", "y.vel", "theta.vel")

if "PowerTelemetryLogger" not in globals():
    helper_path = Path(__file__).with_name("lekiwi_power.py")
    if helper_path.exists():
        _helper_ns = runpy.run_path(str(helper_path))
        PowerTelemetryLogger = _helper_ns["PowerTelemetryLogger"]
        add_power_monitor_args = _helper_ns["add_power_monitor_args"]
    else:
        raise RuntimeError("Power telemetry helper is unavailable.")


def parse_bool(value: str) -> bool:
    normalized = value.strip().lower()
    if normalized in {"1", "true", "yes", "on"}:
        return True
    if normalized in {"0", "false", "no", "off"}:
        return False
    raise argparse.ArgumentTypeError("expected true or false")


def build_robot_config(args: argparse.Namespace) -> LeKiwiConfig:
    fields = getattr(LeKiwiConfig, "__dataclass_fields__", {})
    kwargs = {
        "id": args.robot_id,
        "port": args.robot_port,
    }
    optional_values = {
        "use_degrees": args.use_degrees,
        "base_max_raw_velocity": args.base_max_raw_velocity,
        "base_wheel_torque_limit": args.base_wheel_torque_limit,
        "enable_base": args.enable_base,
    }
    for name, value in optional_values.items():
        if name in fields:
            kwargs[name] = value

    return LeKiwiConfig(**kwargs)


def coerce_json_bool(value: Any, default: bool = False) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return parse_bool(value)
    if value is None:
        return default
    return bool(value)


class UiCommandWatcher:
    def __init__(self, path: str) -> None:
        self.path = Path(path).expanduser() if path else None
        self.last_mtime_ns: int | None = None

    def poll(self) -> dict[str, Any] | None:
        if self.path is None:
            return None

        try:
            stat = self.path.stat()
        except FileNotFoundError:
            return None

        if self.last_mtime_ns == stat.st_mtime_ns:
            return None

        self.last_mtime_ns = stat.st_mtime_ns

        try:
            payload = json.loads(self.path.read_text())
        except Exception as exc:
            print(f"[replay] error failed to parse {self.path}: {exc}", flush=True)
            self._unlink()
            return None

        self._unlink()
        if not isinstance(payload, dict):
            print(f"[replay] error invalid command payload in {self.path}", flush=True)
            return None

        command = payload.get("command")
        if command == "stop-replay":
            return {"command": "stop-replay"}
        if command != "replay":
            print(f"[replay] error unsupported command in {self.path}: {command!r}", flush=True)
            return None

        trajectory_path = payload.get("trajectory_path")
        if not isinstance(trajectory_path, str) or not trajectory_path.strip():
            print(f"[replay] error replay command is missing trajectory_path in {self.path}", flush=True)
            return None

        try:
            speed = float(payload.get("speed", 1.0))
        except (TypeError, ValueError):
            print(f"[replay] error replay speed is invalid in {self.path}", flush=True)
            return None
        if speed <= 0:
            print(f"[replay] error replay speed must be greater than 0 in {self.path}", flush=True)
            return None

        try:
            hold_final_s = float(payload.get("hold_final_s", 0.5))
        except (TypeError, ValueError):
            print(f"[replay] error hold_final_s is invalid in {self.path}", flush=True)
            return None

        try:
            include_base = coerce_json_bool(payload.get("include_base"), default=False)
        except argparse.ArgumentTypeError as exc:
            print(f"[replay] error include_base is invalid in {self.path}: {exc}", flush=True)
            return None

        return {
            "command": "replay",
            "trajectory_path": trajectory_path.strip(),
            "speed": speed,
            "hold_final_s": max(0.0, hold_final_s),
            "include_base": include_base,
        }

    def _unlink(self) -> None:
        if self.path is None:
            return
        try:
            self.path.unlink()
        except FileNotFoundError:
            return
        except OSError:
            return


def load_trajectory(path_str: str) -> tuple[Path, list[dict[str, Any]]]:
    path = Path(path_str).expanduser()
    if not path.exists():
        raise ValueError(f"Trajectory file does not exist: {path}")

    payload = json.loads(path.read_text())
    if not isinstance(payload, dict) or payload.get("format") != "lekiwi-follower-trajectory":
        raise ValueError(f"Unsupported trajectory format in {path}")

    samples = payload.get("samples")
    if not isinstance(samples, list) or not samples:
        raise ValueError(f"No samples found in {path}")

    return path, samples


def get_state_value(state: dict[str, Any], key: str) -> float:
    value = state.get(key)
    if not isinstance(value, (int, float)):
        raise ValueError(f"State is missing numeric {key}.")
    return float(value)


def build_replay_action(state: dict[str, Any], include_base: bool) -> dict[str, float]:
    action = {key: get_state_value(state, key) for key in ARM_STATE_KEYS}
    if include_base:
        action.update({key: get_state_value(state, key) for key in BASE_STATE_KEYS})
    else:
        action.update(dict.fromkeys(BASE_STATE_KEYS, 0.0))
    return action


def publish_observation(robot: LeKiwi, obs_socket: Any) -> None:
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


def stop_replay_motion(robot: LeKiwi, last_action: dict[str, float] | None) -> None:
    try:
        if last_action is not None:
            robot.send_action(
                {
                    **{key: last_action[key] for key in ARM_STATE_KEYS},
                    **dict.fromkeys(BASE_STATE_KEYS, 0.0),
                }
            )
        else:
            robot.stop_base()
    except Exception:
        pass


def drain_command_socket(cmd_socket: Any) -> None:
    while True:
        try:
            cmd_socket.recv_string(zmq.NOBLOCK)
        except zmq.Again:
            return
        except Exception as exc:
            logger.warning("Command drain failed: %s", exc)
            return


def execute_replay(
    command: dict[str, Any],
    robot: LeKiwi,
    torque_watcher: TorqueLimitFileWatcher,
    replay_filter: ArmSafetyFilter,
    power_logger: Any,
    cmd_socket: Any,
    obs_socket: Any,
    ui_command_watcher: UiCommandWatcher,
    loop_hz: float,
) -> dict[str, Any] | None:
    try:
        trajectory_path, samples = load_trajectory(command["trajectory_path"])
    except Exception as exc:
        print(f"[replay] error {exc}", flush=True)
        return None

    speed = float(command["speed"])
    hold_final_s = float(command["hold_final_s"])
    include_base = bool(command["include_base"])
    sleep_step_s = 1.0 / max(loop_hz, 1.0)

    print(
        "[replay] start "
        f"path={trajectory_path} samples={len(samples)} speed={speed:.3f} include_base={str(include_base).lower()}",
        flush=True,
    )

    last_action: dict[str, float] | None = None
    drain_command_socket(cmd_socket)
    start = time.perf_counter()

    try:
        for index, sample in enumerate(samples, start=1):
            next_command = ui_command_watcher.poll()
            if next_command is not None:
                if next_command["command"] == "stop-replay":
                    print(f"[replay] stopped path={trajectory_path}", flush=True)
                    return None
                print(
                    f"[replay] interrupted path={trajectory_path} next={next_command['trajectory_path']}",
                    flush=True,
                )
                return next_command

            if not isinstance(sample, dict):
                raise ValueError(f"Sample {index} is invalid.")

            state = sample.get("state")
            raw_t_s = sample.get("t_s")
            if not isinstance(state, dict) or not isinstance(raw_t_s, (int, float)):
                raise ValueError(f"Sample {index} is missing state or t_s.")

            target_t = float(raw_t_s) / speed
            precise_sleep(max(target_t - (time.perf_counter() - start), 0.0))
            torque_watcher.poll(robot)
            action = replay_filter.normalize(build_replay_action(state, include_base=include_base))
            sent_action = robot.send_action(action)
            replay_filter.update(sent_action)
            last_action = sent_action
            power_logger.maybe_sample()
            publish_observation(robot, obs_socket)

        if last_action is not None and hold_final_s > 0:
            hold_until = time.perf_counter() + hold_final_s
            while time.perf_counter() < hold_until:
                next_command = ui_command_watcher.poll()
                if next_command is not None:
                    if next_command["command"] == "stop-replay":
                        print(f"[replay] stopped path={trajectory_path}", flush=True)
                        return None
                    print(
                        f"[replay] interrupted path={trajectory_path} next={next_command['trajectory_path']}",
                        flush=True,
                    )
                    return next_command

                torque_watcher.poll(robot)
                power_logger.maybe_sample()
                publish_observation(robot, obs_socket)
                precise_sleep(min(sleep_step_s, max(hold_until - time.perf_counter(), 0.0)))

        print(f"[replay] complete path={trajectory_path}", flush=True)
        return None
    except Exception as exc:
        print(f"[replay] error path={trajectory_path}: {exc}", flush=True)
        return None
    finally:
        stop_replay_motion(robot, last_action)
        drain_command_socket(cmd_socket)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="LeKiwi host with power telemetry.")
    parser.add_argument("--robot-id", "--robot.id", dest="robot_id", default="follow")
    parser.add_argument("--robot-port", "--robot.port", dest="robot_port", default="/dev/ttyACM0")
    parser.add_argument("--robot-cameras-json", "--robot.cameras", dest="robot_cameras_json", default="{}")
    parser.add_argument("--use-degrees", "--robot.use_degrees", dest="use_degrees", type=parse_bool, default=True)
    parser.add_argument("--base-max-raw-velocity", "--robot.base_max_raw_velocity", dest="base_max_raw_velocity", type=int, default=3000)
    parser.add_argument("--base-wheel-torque-limit", "--robot.base_wheel_torque_limit", dest="base_wheel_torque_limit", type=int, default=None)
    parser.add_argument("--enable-base", "--robot.enable_base", dest="enable_base", type=parse_bool, default=True)
    parser.add_argument("--port-zmq-cmd", "--host.port_zmq_cmd", dest="port_zmq_cmd", type=int, default=5555)
    parser.add_argument(
        "--port-zmq-observations", "--host.port_zmq_observations", dest="port_zmq_observations", type=int, default=5556
    )
    parser.add_argument("--connection-time-s", "--host.connection_time_s", dest="connection_time_s", type=int, default=600)
    parser.add_argument(
        "--watchdog-timeout-ms", "--host.watchdog_timeout_ms", dest="watchdog_timeout_ms", type=int, default=500
    )
    parser.add_argument("--loop-hz", "--host.max_loop_freq_hz", dest="loop_hz", type=float, default=30.0)
    parser.add_argument("--ui-command-path", default="")
    add_servo_safety_args(parser)
    add_torque_limit_args(parser)
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


def main() -> None:
    args = parse_args()

    robot_config = build_robot_config(args)
    configure_cameras(robot_config, args.robot_cameras_json)
    robot = LeKiwi(robot_config)
    robot.connect(calibrate=False)
    configure_wrist_roll_mode(robot, continuous=args.safer_servo_mode)
    apply_torque_limits(robot, parse_torque_limits_json(args.torque_limits_json))
    torque_watcher = TorqueLimitFileWatcher(args.torque_limits_path)
    torque_watcher.poll(robot, force=True)
    ui_command_watcher = UiCommandWatcher(args.ui_command_path)
    live_safety_filter = ArmSafetyFilter(
        enabled=args.safer_servo_mode,
        map_wrist_to_follower_start=True,
    )
    replay_safety_filter = ArmSafetyFilter(enabled=args.safer_servo_mode)
    try:
        observation = robot.get_observation()
        live_safety_filter.seed_from_observation(observation)
        replay_safety_filter.seed_from_observation(observation)
    except Exception as exc:
        logger.warning("Failed to seed servo safety filter from the current robot state: %s", exc)

    power_logger = PowerTelemetryLogger(robot, args, logger, "lekiwi_host")
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
    pending_ui_command: dict[str, Any] | None = None

    print(
        f"LeKiwi host listening on tcp://*:{args.port_zmq_cmd} and tcp://*:{args.port_zmq_observations}",
        flush=True,
    )
    if args.safer_servo_mode:
        print("Safer servo mode enabled: wrist continuity and bounded arm steps are active.", flush=True)

    start = time.perf_counter()
    try:
        while time.perf_counter() - start < args.connection_time_s:
            loop_start = time.time()

            if pending_ui_command is None:
                pending_ui_command = ui_command_watcher.poll()
            if pending_ui_command is not None:
                if pending_ui_command["command"] == "replay":
                    pending_ui_command = execute_replay(
                        pending_ui_command,
                        robot,
                        torque_watcher,
                        replay_safety_filter,
                        power_logger,
                        cmd_socket,
                        obs_socket,
                        ui_command_watcher,
                        args.loop_hz,
                    )
                else:
                    try:
                        robot.stop_base()
                    except Exception:
                        pass
                    pending_ui_command = None
                last_cmd_time = time.time()
                watchdog_active = False
                continue

            try:
                msg = cmd_socket.recv_string(zmq.NOBLOCK)
                action = live_safety_filter.normalize(dict(json.loads(msg)))
                sent_action = robot.send_action(action)
                live_safety_filter.update(sent_action)
                replay_safety_filter.update(sent_action)
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

            torque_watcher.poll(robot)
            publish_observation(robot, obs_socket)

            power_logger.maybe_sample()

            elapsed = time.time() - loop_start
            time.sleep(max(1.0 / args.loop_hz - elapsed, 0.0))
    except KeyboardInterrupt:
        print("\nStopping LeKiwi host", flush=True)
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
