#!/usr/bin/env python3

from __future__ import annotations

import argparse
import base64
import json
import logging
import math
import runpy
import time
from pathlib import Path
from datetime import datetime, timezone
from typing import Any

import cv2
import zmq

from lekiwi_runtime import (
    ArmSafetyFilter,
    COMMAND_SOURCE_KEYBOARD,
    DEFAULT_SAFER_ARM_MAX_STEP,
    LiveRobotSensorStatusEmitter,
    ResilientObservationReader,
    ServoProtectionSupervisor,
    TorqueLimitFileWatcher,
    apply_robot_action,
    add_servo_safety_args,
    add_torque_limit_args,
    apply_torque_limits,
    build_normalized_arm_position_limits,
    configure_wrist_roll_mode,
    disconnect_robot,
    parse_torque_limits_json,
    stop_robot_base,
)
from lekiwi_sensor_replay import (
    PREPOSITION_HEADING_CROSS_TRIM_DEG,
    PREPOSITION_LINEAR_CROSS_TRIM_M,
    SENSOR_PREPOSITION_TIMEOUT_S,
    THETA_TRACK_TOLERANCE_DEG,
    SensorAwareReplayState,
    XY_TRACK_TOLERANCE_M,
    preposition_vex_base_to_recorded_state,
    recorded_state_has_sensor_reference,
)
from vex_base_bridge import (
    VexBaseBridge,
    VexBaseTelemetryManager,
    add_vex_base_args,
    ensure_vex_command_stream,
    normalize_vex_control_config,
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
GRIPPER_STATE_KEY = "arm_gripper.pos"
ARM_HOME_MOTION_KEYS = tuple(key for key in ARM_STATE_KEYS if key != GRIPPER_STATE_KEY)
BASE_STATE_KEYS = ("x.vel", "y.vel", "theta.vel")
VEX_POSITION_LOG_PREFIX = "[vex-position]"
HOME_COMMAND_LOG_PREFIX = "[home-command]"
HOME_POSITION_TOLERANCE_DEG = 3.0
HOME_SETTLE_TIMEOUT_S = 5.0

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
        if command in {"zero-vex-gyro", "zero_vex_gyro"}:
            request_id = payload.get("request_id")
            return {
                "command": "zero-vex-gyro",
                "request_id": request_id if isinstance(request_id, str) else "",
            }
        if command in {"capture-home", "capture_home"}:
            request_id = payload.get("request_id")
            return {
                "command": "capture-home",
                "request_id": request_id if isinstance(request_id, str) else "",
            }
        if command in {"go-home", "go_home"}:
            request_id = payload.get("request_id")
            positions = payload.get("home_position")
            if not isinstance(positions, dict):
                print(f"[replay] error go-home command is missing home_position in {self.path}", flush=True)
                return None
            try:
                home_position = validate_home_position(positions)
            except Exception as exc:
                print(f"[replay] error invalid home_position in {self.path}: {exc}", flush=True)
                return None
            return {
                "command": "go-home",
                "request_id": request_id if isinstance(request_id, str) else "",
                "home_position": home_position,
            }
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

        try:
            auto_vex_positioning = coerce_json_bool(payload.get("auto_vex_positioning"), default=True)
        except argparse.ArgumentTypeError as exc:
            print(f"[replay] error auto_vex_positioning is invalid in {self.path}: {exc}", flush=True)
            return None

        try:
            vex_positioning_timeout_s = float(
                payload.get("vex_positioning_timeout_s", SENSOR_PREPOSITION_TIMEOUT_S)
            )
        except (TypeError, ValueError):
            print(f"[replay] error vex_positioning_timeout_s is invalid in {self.path}", flush=True)
            return None
        if vex_positioning_timeout_s <= 0:
            print(f"[replay] error vex_positioning_timeout_s must be greater than 0 in {self.path}", flush=True)
            return None

        try:
            vex_positioning_speed = float(payload.get("vex_positioning_speed", 1.0))
        except (TypeError, ValueError):
            print(f"[replay] error vex_positioning_speed is invalid in {self.path}", flush=True)
            return None
        if vex_positioning_speed <= 0:
            print(f"[replay] error vex_positioning_speed must be greater than 0 in {self.path}", flush=True)
            return None

        try:
            vex_positioning_xy_tolerance_m = float(
                payload.get("vex_positioning_xy_tolerance_m", XY_TRACK_TOLERANCE_M)
            )
        except (TypeError, ValueError):
            print(f"[replay] error vex_positioning_xy_tolerance_m is invalid in {self.path}", flush=True)
            return None
        if vex_positioning_xy_tolerance_m <= 0:
            print(
                f"[replay] error vex_positioning_xy_tolerance_m must be greater than 0 in {self.path}",
                flush=True,
            )
            return None

        try:
            vex_positioning_heading_tolerance_deg = float(
                payload.get("vex_positioning_heading_tolerance_deg", THETA_TRACK_TOLERANCE_DEG)
            )
        except (TypeError, ValueError):
            print(f"[replay] error vex_positioning_heading_tolerance_deg is invalid in {self.path}", flush=True)
            return None
        if vex_positioning_heading_tolerance_deg <= 0:
            print(
                f"[replay] error vex_positioning_heading_tolerance_deg must be greater than 0 in {self.path}",
                flush=True,
            )
            return None

        try:
            vex_positioning_xy_trim_tolerance_m = float(
                payload.get("vex_positioning_xy_trim_tolerance_m", PREPOSITION_LINEAR_CROSS_TRIM_M)
            )
        except (TypeError, ValueError):
            print(f"[replay] error vex_positioning_xy_trim_tolerance_m is invalid in {self.path}", flush=True)
            return None
        if vex_positioning_xy_trim_tolerance_m <= 0:
            print(
                f"[replay] error vex_positioning_xy_trim_tolerance_m must be greater than 0 in {self.path}",
                flush=True,
            )
            return None

        try:
            vex_positioning_heading_trim_tolerance_deg = float(
                payload.get("vex_positioning_heading_trim_tolerance_deg", PREPOSITION_HEADING_CROSS_TRIM_DEG)
            )
        except (TypeError, ValueError):
            print(f"[replay] error vex_positioning_heading_trim_tolerance_deg is invalid in {self.path}", flush=True)
            return None
        if vex_positioning_heading_trim_tolerance_deg <= 0:
            print(
                f"[replay] error vex_positioning_heading_trim_tolerance_deg must be greater than 0 in {self.path}",
                flush=True,
            )
            return None

        vex_replay_mode = payload.get("vex_replay_mode", "ecu")
        if vex_replay_mode not in {"drive", "ecu"}:
            print(
                f"[replay] error vex_replay_mode must be 'drive' or 'ecu' in {self.path}",
                flush=True,
            )
            return None

        home_mode = payload.get("home_mode", "none")
        if home_mode not in {"none", "start", "end", "both"}:
            print(
                f"[replay] error home_mode must be 'none', 'start', 'end', or 'both' in {self.path}",
                flush=True,
            )
            return None
        home_position = None
        if home_mode != "none":
            try:
                home_position = validate_home_position(payload.get("home_position"))
            except Exception as exc:
                print(f"[replay] error invalid home_position in {self.path}: {exc}", flush=True)
                return None

        return {
            "command": "replay",
            "trajectory_path": trajectory_path.strip(),
            "speed": speed,
            "hold_final_s": max(0.0, hold_final_s),
            "include_base": include_base,
            "auto_vex_positioning": auto_vex_positioning,
            "vex_positioning_speed": vex_positioning_speed,
            "vex_positioning_timeout_s": vex_positioning_timeout_s,
            "vex_positioning_xy_tolerance_m": vex_positioning_xy_tolerance_m,
            "vex_positioning_heading_tolerance_deg": vex_positioning_heading_tolerance_deg,
            "vex_positioning_xy_trim_tolerance_m": vex_positioning_xy_trim_tolerance_m,
            "vex_positioning_heading_trim_tolerance_deg": vex_positioning_heading_trim_tolerance_deg,
            "vex_replay_mode": vex_replay_mode,
            "home_mode": home_mode,
            "home_position": home_position,
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


def validate_home_position(value: Any) -> dict[str, float]:
    if not isinstance(value, dict):
        raise ValueError("home_position must be an object")
    home_position: dict[str, float] = {}
    for key in ARM_STATE_KEYS:
        raw_value = value.get(key)
        if not isinstance(raw_value, (int, float)):
            raise ValueError(f"home_position is missing numeric {key}")
        home_position[key] = float(raw_value)
    return home_position


def extract_home_position(observation: dict[str, Any]) -> dict[str, float]:
    return validate_home_position({key: observation.get(key) for key in ARM_STATE_KEYS})


def home_motion_errors(
    observation: dict[str, Any],
    home_position: dict[str, float],
) -> dict[str, float]:
    errors: dict[str, float] = {}
    for key in ARM_HOME_MOTION_KEYS:
        value = observation.get(key)
        if isinstance(value, (int, float)) and math.isfinite(float(value)):
            errors[key] = abs(float(value) - home_position[key])
        else:
            errors[key] = float("inf")
    return errors


def max_home_motion_error(errors: dict[str, float]) -> float:
    finite_errors = [error for error in errors.values() if math.isfinite(error)]
    return max(finite_errors, default=float("inf"))


def format_home_motion_errors(errors: dict[str, float]) -> str:
    if not errors:
        return "no joint observations"
    return ", ".join(
        f"{key}={error:.2f}"
        if math.isfinite(error)
        else f"{key}=unavailable"
        for key, error in errors.items()
    )


def home_result_payload(
    command: str,
    request_id: str,
    *,
    success: bool,
    status: str,
    positions: dict[str, float] | None = None,
) -> dict[str, Any]:
    payload: dict[str, Any] = {
        "command": command,
        "request_id": request_id,
        "success": success,
        "status": status,
    }
    if positions is not None:
        payload["positions"] = positions
        payload["captured_at"] = datetime.now(timezone.utc).isoformat(timespec="seconds")
    return payload


def print_home_command_result(payload: dict[str, Any]) -> None:
    print(f"{HOME_COMMAND_LOG_PREFIX} {json.dumps(payload, separators=(',', ':'))}", flush=True)


def publish_observation(
    observation_reader: ResilientObservationReader,
    obs_socket: Any,
    vex_base_bridge: VexBaseBridge | None = None,
    sensor_status_emitter: LiveRobotSensorStatusEmitter | None = None,
    servo_protection: ServoProtectionSupervisor | None = None,
    *,
    sensor_status_source: str = "host-control",
) -> dict[str, Any] | None:
    try:
        observation = observation_reader.get_observation()
    except Exception as exc:
        logger.warning("Observation read failed: %s", exc)
        return None

    if vex_base_bridge is not None:
        observation = vex_base_bridge.merge_observation(observation)

    if sensor_status_emitter is not None:
        sensor_status_emitter.emit(
            observation_reader,
            observation,
            source=sensor_status_source,
            vex_base_bridge=vex_base_bridge,
        )

    if servo_protection is not None:
        observation = servo_protection.enrich_observation(observation)

    for cam_key in observation_reader.robot.cameras:
        frame = observation.get(cam_key)
        if isinstance(frame, str) or frame is None:
            observation[cam_key] = ""
            continue
        ret, buffer = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        observation[cam_key] = base64.b64encode(buffer).decode("utf-8") if ret else ""

    try:
        obs_socket.send_string(json.dumps(observation), flags=zmq.NOBLOCK)
    except zmq.Again:
        pass
    except Exception as exc:
        logger.warning("Observation publish failed: %s", exc)

    return observation


def stop_replay_motion(
    robot: LeKiwi,
    last_action: dict[str, float] | None,
    *,
    allow_legacy_base: bool,
) -> None:
    try:
        if last_action is not None and allow_legacy_base:
            robot.send_action(
                {
                    **{
                        key: value
                        for key, value in last_action.items()
                        if key.endswith(".pos") and isinstance(value, (int, float))
                    },
                    **dict.fromkeys(BASE_STATE_KEYS, 0.0),
                }
            )
        elif allow_legacy_base:
            stop_robot_base(robot, allow_legacy_base=True)
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


def send_vex_live_base_motion(
    vex_base_bridge: VexBaseBridge,
    action: dict[str, Any],
    *,
    context: str,
) -> bool:
    sent = vex_base_bridge.send_motion(
        {
            key: float(action.get(key, 0.0)) if isinstance(action.get(key, 0.0), (int, float)) else 0.0
            for key in BASE_STATE_KEYS
        }
    )
    if not sent:
        logger.warning(
            "VEX live base command was not accepted during %s: %s",
            context,
            vex_base_bridge.status_message,
        )
    return sent


def print_vex_position_status(
    status: str,
    *,
    reason: str | None = None,
    timeout_s: float = SENSOR_PREPOSITION_TIMEOUT_S,
    message: str | None = None,
    arm_replay: str = "continuing",
) -> None:
    payload: dict[str, Any] = {
        "status": status,
        "timeout_s": round(float(timeout_s), 3) if float(timeout_s) > 0 else None,
        "arm_replay": arm_replay,
    }
    if reason:
        payload["reason"] = reason
    if message:
        payload["message"] = message
    print(f"{VEX_POSITION_LOG_PREFIX} {json.dumps(payload, separators=(',', ':'))}", flush=True)


def execute_zero_vex_gyro_command(
    command: dict[str, Any],
    vex_base_bridge: VexBaseBridge | None,
) -> None:
    result: dict[str, Any] = {
        "command": "zero-vex-gyro",
        "request_id": command.get("request_id") or "",
        "success": False,
        "status": "VEX base bridge is unavailable.",
        "gyro": None,
    }
    if vex_base_bridge is None:
        print(f"[vex-command] {json.dumps(result, separators=(',', ':'))}", flush=True)
        return

    try:
        result["success"] = vex_base_bridge.set_pose_origin(ttl_ms=1200, timeout_s=2.0)
        time.sleep(0.2)
        vex_base_bridge.poll()
        gyro = vex_base_bridge.gyro_status_snapshot()
        result["gyro"] = gyro
        result["status"] = (
            gyro.get("message")
            if result["success"] and isinstance(gyro, dict) and gyro.get("message")
            else "Timed out waiting for the VEX Brain to confirm gyro zero."
        )
    except Exception as exc:
        result["status"] = f"VEX gyro zero failed: {exc}"

    print(f"[vex-command] {json.dumps(result, separators=(',', ':'))}", flush=True)


def execute_capture_home_command(
    command: dict[str, Any],
    observation_reader: ResilientObservationReader,
) -> None:
    request_id = command.get("request_id") or ""
    try:
        observation = observation_reader.get_observation()
        positions = extract_home_position(observation)
        print_home_command_result(
            home_result_payload(
                "capture-home",
                request_id,
                success=True,
                status="Captured current arm pose as home.",
                positions=positions,
            )
        )
    except Exception as exc:
        print_home_command_result(
            home_result_payload(
                "capture-home",
                request_id,
                success=False,
                status=f"Could not capture arm home position: {exc}",
            )
        )


def execute_go_home_command(
    command: dict[str, Any],
    robot: LeKiwi,
    observation_reader: ResilientObservationReader,
    sensor_status_emitter: LiveRobotSensorStatusEmitter,
    servo_protection: ServoProtectionSupervisor,
    torque_watcher: TorqueLimitFileWatcher,
    safety_filter: ArmSafetyFilter,
    power_logger: Any,
    obs_socket: Any,
    vex_base_bridge: VexBaseBridge | None,
    *,
    loop_hz: float,
    allow_legacy_base: bool,
    should_stop: Any | None = None,
    emit_result: bool = True,
    source: str = "host-home",
) -> dict[str, float] | None:
    request_id = command.get("request_id") or ""
    try:
        if servo_protection.latched:
            raise RuntimeError("arm safety latch is active; restart the Pi host after inspection")

        home_position = validate_home_position(command.get("home_position"))
        observation = observation_reader.get_observation()
        current_position = extract_home_position(observation)
        safety_filter.seed_from_observation(observation)

        max_steps = 1
        for key in ARM_HOME_MOTION_KEYS:
            target = home_position[key]
            start_value = current_position[key]
            step_limit = DEFAULT_SAFER_ARM_MAX_STEP.get(key, 8.0)
            if step_limit > 0:
                max_steps = max(max_steps, math.ceil(abs(target - start_value) / step_limit))
        max_steps = max(1, min(600, max_steps))
        sleep_step_s = 1.0 / max(loop_hz, 1.0)
        last_action: dict[str, float] | None = None
        final_errors: dict[str, float] = {}

        for step_index in range(1, max_steps + 1):
            if should_stop is not None and should_stop():
                raise RuntimeError("go-home stopped by user")
            progress = step_index / max_steps
            target_action = {
                key: current_position[key] + (home_position[key] - current_position[key]) * progress
                for key in ARM_HOME_MOTION_KEYS
            }
            target_action.update(dict.fromkeys(BASE_STATE_KEYS, 0.0))
            torque_watcher.poll(robot)
            action = safety_filter.normalize(target_action)
            sent_action = apply_robot_action(
                robot,
                action,
                allow_legacy_base=allow_legacy_base,
            )
            safety_filter.update(sent_action)
            servo_protection.record_command(sent_action)
            last_action = sent_action
            if vex_base_bridge is not None:
                vex_base_bridge.send_hold()
            power_logger.maybe_sample()
            observation = publish_observation(
                observation_reader,
                obs_socket,
                vex_base_bridge,
                sensor_status_emitter,
                servo_protection,
                sensor_status_source=source,
            )
            if observation is not None and servo_protection.observe(observation, power_logger.last_sample):
                raise RuntimeError("arm safety latch tripped during go-home")
            precise_sleep(sleep_step_s)

        settle_deadline = time.perf_counter() + HOME_SETTLE_TIMEOUT_S
        while True:
            if observation is not None:
                final_errors = home_motion_errors(observation, home_position)
                if max_home_motion_error(final_errors) <= HOME_POSITION_TOLERANCE_DEG:
                    break

            if time.perf_counter() >= settle_deadline:
                raise RuntimeError(
                    "home target was commanded but not reached within "
                    f"{HOME_SETTLE_TIMEOUT_S:.1f}s; errors: {format_home_motion_errors(final_errors)}"
                )
            if should_stop is not None and should_stop():
                raise RuntimeError("go-home stopped by user")

            target_action = {key: home_position[key] for key in ARM_HOME_MOTION_KEYS}
            target_action.update(dict.fromkeys(BASE_STATE_KEYS, 0.0))
            torque_watcher.poll(robot)
            action = safety_filter.normalize(target_action)
            sent_action = apply_robot_action(
                robot,
                action,
                allow_legacy_base=allow_legacy_base,
            )
            safety_filter.update(sent_action)
            servo_protection.record_command(sent_action)
            last_action = sent_action
            if vex_base_bridge is not None:
                vex_base_bridge.send_hold()
            power_logger.maybe_sample()
            observation = publish_observation(
                observation_reader,
                obs_socket,
                vex_base_bridge,
                sensor_status_emitter,
                servo_protection,
                sensor_status_source=source,
            )
            if observation is not None and servo_protection.observe(observation, power_logger.last_sample):
                raise RuntimeError("arm safety latch tripped during go-home")
            precise_sleep(sleep_step_s)

        if emit_result:
            print_home_command_result(
                home_result_payload(
                    "go-home",
                    request_id,
                    success=True,
                    status=(
                        "Arm moved to saved home position without commanding the gripper "
                        f"(max joint error {max_home_motion_error(final_errors):.2f}deg)."
                    ),
                    positions=home_position,
                )
            )
        return last_action
    except Exception as exc:
        if emit_result:
            print_home_command_result(
                home_result_payload(
                    "go-home",
                    request_id,
                    success=False,
                    status=f"Could not move arm home: {exc}",
                )
            )
        return None


def execute_replay(
    command: dict[str, Any],
    robot: LeKiwi,
    observation_reader: ResilientObservationReader,
    sensor_status_emitter: LiveRobotSensorStatusEmitter,
    servo_protection: ServoProtectionSupervisor,
    torque_watcher: TorqueLimitFileWatcher,
    replay_filter: ArmSafetyFilter,
    power_logger: Any,
    cmd_socket: Any,
    obs_socket: Any,
    ui_command_watcher: UiCommandWatcher,
    vex_base_bridge: VexBaseBridge | None,
    vex_base_telemetry: VexBaseTelemetryManager | None,
    vex_control_config: dict[str, Any],
    vex_telemetry_program_name: str,
    loop_hz: float,
    allow_legacy_base: bool,
) -> dict[str, Any] | None:
    if servo_protection.latched:
        print("[replay] ignored because the arm safety latch is active. Restart the Pi host after inspection.", flush=True)
        return None

    try:
        trajectory_path, samples = load_trajectory(command["trajectory_path"])
    except Exception as exc:
        print(f"[replay] error {exc}", flush=True)
        return None

    speed = float(command["speed"])
    hold_final_s = float(command["hold_final_s"])
    include_base = bool(command["include_base"])
    auto_vex_positioning = bool(command.get("auto_vex_positioning", True))
    vex_positioning_speed = float(command.get("vex_positioning_speed", 1.0))
    vex_positioning_timeout_s = float(command.get("vex_positioning_timeout_s", SENSOR_PREPOSITION_TIMEOUT_S))
    vex_positioning_xy_tolerance_m = float(
        command.get("vex_positioning_xy_tolerance_m", XY_TRACK_TOLERANCE_M)
    )
    vex_positioning_heading_tolerance_deg = float(
        command.get("vex_positioning_heading_tolerance_deg", THETA_TRACK_TOLERANCE_DEG)
    )
    vex_positioning_xy_trim_tolerance_m = float(
        command.get("vex_positioning_xy_trim_tolerance_m", PREPOSITION_LINEAR_CROSS_TRIM_M)
    )
    vex_positioning_heading_trim_tolerance_deg = float(
        command.get("vex_positioning_heading_trim_tolerance_deg", PREPOSITION_HEADING_CROSS_TRIM_DEG)
    )
    vex_replay_mode = command.get("vex_replay_mode", "ecu")
    home_mode = command.get("home_mode", "none")
    home_position = command.get("home_position")
    sleep_step_s = 1.0 / max(loop_hz, 1.0)
    replay_to_vex_base = include_base and not allow_legacy_base
    start_state = samples[0]["state"] if samples and isinstance(samples[0], dict) else None
    auto_positioning_available = (
        not allow_legacy_base
        and isinstance(start_state, dict)
        and recorded_state_has_sensor_reference(start_state)
    )
    auto_preposition_base = auto_vex_positioning and auto_positioning_available
    prepare_vex_base = replay_to_vex_base or auto_preposition_base

    if prepare_vex_base:
        if vex_base_bridge is None:
            if replay_to_vex_base:
                print("[replay] error VEX base replay requested, but the serial bridge is unavailable.", flush=True)
                return None
            print(
                "[replay] warning recorded ultrasonic/gyro start alignment is available, but the VEX serial bridge is unavailable.",
                flush=True,
            )
            print_vex_position_status(
                "skipped",
                reason="serial-bridge-unavailable",
                message="VEX start positioning skipped because the serial bridge is unavailable.",
                timeout_s=vex_positioning_timeout_s,
            )
            prepare_vex_base = False
            auto_preposition_base = False
        else:
            vex_base_bridge.connect()
            if vex_base_telemetry is not None:
                vex_base_telemetry.refresh()
            if not ensure_vex_command_stream(
                vex_base_bridge,
                vex_base_telemetry,
                vex_control_config,
                program_name=vex_telemetry_program_name,
                logger=logger,
            ):
                if replay_to_vex_base:
                    print("[replay] error failed to start the live VEX command/telemetry program.", flush=True)
                    return None
                print(
                    "[replay] warning recorded ultrasonic/gyro start alignment is available, but the live VEX command/telemetry program could not be started.",
                    flush=True,
                )
                print_vex_position_status(
                    "skipped",
                    reason="command-stream-unavailable",
                    message="VEX start positioning skipped because the Brain did not accept live USB commands.",
                    timeout_s=vex_positioning_timeout_s,
                )
                prepare_vex_base = False
                auto_preposition_base = False

    print(
        "[replay] start "
        f"path={trajectory_path} samples={len(samples)} speed={speed:.3f} include_base={str(include_base).lower()} vex_mode={vex_replay_mode} auto_vex_positioning={str(auto_vex_positioning).lower()} start_align={str(auto_preposition_base).lower()} home_mode={home_mode}",
        flush=True,
    )
    if auto_positioning_available and not auto_vex_positioning:
        print_vex_position_status(
            "skipped",
            reason="disabled",
            message="VEX start positioning is disabled for this replay.",
            timeout_s=vex_positioning_timeout_s,
        )

    last_action: dict[str, float] | None = None
    replay_state = None
    last_base_command_at: float | None = None
    vex_base_control_used = False
    if home_mode in {"start", "both"}:
        print("[replay] moving arm to saved home before replay.", flush=True)

        def should_stop_home_start() -> bool:
            next_command = ui_command_watcher.poll()
            return bool(next_command and next_command["command"] == "stop-replay")

        last_action = execute_go_home_command(
            {
                "command": "go-home",
                "request_id": "",
                "home_position": home_position,
            },
            robot,
            observation_reader,
            sensor_status_emitter,
            servo_protection,
            torque_watcher,
            replay_filter,
            power_logger,
            obs_socket,
            vex_base_bridge,
            loop_hz=loop_hz,
            allow_legacy_base=allow_legacy_base,
            should_stop=should_stop_home_start,
            emit_result=False,
            source="host-replay-home",
        )
        if last_action is None:
            print("[replay] error failed to move arm home before replay; replay aborted.", flush=True)
            return None

    if prepare_vex_base and vex_base_bridge is not None:
        replay_state = SensorAwareReplayState(
            samples,
            replay_mode=vex_replay_mode,
            speed=speed,
            control_config=vex_control_config,
        )
        if auto_vex_positioning and isinstance(start_state, dict):
            if auto_preposition_base:
                print("[replay] aligning VEX base to recorded ultrasonic/gyro start pose.", flush=True)
            stop_requested = False

            def should_stop_preposition() -> bool:
                nonlocal stop_requested
                next_command = ui_command_watcher.poll()
                if next_command and next_command["command"] == "stop-replay":
                    stop_requested = True
                    return True
                return False

            prepositioned = preposition_vex_base_to_recorded_state(
                vex_base_bridge,
                observation_reader,
                replay_state,
                start_state,
                timeout_s=vex_positioning_timeout_s,
                xy_tolerance_m=vex_positioning_xy_tolerance_m,
                heading_tolerance_deg=vex_positioning_heading_tolerance_deg,
                xy_trim_tolerance_m=vex_positioning_xy_trim_tolerance_m,
                heading_trim_tolerance_deg=vex_positioning_heading_trim_tolerance_deg,
                speed_scale=vex_positioning_speed,
                sensor_status_emitter=sensor_status_emitter,
                sensor_status_source="host-replay-preposition",
                should_stop=should_stop_preposition,
            )
            vex_base_control_used = True
            if not prepositioned:
                if stop_requested:
                    print(f"[replay] stopped path={trajectory_path}", flush=True)
                    return None
                failure_reason = getattr(prepositioned, "reason", None) or "not-aligned"
                failure_axis = getattr(prepositioned, "axis", None)
                failure_detail = getattr(prepositioned, "detail", None)
                failure_message = "VEX start positioning stopped"
                if failure_axis:
                    failure_message += f" on {failure_axis}"
                if failure_detail:
                    failure_message += f": {failure_detail}"
                failure_message += "; arm replay was aborted."
                print(
                    "[replay] warning stopped recentering the VEX base before the recorded X/Y/gyro start was aligned.",
                    flush=True,
                )
                print_vex_position_status(
                    "skipped",
                    reason=failure_reason,
                    message=failure_message,
                    timeout_s=vex_positioning_timeout_s,
                    arm_replay="aborted",
                )
                return None
            elif auto_preposition_base:
                print_vex_position_status(
                    "aligned",
                    message="VEX start positioning completed before arm replay.",
                    timeout_s=vex_positioning_timeout_s,
                )
        replay_state.prepare()
    drain_command_socket(cmd_socket)
    start = time.perf_counter()

    try:
        for index, sample in enumerate(samples, start=1):
            next_command = ui_command_watcher.poll()
            if next_command is not None:
                if next_command["command"] == "stop-replay":
                    print(f"[replay] stopped path={trajectory_path}", flush=True)
                    return None
                if next_command["command"] != "replay":
                    print(
                        f"[replay] ignored command={next_command['command']} while replaying path={trajectory_path}",
                        flush=True,
                    )
                    continue
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
            sent_action = apply_robot_action(
                robot,
                action,
                allow_legacy_base=allow_legacy_base,
            )
            replay_filter.update(sent_action)
            servo_protection.record_command(sent_action)
            last_action = sent_action
            power_logger.maybe_sample()
            if replay_to_vex_base and replay_state is not None and vex_base_bridge is not None:
                command_now = time.perf_counter()
                command_dt_s = (
                    command_now - last_base_command_at
                    if last_base_command_at is not None
                    else max(0.02, target_t)
                )
                base_command = replay_state.build_replay_command(
                    state,
                    command_dt_s=command_dt_s,
                )
                if base_command.mode == "ecu" and base_command.ecu_targets is not None:
                    vex_base_bridge.send_ecu_targets(
                        base_command.ecu_targets,
                        motion=base_command.motion,
                        command_dt_ms=base_command.command_dt_ms,
                    )
                else:
                    vex_base_bridge.send_motion(base_command.motion)
                last_base_command_at = command_now
            observation = publish_observation(
                observation_reader,
                obs_socket,
                vex_base_bridge,
                sensor_status_emitter,
                servo_protection,
                sensor_status_source="host-replay",
            )
            if observation is not None and servo_protection.observe(observation, power_logger.last_sample):
                print("[replay] stopped because the arm safety latch tripped.", flush=True)
                return None

        if last_action is not None and hold_final_s > 0:
            hold_until = time.perf_counter() + hold_final_s
            if replay_to_vex_base and vex_base_bridge is not None:
                vex_base_bridge.send_hold()
            while time.perf_counter() < hold_until:
                next_command = ui_command_watcher.poll()
                if next_command is not None:
                    if next_command["command"] == "stop-replay":
                        print(f"[replay] stopped path={trajectory_path}", flush=True)
                        return None
                    if next_command["command"] != "replay":
                        print(
                            f"[replay] ignored command={next_command['command']} while holding path={trajectory_path}",
                            flush=True,
                        )
                        continue
                    print(
                        f"[replay] interrupted path={trajectory_path} next={next_command['trajectory_path']}",
                        flush=True,
                    )
                    return next_command

                torque_watcher.poll(robot)
                power_logger.maybe_sample()
                observation = publish_observation(
                    observation_reader,
                    obs_socket,
                    vex_base_bridge,
                    sensor_status_emitter,
                    servo_protection,
                    sensor_status_source="host-replay",
                )
                if observation is not None and servo_protection.observe(observation, power_logger.last_sample):
                    print("[replay] stopped because the arm safety latch tripped.", flush=True)
                    return None
                precise_sleep(min(sleep_step_s, max(hold_until - time.perf_counter(), 0.0)))

        if home_mode in {"end", "both"}:
            print("[replay] moving arm to saved home after replay.", flush=True)

            def should_stop_home_end() -> bool:
                next_command = ui_command_watcher.poll()
                return bool(next_command and next_command["command"] == "stop-replay")

            home_action = execute_go_home_command(
                {
                    "command": "go-home",
                    "request_id": "",
                    "home_position": home_position,
                },
                robot,
                observation_reader,
                sensor_status_emitter,
                servo_protection,
                torque_watcher,
                replay_filter,
                power_logger,
                obs_socket,
                vex_base_bridge,
                loop_hz=loop_hz,
                allow_legacy_base=allow_legacy_base,
                should_stop=should_stop_home_end,
                emit_result=False,
                source="host-replay-home",
            )
            if home_action is not None:
                last_action = home_action
            else:
                print("[replay] warning failed to move arm home after replay.", flush=True)

        print(f"[replay] complete path={trajectory_path}", flush=True)
        return None
    except Exception as exc:
        print(f"[replay] error path={trajectory_path}: {exc}", flush=True)
        return None
    finally:
        stop_replay_motion(
            robot,
            last_action,
            allow_legacy_base=allow_legacy_base,
        )
        if (replay_to_vex_base or vex_base_control_used) and vex_base_bridge is not None:
            vex_base_bridge.send_hold()
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
    add_vex_base_args(parser)
    parser.add_argument("--vex-live-base-control", type=parse_bool, default=False)
    parser.add_argument("--require-vex-live-base-control", type=parse_bool, default=False)
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
    absolute_position_limits = build_normalized_arm_position_limits(
        robot,
        preserve_continuous_wrist_roll=args.safer_servo_mode,
    )
    ui_command_watcher = UiCommandWatcher(args.ui_command_path)
    live_safety_filter = ArmSafetyFilter(
        enabled=args.safer_servo_mode,
        map_wrist_to_follower_start=True,
        absolute_position_limits=absolute_position_limits,
        skip_step_limit_sources=(COMMAND_SOURCE_KEYBOARD,),
    )
    replay_safety_filter = ArmSafetyFilter(
        enabled=args.safer_servo_mode,
        absolute_position_limits=absolute_position_limits,
    )
    servo_protection = ServoProtectionSupervisor(robot, logger)
    observation_reader = ResilientObservationReader(robot, logger)
    sensor_status_emitter = LiveRobotSensorStatusEmitter()
    try:
        observation = observation_reader.get_observation()
        live_safety_filter.seed_from_observation(observation)
        replay_safety_filter.seed_from_observation(observation)
        servo_protection.seed_from_observation(observation)
    except Exception as exc:
        logger.warning("Failed to seed servo safety filter from the current robot state: %s", exc)

    power_logger = PowerTelemetryLogger(robot, args, logger, "lekiwi_host")
    power_logger.start()
    vex_base_bridge = VexBaseBridge(
        requested_port=args.vex_base_port,
        baudrate=args.vex_base_baudrate,
        stale_after_s=args.vex_base_stale_after_s,
        command_timeout_s=args.vex_base_command_timeout_s,
        logger=logger,
    )
    vex_base_bridge.connect()
    vex_control_config = normalize_vex_control_config(json.loads(args.vex_control_config_json))
    vex_base_telemetry = VexBaseTelemetryManager(
        requested_vexcom_path=args.vex_vexcom_path,
        telemetry_slot=args.vex_telemetry_slot,
        cache_dir=args.vex_program_cache_dir,
        logger=logger,
    )

    print(vex_base_bridge.status_message, flush=True)
    print(vex_base_telemetry.status_message, flush=True)
    vex_live_base_control_ready = False
    if args.vex_live_base_control and not args.enable_base:
        vex_live_base_control_ready = ensure_vex_command_stream(
            vex_base_bridge,
            vex_base_telemetry,
            vex_control_config,
            program_name=args.vex_telemetry_program_name,
            logger=logger,
        )
        if vex_live_base_control_ready:
            print("VEX live base control is ready for keyboard/teleop velocity commands.", flush=True)
        else:
            message = (
                "VEX live base control requested, but the Brain did not accept live USB commands. "
                f"{vex_base_bridge.status_message} {vex_base_telemetry.status_message}"
            )
            if args.require_vex_live_base_control:
                try:
                    disconnect_robot(robot, allow_legacy_base=args.enable_base)
                except Exception:
                    pass
                vex_base_bridge.close()
                try:
                    power_logger.close()
                except Exception:
                    pass
                raise SystemExit(f"Failed: {message}")
            print(f"Warning: {message}", flush=True)

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
        print(
            "Safer servo mode enabled: calibrated joint clamps, wrist continuity, and bounded arm steps are active.",
            flush=True,
        )

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
                        observation_reader,
                        sensor_status_emitter,
                        servo_protection,
                        torque_watcher,
                        replay_safety_filter,
                        power_logger,
                        cmd_socket,
                        obs_socket,
                        ui_command_watcher,
                        vex_base_bridge,
                        vex_base_telemetry,
                        vex_control_config,
                        args.vex_telemetry_program_name,
                        args.loop_hz,
                        args.enable_base,
                    )
                elif pending_ui_command["command"] == "zero-vex-gyro":
                    execute_zero_vex_gyro_command(pending_ui_command, vex_base_bridge)
                    pending_ui_command = None
                elif pending_ui_command["command"] == "capture-home":
                    execute_capture_home_command(pending_ui_command, observation_reader)
                    pending_ui_command = None
                elif pending_ui_command["command"] == "go-home":
                    home_action = execute_go_home_command(
                        pending_ui_command,
                        robot,
                        observation_reader,
                        sensor_status_emitter,
                        servo_protection,
                        torque_watcher,
                        live_safety_filter,
                        power_logger,
                        obs_socket,
                        vex_base_bridge,
                        loop_hz=args.loop_hz,
                        allow_legacy_base=args.enable_base,
                    )
                    if home_action is not None:
                        replay_safety_filter.update(home_action)
                    drain_command_socket(cmd_socket)
                    pending_ui_command = None
                else:
                    try:
                        stop_robot_base(robot, allow_legacy_base=args.enable_base)
                    except Exception:
                        pass
                    pending_ui_command = None
                last_cmd_time = time.time()
                watchdog_active = False
                continue

            try:
                msg = cmd_socket.recv_string(zmq.NOBLOCK)
                if not servo_protection.latched:
                    action = live_safety_filter.normalize(dict(json.loads(msg)))
                    sent_action = apply_robot_action(
                        robot,
                        action,
                        allow_legacy_base=args.enable_base,
                    )
                    if vex_live_base_control_ready:
                        send_vex_live_base_motion(vex_base_bridge, sent_action, context="live control")
                    live_safety_filter.update(sent_action)
                    replay_safety_filter.update(sent_action)
                    servo_protection.record_command(sent_action)
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
                stop_robot_base(robot, allow_legacy_base=args.enable_base)
                if vex_live_base_control_ready:
                    vex_base_bridge.send_hold()
                watchdog_active = True

            torque_watcher.poll(robot)
            observation = publish_observation(
                observation_reader,
                obs_socket,
                vex_base_bridge,
                sensor_status_emitter,
                servo_protection,
                sensor_status_source="host-control",
            )
            power_logger.maybe_sample()
            if observation is not None and servo_protection.observe(observation, power_logger.last_sample):
                stop_robot_base(robot, allow_legacy_base=args.enable_base)
                if vex_live_base_control_ready:
                    vex_base_bridge.send_hold()
                watchdog_active = True

            elapsed = time.time() - loop_start
            time.sleep(max(1.0 / args.loop_hz - elapsed, 0.0))
    except KeyboardInterrupt:
        print("\nStopping LeKiwi host", flush=True)
    finally:
        if vex_live_base_control_ready:
            vex_base_bridge.send_hold()
        try:
            disconnect_robot(robot, allow_legacy_base=args.enable_base)
        except Exception as exc:
            print(f"disconnect: {exc}", flush=True)
        vex_base_bridge.close()
        power_logger.maybe_sample(force=True)
        power_logger.close()
        cmd_socket.close()
        obs_socket.close()
        ctx.term()


if __name__ == "__main__":
    main()
