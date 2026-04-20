#!/usr/bin/env python3

import argparse
import json
import logging
import time
from pathlib import Path
from typing import Any

ARM_MOTORS = (
    "arm_shoulder_pan",
    "arm_shoulder_lift",
    "arm_elbow_flex",
    "arm_wrist_flex",
    "arm_wrist_roll",
    "arm_gripper",
)

TORQUE_LIMIT_MIN = 0
TORQUE_LIMIT_MAX = 1000
ARM_STATE_KEYS = tuple(f"{motor}.pos" for motor in ARM_MOTORS)
BASE_STATE_KEYS = ("x.vel", "y.vel", "theta.vel")
WRIST_ROLL_KEY = "arm_wrist_roll.pos"
GRIPPER_KEY = "arm_gripper.pos"
WRIST_ROLL_MOTOR = "arm_wrist_roll"
WRIST_ROLL_SINGLE_TURN_LIMITS = (0, 4095)
WRIST_ROLL_CONTINUOUS_LIMITS = (0, 0)

DEFAULT_SAFER_ARM_MAX_STEP = {
    "arm_shoulder_pan.pos": 8.0,
    "arm_shoulder_lift.pos": 8.0,
    "arm_elbow_flex.pos": 8.0,
    "arm_wrist_flex.pos": 10.0,
    "arm_wrist_roll.pos": 12.0,
    "arm_gripper.pos": 2.0,
}
DEFAULT_SAFER_MAX_RELATIVE_TARGET = {
    "arm_shoulder_pan.pos": 10.0,
    "arm_shoulder_lift.pos": 10.0,
    "arm_elbow_flex.pos": 10.0,
    "arm_wrist_flex.pos": 12.0,
    "arm_wrist_roll.pos": 720.0,
    "arm_gripper.pos": 3.0,
}
SAFE_ABSOLUTE_POSITION_LIMITS = {
    GRIPPER_KEY: (0.0, 85.0),
}
RECOMMENDED_ARM_TORQUE_LIMITS = {
    "arm_shoulder_pan": 500,
    "arm_shoulder_lift": 650,
    "arm_elbow_flex": 600,
    "arm_wrist_flex": 450,
    "arm_wrist_roll": 250,
    "arm_gripper": 1000,
}


def wrap_degrees(value: float) -> float:
    return ((float(value) + 180.0) % 360.0) - 180.0


def align_degrees_near_reference(value: float, reference: float) -> float:
    return float(reference) + wrap_degrees(float(value) - float(reference))


def normalize_arm_action(action: dict[str, float], *, wrap_wrist_roll: bool = True) -> dict[str, float]:
    normalized = dict(action)
    if wrap_wrist_roll and WRIST_ROLL_KEY in normalized:
        normalized[WRIST_ROLL_KEY] = wrap_degrees(normalized[WRIST_ROLL_KEY])
    return normalized


def build_safer_max_relative_target() -> dict[str, float]:
    return dict(DEFAULT_SAFER_MAX_RELATIVE_TARGET)


def configure_wrist_roll_mode(robot: Any, *, continuous: bool) -> None:
    target_min, target_max = (
        WRIST_ROLL_CONTINUOUS_LIMITS if continuous else WRIST_ROLL_SINGLE_TURN_LIMITS
    )
    current_min = robot.bus.read("Min_Position_Limit", WRIST_ROLL_MOTOR, normalize=False)
    current_max = robot.bus.read("Max_Position_Limit", WRIST_ROLL_MOTOR, normalize=False)
    if current_min == target_min and current_max == target_max:
        return

    torque_disabled = False
    try:
        robot.bus.disable_torque(WRIST_ROLL_MOTOR, num_retry=5)
        torque_disabled = True
        robot.bus.write("Min_Position_Limit", WRIST_ROLL_MOTOR, target_min, normalize=False, num_retry=5)
        robot.bus.write("Max_Position_Limit", WRIST_ROLL_MOTOR, target_max, normalize=False, num_retry=5)
    finally:
        if torque_disabled:
            robot.bus.enable_torque(WRIST_ROLL_MOTOR, num_retry=5)


def add_servo_safety_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--safer-servo-mode",
        action="store_true",
        help="Enable wrist continuity handling plus per-command arm step limiting.",
    )


class ResilientObservationReader:
    def __init__(
        self,
        robot: Any,
        logger: logging.Logger,
        *,
        read_retries: int = 2,
        warning_interval_s: float = 1.0,
    ) -> None:
        self.robot = robot
        self.logger = logger
        self.read_retries = max(int(read_retries), 1)
        self.warning_interval_s = max(float(warning_interval_s), 0.0)
        arm_state_cache = {
            f"{motor}.pos": 0.0 for motor in getattr(robot, "arm_motors", ARM_MOTORS)
        }
        self.cached_state: dict[str, float] = {
            **arm_state_cache,
            **dict.fromkeys(BASE_STATE_KEYS, 0.0),
        }
        self.cached_frames: dict[str, Any] = {}
        self.next_warning_at: dict[str, float] = {}

    def _warn(self, key: str, message: str, *args: Any) -> None:
        now = time.time()
        next_warning_at = self.next_warning_at.get(key, 0.0)
        if now < next_warning_at:
            return
        self.next_warning_at[key] = now + self.warning_interval_s
        self.logger.warning(message, *args)

    def _read_with_fallback(
        self,
        data_name: str,
        motors: list[str],
        cache_suffix: str,
    ) -> dict[str, float]:
        if not motors:
            return {}

        try:
            return self.robot.bus.sync_read(data_name, motors, num_retry=self.read_retries)
        except Exception as exc:
            self._warn(
                f"sync:{data_name}",
                "Observation sync_read(%s) failed for %s: %s",
                data_name,
                motors,
                exc,
            )

        values: dict[str, float] = {}
        for motor in motors:
            try:
                values[motor] = self.robot.bus.read(data_name, motor, num_retry=self.read_retries)
            except Exception as exc:
                cache_key = f"{motor}{cache_suffix}"
                values[motor] = self.cached_state.get(cache_key, 0.0)
                self._warn(
                    f"read:{data_name}:{motor}",
                    "Observation read(%s, %s) failed, using cached value %.3f: %s",
                    data_name,
                    motor,
                    values[motor],
                    exc,
                )

        return values

    def get_observation(self) -> dict[str, Any]:
        arm_motors = list(getattr(self.robot, "arm_motors", []))
        arm_pos = self._read_with_fallback("Present_Position", arm_motors, ".pos")

        base_vel = dict.fromkeys(BASE_STATE_KEYS, 0.0)
        base_motors = list(getattr(self.robot, "base_motors", []))
        if len(base_motors) >= 3:
            base_wheel_vel = self._read_with_fallback("Present_Velocity", base_motors, ".vel")
            if all(motor in base_wheel_vel for motor in base_motors[:3]):
                try:
                    base_vel = self.robot._wheel_raw_to_body(
                        base_wheel_vel[base_motors[0]],
                        base_wheel_vel[base_motors[1]],
                        base_wheel_vel[base_motors[2]],
                    )
                except Exception as exc:
                    self._warn("base:wheel_to_body", "Observation base velocity conversion failed: %s", exc)

        observation: dict[str, Any] = {f"{motor}.pos": arm_pos.get(motor, 0.0) for motor in arm_motors}
        observation.update(base_vel)

        for key, value in observation.items():
            if isinstance(value, (int, float)) and key in self.cached_state:
                self.cached_state[key] = float(value)

        for cam_key, cam in getattr(self.robot, "cameras", {}).items():
            try:
                frame = cam.read_latest()
            except Exception as exc:
                self._warn(f"camera:{cam_key}", "Camera %s read failed: %s", cam_key, exc)
                frame = None
            if frame is None:
                frame = self.cached_frames.get(cam_key)
            else:
                self.cached_frames[cam_key] = frame
            observation[cam_key] = frame if frame is not None else ""

        return observation


class ArmSafetyFilter:
    def __init__(self, enabled: bool, *, map_wrist_to_follower_start: bool = False) -> None:
        self.enabled = bool(enabled)
        self.map_wrist_to_follower_start = bool(map_wrist_to_follower_start)
        self.last_targets: dict[str, float] = {}
        self.initial_follower_wrist_roll: float | None = None
        self.initial_leader_wrist_roll: float | None = None
        self.last_input_wrist_roll: float | None = None

    def seed_from_observation(self, observation: dict[str, Any]) -> None:
        for key in ARM_STATE_KEYS:
            value = observation.get(key)
            if isinstance(value, (int, float)):
                self.last_targets[key] = float(value)
        wrist_value = observation.get(WRIST_ROLL_KEY)
        if isinstance(wrist_value, (int, float)):
            self.initial_follower_wrist_roll = float(wrist_value)

    def update(self, action: dict[str, Any]) -> None:
        for key in ARM_STATE_KEYS:
            value = action.get(key)
            if isinstance(value, (int, float)):
                self.last_targets[key] = float(value)

    def normalize(self, action: dict[str, Any]) -> dict[str, float]:
        normalized = normalize_arm_action(action, wrap_wrist_roll=not self.enabled)
        if not self.enabled:
            return normalized

        filtered = dict(normalized)
        wrist_value = filtered.get(WRIST_ROLL_KEY)
        if isinstance(wrist_value, (int, float)):
            continuous_wrist_value = float(wrist_value)
            if self.last_input_wrist_roll is not None:
                continuous_wrist_value = align_degrees_near_reference(
                    continuous_wrist_value,
                    self.last_input_wrist_roll,
                )
            self.last_input_wrist_roll = continuous_wrist_value

            if self.map_wrist_to_follower_start:
                if self.initial_leader_wrist_roll is None:
                    self.initial_leader_wrist_roll = continuous_wrist_value
                follower_anchor = (
                    self.initial_follower_wrist_roll
                    if self.initial_follower_wrist_roll is not None
                    else self.last_targets.get(WRIST_ROLL_KEY, continuous_wrist_value)
                )
                continuous_wrist_value = follower_anchor + (
                    continuous_wrist_value - self.initial_leader_wrist_roll
                )
            else:
                wrist_reference = self.last_targets.get(WRIST_ROLL_KEY)
                if wrist_reference is not None:
                    continuous_wrist_value = align_degrees_near_reference(
                        continuous_wrist_value,
                        wrist_reference,
                    )

            filtered[WRIST_ROLL_KEY] = continuous_wrist_value

        for key in ARM_STATE_KEYS:
            value = filtered.get(key)
            if not isinstance(value, (int, float)):
                continue

            next_value = float(value)
            absolute_limits = SAFE_ABSOLUTE_POSITION_LIMITS.get(key)
            if absolute_limits is not None:
                low, high = absolute_limits
                next_value = min(max(next_value, low), high)

            previous = self.last_targets.get(key)
            max_step = DEFAULT_SAFER_ARM_MAX_STEP.get(key)
            if key == WRIST_ROLL_KEY:
                filtered[key] = next_value
                continue

            if previous is not None and max_step is not None:
                next_value = min(max(next_value, previous - max_step), previous + max_step)

            filtered[key] = next_value

        return filtered


def add_torque_limit_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--torque-limits-json", default="{}")
    parser.add_argument("--torque-limits-path", default="")


def _coerce_torque_limit(value: Any) -> int | None:
    try:
        numeric = int(round(float(value)))
    except (TypeError, ValueError):
        return None
    return min(TORQUE_LIMIT_MAX, max(TORQUE_LIMIT_MIN, numeric))


def parse_torque_limits_json(value: str) -> dict[str, int]:
    text = value.strip()
    if not text:
        return {}

    parsed = json.loads(text)
    if not isinstance(parsed, dict):
        raise ValueError("Torque limits must be a JSON object.")

    limits: dict[str, int] = {}
    for motor in ARM_MOTORS:
        if motor not in parsed:
            continue
        limit = _coerce_torque_limit(parsed[motor])
        if limit is not None:
            limits[motor] = limit
    return limits


def apply_torque_limits(robot: Any, limits: dict[str, int]) -> None:
    if not limits:
        return

    applied: list[str] = []
    failed: list[str] = []
    available_motors = set(getattr(robot, "arm_motors", ARM_MOTORS))
    for motor, limit in limits.items():
        if motor not in available_motors:
            continue
        try:
            robot.bus.write("Torque_Limit", motor, int(limit))
            applied.append(f"{motor}={int(limit)}")
        except Exception as exc:
            failed.append(f"{motor}: {exc}")

    if applied:
        print(f"[torque] applied {', '.join(applied)}", flush=True)
    if failed:
        print(f"[torque] failed {', '.join(failed)}", flush=True)


class TorqueLimitFileWatcher:
    def __init__(self, path: str, min_interval_s: float = 0.4) -> None:
        self.path = Path(path).expanduser() if path else None
        self.min_interval_s = min_interval_s
        self.next_check_time = 0.0
        self.last_mtime_ns: int | None = None

    def poll(self, robot: Any, force: bool = False) -> None:
        if self.path is None:
            return

        now = time.time()
        if not force and now < self.next_check_time:
            return
        self.next_check_time = now + self.min_interval_s

        try:
            stat = self.path.stat()
        except FileNotFoundError:
            return

        if not force and self.last_mtime_ns == stat.st_mtime_ns:
            return

        self.last_mtime_ns = stat.st_mtime_ns
        try:
            limits = parse_torque_limits_json(self.path.read_text())
            apply_torque_limits(robot, limits)
        except Exception as exc:
            print(f"[torque] failed to load {self.path}: {exc}", flush=True)


def apply_robot_action(
    robot: Any,
    action: dict[str, Any],
    *,
    allow_legacy_base: bool,
) -> dict[str, float]:
    arm_goal_pos = {
        key: float(value)
        for key, value in action.items()
        if key.endswith(".pos") and isinstance(value, (int, float))
    }
    base_goal_vel = {
        key: float(action.get(key, 0.0)) if isinstance(action.get(key, 0.0), (int, float)) else 0.0
        for key in BASE_STATE_KEYS
    }

    if allow_legacy_base:
        return robot.send_action({**arm_goal_pos, **base_goal_vel})

    arm_goal_pos_raw = {key.removesuffix(".pos"): value for key, value in arm_goal_pos.items()}
    if arm_goal_pos_raw:
        robot.bus.sync_write("Goal_Position", arm_goal_pos_raw)
    return {**arm_goal_pos, **base_goal_vel}


def stop_robot_base(robot: Any, *, allow_legacy_base: bool) -> None:
    if allow_legacy_base:
        robot.stop_base()


def disconnect_robot(robot: Any, *, allow_legacy_base: bool) -> None:
    if allow_legacy_base:
        robot.disconnect()
        return

    try:
        robot.bus.disconnect(False)
    finally:
        for cam in getattr(robot, "cameras", {}).values():
            try:
                cam.disconnect()
            except Exception:
                pass
