#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import logging
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

try:
    import RPi.GPIO as GPIO
except Exception:  # pragma: no cover - only available on the Pi
    GPIO = None

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
ULTRASONIC_STATE_KEYS = (
    "ultrasonic_sensor_1.distance_m",
    "ultrasonic_sensor_2.distance_m",
)
ULTRASONIC_SENSOR_CONFIGS = (
    {
        "key": "ultrasonic_sensor_1.distance_m",
        "trigger_gpio": 17,
        "echo_gpio": 27,
    },
    {
        "key": "ultrasonic_sensor_2.distance_m",
        "trigger_gpio": 22,
        "echo_gpio": 23,
    },
)
ULTRASONIC_MIN_DISTANCE_M = 0.02
ULTRASONIC_MAX_DISTANCE_M = 4.0
ULTRASONIC_TRIGGER_PULSE_S = 0.00001
ULTRASONIC_PRE_TRIGGER_SETTLE_S = 0.000002
ULTRASONIC_INTER_SENSOR_DELAY_S = 0.01
ULTRASONIC_ECHO_START_TIMEOUT_S = 0.012
ULTRASONIC_ECHO_END_TIMEOUT_S = 0.025
ULTRASONIC_POLL_INTERVAL_S = 0.12
ULTRASONIC_STATUS_STALE_AFTER_S = 1.0
SPEED_OF_SOUND_MPS = 343.0
SENSOR_STATUS_LOG_PREFIX = "[sensor-status]"
SENSOR_STATUS_EMIT_INTERVAL_S = 2.0
WRIST_ROLL_KEY = "arm_wrist_roll.pos"
GRIPPER_MOTOR = "arm_gripper"
GRIPPER_KEY = "arm_gripper.pos"
WRIST_ROLL_MOTOR = "arm_wrist_roll"
WRIST_ROLL_SINGLE_TURN_LIMITS = (0, 4095)
WRIST_ROLL_CONTINUOUS_LIMITS = (0, 0)
DEFAULT_GRIPPER_FORCE_CURRENT_LIMIT_MA = 700.0
DEFAULT_GRIPPER_FORCE_HOLD_MARGIN = 0.8
DEFAULT_GRIPPER_FORCE_RELEASE_MARGIN = 2.0
DEFAULT_GRIPPER_FORCE_CLOSE_DIRECTION = -1.0
DEFAULT_GRIPPER_FORCE_MIN_CLOSING_ERROR = 1.0

DEFAULT_SAFER_ARM_MAX_STEP = {
    "arm_shoulder_pan.pos": 8.0,
    "arm_shoulder_lift.pos": 8.0,
    "arm_elbow_flex.pos": 4.0,
    "arm_wrist_flex.pos": 10.0,
    "arm_wrist_roll.pos": 12.0,
    "arm_gripper.pos": 2.0,
}
DEFAULT_SAFER_MAX_RELATIVE_TARGET = {
    "arm_shoulder_pan.pos": 10.0,
    "arm_shoulder_lift.pos": 10.0,
    "arm_elbow_flex.pos": 5.0,
    "arm_wrist_flex.pos": 12.0,
    "arm_wrist_roll.pos": 720.0,
    "arm_gripper.pos": 3.0,
}
SAFE_ABSOLUTE_POSITION_LIMITS = {
    GRIPPER_KEY: (0.0, 85.0),
}
RECOMMENDED_ARM_TORQUE_LIMITS = {
    "arm_shoulder_pan": 810,
    "arm_shoulder_lift": 870,
    "arm_elbow_flex": 790,
    "arm_wrist_flex": 820,
    "arm_wrist_roll": 850,
    "arm_gripper": 1000,
}
SERVO_STALL_ERROR_THRESHOLD = {
    "arm_shoulder_pan": 6.0,
    "arm_shoulder_lift": 6.0,
    "arm_elbow_flex": 6.0,
    "arm_wrist_flex": 5.0,
    "arm_wrist_roll": 8.0,
    "arm_gripper": 3.0,
}
SERVO_STALL_PROGRESS_EPSILON = {
    "arm_shoulder_pan": 0.35,
    "arm_shoulder_lift": 0.35,
    "arm_elbow_flex": 0.35,
    "arm_wrist_flex": 0.25,
    "arm_wrist_roll": 0.5,
    "arm_gripper": 0.15,
}
SERVO_STALL_TIMEOUT_S = {
    "arm_shoulder_pan": 0.55,
    "arm_shoulder_lift": 0.55,
    "arm_elbow_flex": 0.45,
    "arm_wrist_flex": 0.45,
    "arm_wrist_roll": 0.45,
    "arm_gripper": 0.18,
}
SERVO_STALL_CURRENT_LIMIT_MA = {
    "arm_shoulder_pan": 600.0,
    "arm_shoulder_lift": 650.0,
    "arm_elbow_flex": 600.0,
    "arm_wrist_flex": 450.0,
    "arm_wrist_roll": 350.0,
}
SERVO_HARD_TEMPERATURE_LIMIT_C = 50.0
SERVO_OVERTEMP_CONFIRMATION_SAMPLES = 2
SERVO_OVERTEMP_CONFIRMATION_S = 0.4
ARM_SAFETY_LOG_PREFIX = "[safety]"
ACTION_COMMAND_SOURCE_KEY = "__command_source__"
COMMAND_SOURCE_KEYBOARD = "keyboard"
VEX_CONTROL_MODE_KEY = "__vex_control_mode__"


def wrap_degrees(value: float) -> float:
    return ((float(value) + 180.0) % 360.0) - 180.0


def align_degrees_near_reference(value: float, reference: float) -> float:
    return float(reference) + wrap_degrees(float(value) - float(reference))


def iso_timestamp_from_epoch(epoch_s: float | None) -> str | None:
    if epoch_s is None or epoch_s <= 0:
        return None
    return datetime.fromtimestamp(epoch_s, tz=timezone.utc).isoformat(timespec="seconds")


def normalize_arm_action(action: dict[str, float], *, wrap_wrist_roll: bool = True) -> dict[str, float]:
    normalized = dict(action)
    if wrap_wrist_roll and WRIST_ROLL_KEY in normalized:
        normalized[WRIST_ROLL_KEY] = wrap_degrees(normalized[WRIST_ROLL_KEY])
    return normalized


def build_safer_max_relative_target() -> dict[str, float]:
    return dict(DEFAULT_SAFER_MAX_RELATIVE_TARGET)


def sanitize_position_limits(
    limits: dict[str, tuple[float, float]] | None,
) -> dict[str, tuple[float, float]]:
    sanitized: dict[str, tuple[float, float]] = {}
    if not limits:
        return sanitized

    for key, raw_limits in limits.items():
        if not isinstance(raw_limits, (list, tuple)) or len(raw_limits) != 2:
            continue
        try:
            low = float(raw_limits[0])
            high = float(raw_limits[1])
        except (TypeError, ValueError):
            continue
        sanitized[str(key)] = (min(low, high), max(low, high))
    return sanitized


def merge_position_limits(
    *limit_sets: dict[str, tuple[float, float]] | None,
) -> dict[str, tuple[float, float]]:
    merged: dict[str, tuple[float, float]] = {}
    for limits in limit_sets:
        for key, (low, high) in sanitize_position_limits(limits).items():
            existing = merged.get(key)
            if existing is None:
                merged[key] = (low, high)
                continue

            merged_low = max(existing[0], low)
            merged_high = min(existing[1], high)
            if merged_low <= merged_high:
                merged[key] = (merged_low, merged_high)
    return merged


def parse_position_limits_json(value: str) -> dict[str, tuple[float, float]]:
    text = value.strip()
    if not text:
        return {}

    parsed = json.loads(text)
    if not isinstance(parsed, dict):
        raise ValueError("Position limits must be a JSON object.")

    return sanitize_position_limits(parsed)


def build_normalized_arm_position_limits(
    robot: Any,
    *,
    preserve_continuous_wrist_roll: bool = False,
) -> dict[str, tuple[float, float]]:
    limits = merge_position_limits(SAFE_ABSOLUTE_POSITION_LIMITS)
    bus = getattr(robot, "bus", None)
    calibration = getattr(bus, "calibration", None)
    motors = getattr(bus, "motors", None)
    model_resolution_table = getattr(bus, "model_resolution_table", None)
    arm_motors = list(getattr(robot, "arm_motors", ARM_MOTORS))
    if not isinstance(calibration, dict) or not calibration:
        return limits
    if not isinstance(motors, dict) or not motors:
        return limits
    if not isinstance(model_resolution_table, dict):
        return limits

    calibration_limits: dict[str, tuple[float, float]] = {}
    for motor in arm_motors:
        if preserve_continuous_wrist_roll and motor == WRIST_ROLL_MOTOR:
            continue

        calibration_entry = calibration.get(motor)
        motor_config = motors.get(motor)
        if calibration_entry is None or motor_config is None:
            continue

        try:
            range_min = float(calibration_entry.range_min)
            range_max = float(calibration_entry.range_max)
        except (AttributeError, TypeError, ValueError):
            continue
        if range_max <= range_min:
            continue

        norm_mode = str(
            getattr(getattr(motor_config, "norm_mode", None), "value", getattr(motor_config, "norm_mode", ""))
        )
        action_key = f"{motor}.pos"
        if norm_mode == "degrees":
            model_name = getattr(motor_config, "model", None)
            max_res = float(model_resolution_table.get(model_name, 0)) - 1.0
            if max_res <= 0:
                continue
            midpoint = (range_min + range_max) / 2.0
            calibration_limits[action_key] = (
                ((range_min - midpoint) * 360.0) / max_res,
                ((range_max - midpoint) * 360.0) / max_res,
            )
        elif norm_mode == "range_0_100":
            calibration_limits[action_key] = (0.0, 100.0)
        elif norm_mode == "range_m100_100":
            calibration_limits[action_key] = (-100.0, 100.0)

    return merge_position_limits(calibration_limits, limits)


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
    parser.add_argument(
        "--disable-gripper-force-limit",
        action="store_true",
        help="Disable the follower gripper's soft force-limit hold behavior.",
    )
    parser.add_argument(
        "--gripper-force-current-limit-ma",
        type=float,
        default=DEFAULT_GRIPPER_FORCE_CURRENT_LIMIT_MA,
        help="Estimated gripper current threshold that activates soft hold while closing.",
    )
    parser.add_argument(
        "--gripper-force-hold-margin",
        type=float,
        default=DEFAULT_GRIPPER_FORCE_HOLD_MARGIN,
        help="Small extra closing offset added to the observed gripper position when soft hold activates.",
    )
    parser.add_argument(
        "--gripper-force-release-margin",
        type=float,
        default=DEFAULT_GRIPPER_FORCE_RELEASE_MARGIN,
        help="Leader/opening motion required before the follower gripper leaves soft hold.",
    )
    parser.add_argument(
        "--gripper-force-close-direction",
        type=float,
        choices=(-1.0, 1.0),
        default=DEFAULT_GRIPPER_FORCE_CLOSE_DIRECTION,
        help="Follower gripper closing direction in normalized position units.",
    )


def servo_protection_kwargs_from_args(args: argparse.Namespace) -> dict[str, Any]:
    return {
        "gripper_force_limit_enabled": not bool(getattr(args, "disable_gripper_force_limit", False)),
        "gripper_current_limit_ma": float(
            getattr(args, "gripper_force_current_limit_ma", DEFAULT_GRIPPER_FORCE_CURRENT_LIMIT_MA)
        ),
        "gripper_hold_margin": float(
            getattr(args, "gripper_force_hold_margin", DEFAULT_GRIPPER_FORCE_HOLD_MARGIN)
        ),
        "gripper_release_margin": float(
            getattr(args, "gripper_force_release_margin", DEFAULT_GRIPPER_FORCE_RELEASE_MARGIN)
        ),
        "gripper_close_direction": float(
            getattr(args, "gripper_force_close_direction", DEFAULT_GRIPPER_FORCE_CLOSE_DIRECTION)
        ),
    }


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
            **dict.fromkeys(ULTRASONIC_STATE_KEYS, 0.0),
        }
        self.cached_frames: dict[str, Any] = {}
        self.next_warning_at: dict[str, float] = {}
        self.ultrasonic_reader = UltrasonicArrayReader(logger, warning_interval_s=self.warning_interval_s)

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
        observation.update(self.ultrasonic_reader.read())

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

    def get_sensor_status_snapshot(self) -> dict[str, dict[str, Any]]:
        return self.ultrasonic_reader.status_snapshot()


class UltrasonicArrayReader:
    def __init__(
        self,
        logger: logging.Logger,
        *,
        warning_interval_s: float = 5.0,
    ) -> None:
        self.logger = logger
        self.warning_interval_s = max(float(warning_interval_s), 0.0)
        self.cached_values = dict.fromkeys(ULTRASONIC_STATE_KEYS, 0.0)
        self.last_measurement_at: dict[str, float | None] = dict.fromkeys(ULTRASONIC_STATE_KEYS, None)
        self.last_poll_ok: dict[str, bool] = dict.fromkeys(ULTRASONIC_STATE_KEYS, False)
        self.sensor_messages = {
            sensor["key"]: "Waiting for the first valid echo."
            for sensor in ULTRASONIC_SENSOR_CONFIGS
        }
        self.available = False
        self.next_poll_at = 0.0
        self.next_warning_at = 0.0
        self.unavailable_reason: str | None = None

        if GPIO is None:
            self.unavailable_reason = "RPi.GPIO is not installed on the Pi."
            for key in ULTRASONIC_STATE_KEYS:
                self.sensor_messages[key] = self.unavailable_reason
            self._warn("Ultrasonic sensors unavailable: RPi.GPIO is not installed.")
            return

        try:
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)
            for sensor in ULTRASONIC_SENSOR_CONFIGS:
                GPIO.setup(sensor["trigger_gpio"], GPIO.OUT)
                GPIO.setup(sensor["echo_gpio"], GPIO.IN)
                GPIO.output(sensor["trigger_gpio"], False)
            time.sleep(0.05)
            self.available = True
        except Exception as exc:
            reason = f"Ultrasonic sensor setup failed: {exc}"
            if "Cannot determine SOC peripheral base address" in str(exc):
                reason = (
                    "Ultrasonic sensor setup failed: incompatible RPi.GPIO backend for this Pi. "
                    "Install rpi-lgpio in the active Python environment."
                )
            self.unavailable_reason = reason
            for key in ULTRASONIC_STATE_KEYS:
                self.sensor_messages[key] = self.unavailable_reason
            self._warn("%s", self.unavailable_reason)

    def _warn(self, message: str, *args: Any) -> None:
        now = time.time()
        if now < self.next_warning_at:
            return
        self.next_warning_at = now + self.warning_interval_s
        self.logger.warning(message, *args)

    def read(self) -> dict[str, float]:
        if not self.available:
            return dict(self.cached_values)

        now = time.time()
        if now < self.next_poll_at:
            return dict(self.cached_values)

        for sensor in ULTRASONIC_SENSOR_CONFIGS:
            measured = self._measure_sensor(
                sensor["key"],
                sensor["trigger_gpio"],
                sensor["echo_gpio"],
            )
            if measured is not None:
                self.cached_values[sensor["key"]] = measured
                self.last_measurement_at[sensor["key"]] = time.time()
                self.last_poll_ok[sensor["key"]] = True
                self.sensor_messages[sensor["key"]] = (
                    f"Distance streaming on GPIO {sensor['trigger_gpio']}/{sensor['echo_gpio']}."
                )
            else:
                self.last_poll_ok[sensor["key"]] = False
            time.sleep(ULTRASONIC_INTER_SENSOR_DELAY_S)

        self.next_poll_at = time.time() + ULTRASONIC_POLL_INTERVAL_S
        return dict(self.cached_values)

    def status_snapshot(self) -> dict[str, dict[str, Any]]:
        now = time.time()
        snapshot: dict[str, dict[str, Any]] = {}
        for sensor in ULTRASONIC_SENSOR_CONFIGS:
            key = sensor["key"]
            last_measurement_at = self.last_measurement_at.get(key)
            age_s = now - last_measurement_at if last_measurement_at is not None else None
            value = self.cached_values.get(key, 0.0)
            fresh = bool(self.last_poll_ok.get(key)) and (
                age_s is None or age_s <= ULTRASONIC_STATUS_STALE_AFTER_S
            )

            state = "missing"
            connected = False
            message = self.sensor_messages.get(key, "Ultrasonic sensor unavailable.")
            if self.available:
                if last_measurement_at is None:
                    state = "waiting"
                    message = self.sensor_messages.get(key, "Waiting for the first valid echo.")
                elif not self.last_poll_ok.get(key):
                    state = "stale"
                    connected = True
                    if age_s is not None:
                        message = (
                            f"{self.sensor_messages.get(key, 'Last echo timed out.')} "
                            f"Last valid echo {age_s:.1f}s ago."
                        )
                elif age_s is not None and age_s > ULTRASONIC_STATUS_STALE_AFTER_S:
                    state = "stale"
                    connected = True
                    message = f"{self.sensor_messages.get(key, 'Last echo timed out.')} Last valid echo {age_s:.1f}s ago."
                else:
                    state = "online"
                    connected = True
                    message = self.sensor_messages.get(key, "Distance streaming.")

            snapshot[key] = {
                "state": state,
                "connected": connected,
                "value": round(float(value), 4) if last_measurement_at is not None else None,
                "unit": "m",
                "updated_at": iso_timestamp_from_epoch(last_measurement_at),
                "message": message,
                "fresh": fresh,
            }

        return snapshot

    def _measure_sensor(self, sensor_key: str, trigger_gpio: int, echo_gpio: int) -> float | None:
        if GPIO is None:
            return None

        try:
            GPIO.output(trigger_gpio, False)
            time.sleep(ULTRASONIC_PRE_TRIGGER_SETTLE_S)
            GPIO.output(trigger_gpio, True)
            time.sleep(ULTRASONIC_TRIGGER_PULSE_S)
            GPIO.output(trigger_gpio, False)

            wait_deadline = time.perf_counter() + ULTRASONIC_ECHO_START_TIMEOUT_S
            pulse_start = None
            while time.perf_counter() < wait_deadline:
                if GPIO.input(echo_gpio):
                    pulse_start = time.perf_counter()
                    break
            if pulse_start is None:
                self.sensor_messages[sensor_key] = (
                    f"No echo detected on GPIO {trigger_gpio}/{echo_gpio}; holding the last value."
                )
                return None

            pulse_end = pulse_start
            pulse_deadline = pulse_start + ULTRASONIC_ECHO_END_TIMEOUT_S
            while time.perf_counter() < pulse_deadline:
                if not GPIO.input(echo_gpio):
                    pulse_end = time.perf_counter()
                    break
            else:
                self.sensor_messages[sensor_key] = (
                    f"Echo pulse timed out on GPIO {trigger_gpio}/{echo_gpio}; holding the last value."
                )
                return None

            distance_m = ((pulse_end - pulse_start) * SPEED_OF_SOUND_MPS) / 2.0
            if not (ULTRASONIC_MIN_DISTANCE_M <= distance_m <= ULTRASONIC_MAX_DISTANCE_M):
                self.sensor_messages[sensor_key] = (
                    f"Echo on GPIO {trigger_gpio}/{echo_gpio} was out of range; holding the last value."
                )
                return None
            return round(distance_m, 4)
        except Exception as exc:
            self.sensor_messages[sensor_key] = (
                f"Read failed on GPIO {trigger_gpio}/{echo_gpio}: {exc}"
            )
            self._warn("Ultrasonic sensor read failed on GPIO %s/%s: %s", trigger_gpio, echo_gpio, exc)
            return None


def build_live_robot_sensor_status_snapshot(
    observation_reader: ResilientObservationReader,
    observation: dict[str, Any],
    *,
    source: str,
    vex_base_bridge: Any | None = None,
) -> dict[str, Any]:
    gyro_status = {
        "state": "waiting",
        "connected": False,
        "value": None,
        "unit": "deg",
        "updated_at": None,
        "message": "Waiting for VEX inertial telemetry.",
        "source": None,
    }
    if vex_base_bridge is not None and hasattr(vex_base_bridge, "gyro_status_snapshot"):
        candidate = vex_base_bridge.gyro_status_snapshot()
        if isinstance(candidate, dict):
            gyro_status.update(candidate)
    else:
        rotation = observation.get("vex_inertial_rotation.deg")
        if isinstance(rotation, (int, float)):
            gyro_status.update(
                {
                    "state": "online",
                    "connected": True,
                    "value": round(float(rotation), 3),
                    "updated_at": iso_timestamp_from_epoch(time.time()),
                    "message": "Gyro rotation is streaming from the VEX Brain.",
                    "source": source,
                }
            )

    ultrasonic_status = observation_reader.get_sensor_status_snapshot()
    sensor_1_status = dict(ultrasonic_status.get(ULTRASONIC_STATE_KEYS[0], {}))
    sensor_2_status = dict(ultrasonic_status.get(ULTRASONIC_STATE_KEYS[1], {}))
    sensor_1_status["source"] = source
    sensor_2_status["source"] = source

    return {
        "timestamp": iso_timestamp_from_epoch(time.time()),
        "source": source,
        "gyro": gyro_status,
        "x": sensor_1_status,
        "y": sensor_2_status,
    }


class LiveRobotSensorStatusEmitter:
    def __init__(self, *, emit_interval_s: float = SENSOR_STATUS_EMIT_INTERVAL_S) -> None:
        self.emit_interval_s = max(float(emit_interval_s), 0.0)
        self.next_emit_at = 0.0

    def emit(
        self,
        observation_reader: ResilientObservationReader,
        observation: dict[str, Any],
        *,
        source: str,
        vex_base_bridge: Any | None = None,
        force: bool = False,
    ) -> None:
        now = time.time()
        if not force and now < self.next_emit_at:
            return

        payload = build_live_robot_sensor_status_snapshot(
            observation_reader,
            observation,
            source=source,
            vex_base_bridge=vex_base_bridge,
        )
        print(
            f"{SENSOR_STATUS_LOG_PREFIX} {json.dumps(payload, sort_keys=True, separators=(',', ':'))}",
            flush=True,
        )
        self.next_emit_at = now + self.emit_interval_s


class ArmSafetyFilter:
    def __init__(
        self,
        enabled: bool,
        *,
        map_wrist_to_follower_start: bool = False,
        absolute_position_limits: dict[str, tuple[float, float]] | None = None,
        skip_step_limit_sources: tuple[str, ...] | list[str] | set[str] = (),
    ) -> None:
        self.enabled = bool(enabled)
        self.map_wrist_to_follower_start = bool(map_wrist_to_follower_start)
        self.absolute_position_limits = merge_position_limits(
            SAFE_ABSOLUTE_POSITION_LIMITS,
            absolute_position_limits,
        )
        self.skip_step_limit_sources = {
            str(source).strip().lower()
            for source in skip_step_limit_sources
            if str(source).strip()
        }
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
        filtered = dict(normalized)
        command_source = str(filtered.get(ACTION_COMMAND_SOURCE_KEY, "")).strip().lower()
        skip_step_limit = command_source in self.skip_step_limit_sources
        filtered.pop(ACTION_COMMAND_SOURCE_KEY, None)
        wrist_value = filtered.get(WRIST_ROLL_KEY)
        if self.enabled and isinstance(wrist_value, (int, float)):
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
            absolute_limits = self.absolute_position_limits.get(key)
            if absolute_limits is not None:
                low, high = absolute_limits
                next_value = min(max(next_value, low), high)

            if not self.enabled:
                filtered[key] = next_value
                continue

            previous = self.last_targets.get(key)
            max_step = DEFAULT_SAFER_ARM_MAX_STEP.get(key)
            if key == WRIST_ROLL_KEY:
                filtered[key] = next_value
                continue

            if not skip_step_limit and previous is not None and max_step is not None:
                next_value = min(max(next_value, previous - max_step), previous + max_step)

            filtered[key] = next_value

        return filtered


def clamp_safe_arm_torque_limits(limits: dict[str, int]) -> tuple[dict[str, int], list[str]]:
    if not limits:
        return {}, []

    clamped: dict[str, int] = {}
    capped_messages: list[str] = []
    for motor, limit in limits.items():
        numeric = int(limit)
        safe_max = int(RECOMMENDED_ARM_TORQUE_LIMITS.get(motor, TORQUE_LIMIT_MAX))
        capped = min(numeric, safe_max)
        clamped[motor] = capped
        if capped != numeric:
            capped_messages.append(f"{motor}={numeric}->{capped}")
    return clamped, capped_messages


@dataclass
class ServoProtectionState:
    last_observed: float | None = None
    last_command: float | None = None
    candidate_since: float | None = None


@dataclass
class TemperatureProtectionState:
    candidate_since: float | None = None
    samples: int = 0
    last_sample_id: str | None = None
    peak_temperature_c: float | None = None

    def reset(self) -> None:
        self.candidate_since = None
        self.samples = 0
        self.last_sample_id = None
        self.peak_temperature_c = None


@dataclass
class GripperForceLimitState:
    active: bool = False
    hold_position: float | None = None
    hold_target: float | None = None
    close_direction: float = DEFAULT_GRIPPER_FORCE_CLOSE_DIRECTION
    activated_at: float | None = None
    reason: str = ""
    current_ma: float | None = None

    def reset(self) -> None:
        self.active = False
        self.hold_position = None
        self.hold_target = None
        self.close_direction = DEFAULT_GRIPPER_FORCE_CLOSE_DIRECTION
        self.activated_at = None
        self.reason = ""
        self.current_ma = None


class ServoProtectionSupervisor:
    def __init__(
        self,
        robot: Any,
        logger: logging.Logger,
        *,
        enabled: bool = True,
        stall_detection_enabled: bool = True,
        hard_temperature_c: float = SERVO_HARD_TEMPERATURE_LIMIT_C,
        gripper_force_limit_enabled: bool = True,
        gripper_current_limit_ma: float = DEFAULT_GRIPPER_FORCE_CURRENT_LIMIT_MA,
        gripper_hold_margin: float = DEFAULT_GRIPPER_FORCE_HOLD_MARGIN,
        gripper_release_margin: float = DEFAULT_GRIPPER_FORCE_RELEASE_MARGIN,
        gripper_close_direction: float = DEFAULT_GRIPPER_FORCE_CLOSE_DIRECTION,
    ) -> None:
        self.robot = robot
        self.logger = logger
        self.enabled = bool(enabled)
        self.stall_detection_enabled = bool(stall_detection_enabled)
        self.hard_temperature_c = max(float(hard_temperature_c), 0.0)
        self.gripper_force_limit_enabled = bool(gripper_force_limit_enabled)
        self.gripper_current_limit_ma = max(float(gripper_current_limit_ma), 0.0)
        self.gripper_hold_margin = max(float(gripper_hold_margin), 0.0)
        self.gripper_release_margin = max(float(gripper_release_margin), 0.0)
        self.gripper_close_direction = 1.0 if float(gripper_close_direction) >= 0.0 else -1.0
        self.arm_motors = list(getattr(robot, "arm_motors", ARM_MOTORS))
        all_robot_motors = getattr(getattr(robot, "bus", None), "motors", None)
        if isinstance(all_robot_motors, dict) and all_robot_motors:
            self.all_motors = list(all_robot_motors.keys())
        else:
            self.all_motors = list(self.arm_motors)
        self.states = {motor: ServoProtectionState() for motor in self.arm_motors}
        self.temperature_states = {
            motor: TemperatureProtectionState()
            for motor in self.all_motors
        }
        self.gripper_force_limit = GripperForceLimitState(
            close_direction=self.gripper_close_direction,
        )
        self.fault: dict[str, Any] | None = None

    @property
    def latched(self) -> bool:
        return self.fault is not None

    def seed_from_observation(self, observation: dict[str, Any]) -> None:
        if not self.stall_detection_enabled:
            return

        for motor, state in self.states.items():
            value = observation.get(f"{motor}.pos")
            if not isinstance(value, (int, float)):
                continue
            position = float(value)
            state.last_observed = position
            state.last_command = position
            state.candidate_since = None

    def record_command(self, action: dict[str, Any]) -> None:
        if not self.enabled or not self.stall_detection_enabled or self.fault is not None:
            return

        for motor, state in self.states.items():
            value = action.get(f"{motor}.pos")
            if not isinstance(value, (int, float)):
                continue
            state.last_command = float(value)

    @staticmethod
    def _sample_float(sample: dict[str, Any] | None, key: str) -> float | None:
        if not sample:
            return None
        value = sample.get(key)
        if value in ("", None):
            return None
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _error_threshold(motor: str) -> float:
        return float(SERVO_STALL_ERROR_THRESHOLD.get(motor, 6.0))

    @staticmethod
    def _progress_epsilon(motor: str) -> float:
        return float(SERVO_STALL_PROGRESS_EPSILON.get(motor, 0.35))

    @staticmethod
    def _stall_timeout_s(motor: str) -> float:
        return float(SERVO_STALL_TIMEOUT_S.get(motor, 0.5))

    @staticmethod
    def _stall_current_limit_ma(motor: str) -> float | None:
        value = SERVO_STALL_CURRENT_LIMIT_MA.get(motor)
        if value is None:
            return None
        return float(value)

    def _stall_has_load_evidence(
        self,
        motor: str,
        power_sample: dict[str, Any] | None,
    ) -> tuple[bool, float | None, float | None]:
        limit_ma = (
            self.gripper_current_limit_ma
            if motor == GRIPPER_MOTOR
            else self._stall_current_limit_ma(motor)
        )
        if limit_ma is None or limit_ma <= 0.0:
            return True, None, limit_ma

        current_ma = self._sample_float(power_sample, f"{motor}.current_ma_est")
        if current_ma is None:
            return True, None, limit_ma

        return current_ma >= limit_ma, current_ma, limit_ma

    def _clamp_gripper_target(self, value: float) -> float:
        low, high = SAFE_ABSOLUTE_POSITION_LIMITS.get(GRIPPER_KEY, (0.0, 100.0))
        return min(max(float(value), low), high)

    def _is_gripper_closing(self, observed: float, command: float) -> bool:
        error_in_close_direction = (float(command) - float(observed)) * self.gripper_close_direction
        return error_in_close_direction > DEFAULT_GRIPPER_FORCE_MIN_CLOSING_ERROR

    def _command_gripper_hold(self, hold_target: float) -> None:
        try:
            self.robot.bus.sync_write("Goal_Position", {GRIPPER_MOTOR: float(hold_target)})
        except Exception as exc:
            self.logger.warning("Failed to command gripper force-limit hold: %s", exc)

    def _activate_gripper_force_limit(
        self,
        *,
        observed: float,
        command: float,
        reason: str,
        current_ma: float | None,
    ) -> bool:
        if (
            not self.gripper_force_limit_enabled
            or GRIPPER_MOTOR not in self.states
            or not self._is_gripper_closing(observed, command)
        ):
            return False

        close_direction = self.gripper_close_direction
        hold_position = float(observed)
        hold_target = self._clamp_gripper_target(hold_position + close_direction * self.gripper_hold_margin)
        already_active = self.gripper_force_limit.active
        self.gripper_force_limit.active = True
        self.gripper_force_limit.hold_position = hold_position
        self.gripper_force_limit.hold_target = hold_target
        self.gripper_force_limit.close_direction = close_direction
        self.gripper_force_limit.activated_at = time.monotonic()
        self.gripper_force_limit.reason = reason
        self.gripper_force_limit.current_ma = current_ma

        state = self.states[GRIPPER_MOTOR]
        state.last_command = hold_target
        state.candidate_since = None
        self._command_gripper_hold(hold_target)

        if not already_active:
            current_text = f", current {current_ma:.0f}mA" if current_ma is not None else ""
            print(
                f"{ARM_SAFETY_LOG_PREFIX} gripper force limit active; "
                f"holding {GRIPPER_KEY} at {hold_target:.2f} instead of {command:.2f} "
                f"({reason}{current_text})",
                flush=True,
            )
        return True

    def _clear_gripper_force_limit(self, command: float | None = None) -> None:
        if not self.gripper_force_limit.active:
            return

        command_text = f" command={command:.2f}" if command is not None else ""
        print(
            f"{ARM_SAFETY_LOG_PREFIX} gripper force limit released; opening command accepted{command_text}",
            flush=True,
        )
        self.gripper_force_limit.reset()
        self.gripper_force_limit.close_direction = self.gripper_close_direction
        state = self.states.get(GRIPPER_MOTOR)
        if state is not None:
            state.candidate_since = None

    def limit_action(self, action: dict[str, Any]) -> dict[str, Any]:
        if (
            not self.enabled
            or self.fault is not None
            or not self.gripper_force_limit.active
            or GRIPPER_KEY not in action
        ):
            return action

        command = action.get(GRIPPER_KEY)
        if not isinstance(command, (int, float)):
            return action

        hold_target = self.gripper_force_limit.hold_target
        close_direction = self.gripper_force_limit.close_direction
        if hold_target is None:
            self._clear_gripper_force_limit(float(command))
            return action

        numeric_command = float(command)
        opening_delta = (numeric_command - hold_target) * close_direction
        if opening_delta <= -self.gripper_release_margin:
            self._clear_gripper_force_limit(numeric_command)
            return action

        limited = dict(action)
        limited[GRIPPER_KEY] = hold_target
        return limited

    @staticmethod
    def _power_sample_id(power_sample: dict[str, Any] | None, now: float) -> str:
        if isinstance(power_sample, dict):
            sample_time = power_sample.get("monotonic_s")
            if sample_time not in ("", None):
                return f"sample:{sample_time}"
        return f"observe:{now:.6f}"

    def _overtemp_confirmed(
        self,
        motor: str,
        temperature_c: float,
        *,
        now: float,
        sample_id: str,
    ) -> bool:
        state = self.temperature_states.setdefault(motor, TemperatureProtectionState())
        if state.candidate_since is None:
            state.candidate_since = now
            state.samples = 0
            state.last_sample_id = None
            state.peak_temperature_c = temperature_c

        if state.last_sample_id != sample_id:
            state.samples += 1
            state.last_sample_id = sample_id

        if state.peak_temperature_c is None or temperature_c > state.peak_temperature_c:
            state.peak_temperature_c = temperature_c

        duration_s = now - state.candidate_since
        return (
            state.samples >= SERVO_OVERTEMP_CONFIRMATION_SAMPLES
            and duration_s >= SERVO_OVERTEMP_CONFIRMATION_S
        )

    def _trip(
        self,
        motor: str,
        reason: str,
        observation: dict[str, Any],
        power_sample: dict[str, Any] | None,
    ) -> None:
        disabled_motors = list(self.all_motors)
        disable_error: str | None = None
        try:
            self.robot.bus.disable_torque(disabled_motors, num_retry=10)
        except Exception as exc:
            disable_error = str(exc)
            self.logger.warning("Failed to disable full robot torque after safety fault: %s", exc)

        self.fault = {
            "timestamp": iso_timestamp_from_epoch(time.time()),
            "motor": motor,
            "motors": disabled_motors,
            "reason": reason,
            "temperature_c": self._sample_float(power_sample, f"{motor}.temperature_c"),
            "position_deg": observation.get(f"{motor}.pos"),
            "disable_error": disable_error,
        }
        disabled_text = ", ".join(disabled_motors)
        message = f"{ARM_SAFETY_LOG_PREFIX} latched {reason}; disabled all motor torque on {disabled_text}"
        if disable_error:
            message += f" (disable failed: {disable_error})"
        print(message, flush=True)

    def observe(self, observation: dict[str, Any], power_sample: dict[str, Any] | None = None) -> bool:
        if not self.enabled:
            return False
        if self.fault is not None:
            return True

        now = time.monotonic()
        sample_id = self._power_sample_id(power_sample, now)
        for motor in self.all_motors:
            temperature_c = self._sample_float(power_sample, f"{motor}.temperature_c")
            if temperature_c is None or temperature_c < self.hard_temperature_c:
                self.temperature_states.setdefault(motor, TemperatureProtectionState()).reset()
                continue
            if not self._overtemp_confirmed(
                motor,
                temperature_c,
                now=now,
                sample_id=sample_id,
            ):
                continue
            peak_temperature_c = (
                self.temperature_states
                .setdefault(motor, TemperatureProtectionState())
                .peak_temperature_c
                or temperature_c
            )
            self._trip(
                motor,
                (
                    f"{motor} temperature reached {peak_temperature_c:.1f}C "
                    f"(hard limit {self.hard_temperature_c:.1f}C)"
                ),
                observation,
                power_sample,
            )
            return True

        if not self.stall_detection_enabled:
            return False

        for motor, state in self.states.items():
            value = observation.get(f"{motor}.pos")
            if not isinstance(value, (int, float)):
                continue

            observed = float(value)
            previous_observed = state.last_observed
            progress = abs(observed - previous_observed) if previous_observed is not None else None
            state.last_observed = observed

            command = state.last_command
            if command is None:
                state.candidate_since = None
                continue

            gripper_current_ma = None
            if motor == GRIPPER_MOTOR:
                gripper_current_ma = self._sample_float(power_sample, f"{GRIPPER_MOTOR}.current_ma_est")
                if (
                    gripper_current_ma is not None
                    and self.gripper_current_limit_ma > 0.0
                    and gripper_current_ma >= self.gripper_current_limit_ma
                    and self._activate_gripper_force_limit(
                        observed=observed,
                        command=command,
                        reason=(
                            f"{GRIPPER_MOTOR} current reached {gripper_current_ma:.0f}mA "
                            f"(soft limit {self.gripper_current_limit_ma:.0f}mA)"
                        ),
                        current_ma=gripper_current_ma,
                    )
                ):
                    continue

            error = abs(command - observed)
            if error <= self._error_threshold(motor):
                state.candidate_since = None
                continue

            if progress is not None and progress >= self._progress_epsilon(motor):
                state.candidate_since = now
                continue

            if state.candidate_since is None:
                state.candidate_since = now
                continue

            duration_s = now - state.candidate_since
            if duration_s >= self._stall_timeout_s(motor):
                has_load_evidence, current_ma, current_limit_ma = self._stall_has_load_evidence(
                    motor,
                    power_sample,
                )
                if not has_load_evidence:
                    state.candidate_since = now
                    continue

                if motor == GRIPPER_MOTOR:
                    if current_ma is None:
                        state.candidate_since = now
                        continue
                    if self._activate_gripper_force_limit(
                        observed=observed,
                        command=command,
                        reason=(
                            f"{GRIPPER_MOTOR} current reached {current_ma:.0f}mA "
                            f"(soft limit {current_limit_ma:.0f}mA) after {duration_s:.2f}s stall"
                        ),
                        current_ma=current_ma,
                    ):
                        continue

                current_text = ""
                if current_ma is not None and current_limit_ma is not None:
                    current_text = f", current {current_ma:.0f}mA >= {current_limit_ma:.0f}mA"
                elif current_limit_ma is not None:
                    current_text = ", no current sample available"
                self._trip(
                    motor,
                    (
                        f"{motor} stalled with {error:.2f}deg position error and no meaningful motion "
                        f"for {duration_s:.2f}s{current_text}"
                    ),
                    observation,
                    power_sample,
                )
                return True

        return False

    def enrich_observation(self, observation: dict[str, Any]) -> dict[str, Any]:
        if self.fault is None and not self.gripper_force_limit.active:
            return observation

        enriched = dict(observation)
        if self.gripper_force_limit.active:
            enriched["telemetry.gripper_force_limit_active"] = True
            enriched["telemetry.gripper_force_limit_hold_target"] = self.gripper_force_limit.hold_target
            enriched["telemetry.gripper_force_limit_reason"] = self.gripper_force_limit.reason
            if self.gripper_force_limit.current_ma is not None:
                enriched["telemetry.gripper_force_limit_current_ma"] = round(
                    self.gripper_force_limit.current_ma,
                    3,
                )
        if self.fault is not None:
            enriched["telemetry.safety_fault_active"] = True
            enriched["telemetry.safety_fault_motor"] = self.fault.get("motor", "")
            enriched["telemetry.safety_fault_reason"] = self.fault.get("reason", "")
            enriched["telemetry.safety_fault_at"] = self.fault.get("timestamp", "")
        return enriched


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

    limits, capped = clamp_safe_arm_torque_limits(limits)
    if capped:
        print(f"[torque] capped {', '.join(capped)}", flush=True)

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
