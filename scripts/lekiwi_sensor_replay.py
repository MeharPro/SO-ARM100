#!/usr/bin/env python3

from __future__ import annotations

from collections import deque
import json
from statistics import median
import time
from dataclasses import dataclass, field
from typing import Any

from lekiwi_runtime import align_degrees_near_reference
from vex_base_bridge import BASE_STATE_KEYS, VEX_MOTOR_STATE_KEYS

ULTRASONIC_X_KEY = "ultrasonic_sensor_1.distance_m"
ULTRASONIC_Y_KEY = "ultrasonic_sensor_2.distance_m"
GYRO_ROTATION_KEY = "vex_inertial_rotation.deg"
VEX_POSE_EPOCH_KEY = "vex_pose_epoch"
SENSOR_REPLAY_LOG_PREFIX = "[sensor-replay]"
RECORDED_SENSOR_REFERENCE_KEYS = (
    ULTRASONIC_X_KEY,
    ULTRASONIC_Y_KEY,
    GYRO_ROTATION_KEY,
)

XY_CORRECTION_GAIN_MPS_PER_M = 1.5
XY_CORRECTION_MAX_LINEAR_RATIO = 0.55
XY_CORRECTION_MIN_LINEAR_MPS = 0.045
THETA_CORRECTION_GAIN_DPS_PER_DEG = 1.8
THETA_CORRECTION_MAX_TURN_RATIO = 0.40
THETA_CORRECTION_MIN_TURN_DPS = 10.0
XY_TRACK_TOLERANCE_M = 0.02
XY_RECENTER_THRESHOLD_M = 0.05
THETA_TRACK_TOLERANCE_DEG = 1.5
THETA_RECENTER_THRESHOLD_DEG = 5.0
ULTRASONIC_MIN_VALID_M = 0.03
ULTRASONIC_MAX_VALID_M = 3.5

PREPOSITION_TIMEOUT_S = 6.0
PREPOSITION_TOLERANCE_DEG = 6.0
PREPOSITION_RESEND_INTERVAL_S = 0.12
PREPOSITION_COMMAND_DT_MS = 1000
PREPOSITION_COMMAND_TTL_MS = 100
PREPOSITION_HEADING_COMMAND_TTL_MS = 60
SENSOR_PREPOSITION_TIMEOUT_S = 6.5
SENSOR_PREPOSITION_SETTLE_S = 0.35
SENSOR_PREPOSITION_RESEND_INTERVAL_S = 0.10
SENSOR_PREPOSITION_HEADING_SETTLE_S = 0.18
SENSOR_PREPOSITION_LINEAR_SETTLE_S = 0.14
PREPOSITION_LINEAR_MEDIUM_ERROR_M = 0.055
PREPOSITION_LINEAR_LARGE_ERROR_M = 0.10
PREPOSITION_HEADING_MEDIUM_ERROR_DEG = 5.0
PREPOSITION_HEADING_LARGE_ERROR_DEG = 12.0
PREPOSITION_HEADING_GAIN_DPS_PER_DEG = 0.9
PREPOSITION_HEADING_MIN_TURN_DPS = 5.0
PREPOSITION_HEADING_MAX_TURN_DPS = 12.0
PREPOSITION_HEADING_SMALL_PULSE_TTL_MS = 80
PREPOSITION_HEADING_MEDIUM_PULSE_TTL_MS = 100
PREPOSITION_HEADING_LARGE_PULSE_TTL_MS = 120
PREPOSITION_HEADING_CROSS_TRIM_DEG = 8.5
PREPOSITION_HEADING_WIDE_CROSS_CONFIRMATIONS = 2
PREPOSITION_SMALL_PULSE_TTL_MS = 120
PREPOSITION_MEDIUM_PULSE_TTL_MS = 220
PREPOSITION_LARGE_PULSE_TTL_MS = 300
PREPOSITION_ALIGNED_CONFIRMATIONS = 3
PREPOSITION_STAGE_ORDER = ("heading", "x", "y")
PREPOSITION_MAX_AXIS_COMMANDS_WITHOUT_PROGRESS = 6
PREPOSITION_MAX_TOTAL_MOTION_COMMANDS = 40
PREPOSITION_MIN_PROGRESS_M = 0.002
PREPOSITION_MIN_PROGRESS_DEG = 0.25
PREPOSITION_SENSOR_PROGRESS_EPSILON_M = 0.0006
PREPOSITION_SENSOR_PROGRESS_EPSILON_DEG = 0.12
PREPOSITION_ERROR_INCREASE_M = 0.025
PREPOSITION_ERROR_INCREASE_DEG = 1.5
PREPOSITION_ERROR_INCREASE_CONFIRMATIONS = 2
PREPOSITION_CROSS_ACCEPT_MULTIPLIER = 2.0
PREPOSITION_MAX_CLOSE_CROSSES = 3
PREPOSITION_LINEAR_CROSS_ACCEPT_M = XY_TRACK_TOLERANCE_M
PREPOSITION_LINEAR_CROSS_TRIM_M = 0.05
PREPOSITION_LINEAR_CROSS_CONFIRMATIONS = 2
PREPOSITION_MAX_WHEEL_PERCENT = 18.0
ULTRASONIC_FILTER_WINDOW = 5
ULTRASONIC_FILTER_MIN_SAMPLES = 3
ULTRASONIC_FILTER_MAX_STEP_M = 0.03
ULTRASONIC_FILTER_CHANGE_CONFIRMATIONS = 2
RECORDED_HEADING_ZERO_TOLERANCE_DEG = 2.0

WHEEL_CORRECTION_SIGNS = {
    "vex_front_right.pos": (-1.0, -1.0),
    "vex_front_left.pos": (1.0, 1.0),
    "vex_rear_right.pos": (1.0, -1.0),
    "vex_rear_left.pos": (-1.0, 1.0),
}


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, float(value)))


def _apply_tolerance(error: float | None, tolerance: float) -> float | None:
    if error is None:
        return None
    if abs(float(error)) <= float(tolerance):
        return 0.0
    return float(error)


def _state_numeric(state: dict[str, Any], key: str) -> float | None:
    value = state.get(key)
    if isinstance(value, (int, float)):
        return float(value)
    return None


def _extract_motion(state: dict[str, Any], speed: float) -> dict[str, float]:
    speed_scale = max(float(speed), 0.0)
    return {
        "x.vel": float(state.get("x.vel", 0.0) or 0.0) * speed_scale,
        "y.vel": float(state.get("y.vel", 0.0) or 0.0) * speed_scale,
        "theta.vel": float(state.get("theta.vel", 0.0) or 0.0) * speed_scale,
    }


def _extract_targets(state: dict[str, Any]) -> dict[str, float] | None:
    targets = {
        key: float(state[key])
        for key in VEX_MOTOR_STATE_KEYS
        if isinstance(state.get(key), (int, float))
    }
    if len(targets) != len(VEX_MOTOR_STATE_KEYS):
        return None
    return targets


def _status_value(status: dict[str, Any] | None) -> float | None:
    if not isinstance(status, dict):
        return None
    if status.get("state") != "online":
        return None
    value = status.get("value")
    if isinstance(value, (int, float)):
        return float(value)
    return None


def _valid_ultrasonic_value(status: dict[str, Any] | None) -> float | None:
    value = _status_value(status)
    if value is None:
        return None
    if ULTRASONIC_MIN_VALID_M <= value <= ULTRASONIC_MAX_VALID_M:
        return value
    return None


def _axis_cross_accept_tolerance(axis: str) -> float:
    if axis == "heading":
        return THETA_TRACK_TOLERANCE_DEG * PREPOSITION_CROSS_ACCEPT_MULTIPLIER
    return XY_TRACK_TOLERANCE_M * PREPOSITION_CROSS_ACCEPT_MULTIPLIER


def _axis_progress_epsilon(axis: str) -> float:
    if axis == "heading":
        return PREPOSITION_SENSOR_PROGRESS_EPSILON_DEG
    return PREPOSITION_SENSOR_PROGRESS_EPSILON_M


def _bounded_signed_correction(raw_value: float, *, limit: float, floor: float) -> float:
    if abs(raw_value) <= 1e-9:
        return 0.0
    correction_limit = max(float(limit), 0.0)
    if correction_limit <= 0.0:
        return 0.0
    correction_floor = min(max(float(floor), 0.0), correction_limit)
    magnitude = min(abs(float(raw_value)), correction_limit)
    magnitude = max(magnitude, correction_floor)
    return magnitude if raw_value > 0.0 else -magnitude


def _preposition_pulse_ttl_ms(axis: str, abs_error: float, *, axis_motion_count: int = 0) -> int:
    if axis == "heading":
        if abs_error >= PREPOSITION_HEADING_LARGE_ERROR_DEG:
            return PREPOSITION_HEADING_LARGE_PULSE_TTL_MS
        if abs_error >= PREPOSITION_HEADING_MEDIUM_ERROR_DEG:
            return PREPOSITION_HEADING_MEDIUM_PULSE_TTL_MS
        return PREPOSITION_HEADING_SMALL_PULSE_TTL_MS

    if abs_error >= PREPOSITION_LINEAR_LARGE_ERROR_M:
        return PREPOSITION_LARGE_PULSE_TTL_MS
    if abs_error >= PREPOSITION_LINEAR_MEDIUM_ERROR_M:
        return PREPOSITION_MEDIUM_PULSE_TTL_MS
    return PREPOSITION_SMALL_PULSE_TTL_MS


def _preposition_heading_turn_dps(
    raw_error_deg: float,
    *,
    max_turn_speed_dps: float,
) -> float:
    """Use short, low-speed nudges for start heading alignment."""
    turn_limit = min(
        PREPOSITION_HEADING_MAX_TURN_DPS,
        max(float(max_turn_speed_dps), PREPOSITION_HEADING_MIN_TURN_DPS),
    )
    return _bounded_signed_correction(
        float(raw_error_deg) * PREPOSITION_HEADING_GAIN_DPS_PER_DEG,
        limit=turn_limit,
        floor=PREPOSITION_HEADING_MIN_TURN_DPS,
    )


def _drive_wheel_percentages(
    motion: dict[str, float],
    *,
    max_linear_speed_mps: float,
    max_turn_speed_dps: float,
) -> dict[str, float]:
    max_linear = max(float(max_linear_speed_mps), 0.001)
    max_turn = max(float(max_turn_speed_dps), 0.001)
    forward_pct = clamp((float(motion.get("x.vel", 0.0) or 0.0) / max_linear) * 100.0, -100.0, 100.0)
    strafe_pct = clamp((float(motion.get("y.vel", 0.0) or 0.0) / max_linear) * 100.0, -100.0, 100.0)
    turn_pct = clamp((float(motion.get("theta.vel", 0.0) or 0.0) / max_turn) * 100.0, -100.0, 100.0)
    return {
        "front_right": round(forward_pct - turn_pct - strafe_pct, 2),
        "front_left": round(forward_pct + turn_pct + strafe_pct, 2),
        "rear_right": round(forward_pct - turn_pct + strafe_pct, 2),
        "rear_left": round(forward_pct + turn_pct - strafe_pct, 2),
    }


def _limit_motion_by_wheel_percent(
    motion: dict[str, float],
    *,
    max_linear_speed_mps: float,
    max_turn_speed_dps: float,
    max_abs_wheel_percent: float,
) -> dict[str, float]:
    wheel_pct = _drive_wheel_percentages(
        motion,
        max_linear_speed_mps=max_linear_speed_mps,
        max_turn_speed_dps=max_turn_speed_dps,
    )
    max_seen = max((abs(value) for value in wheel_pct.values()), default=0.0)
    limit = max(float(max_abs_wheel_percent), 0.0)
    if limit <= 0.0 or max_seen <= limit:
        return motion
    scale = limit / max_seen
    return {key: float(value) * scale for key, value in motion.items()}


def _linear_raw_error(feedback: dict[str, Any], axis: str) -> float | None:
    key = "raw_x_error_m" if axis == "x" else "raw_y_error_m"
    value = feedback.get(key)
    return float(value) if isinstance(value, (int, float)) else None


def _linear_decision_error(feedback: dict[str, Any], axis: str) -> float | None:
    key = "decision_x_error_m" if axis == "x" else "decision_y_error_m"
    value = feedback.get(key)
    return float(value) if isinstance(value, (int, float)) else None


def _linear_effective_error(feedback: dict[str, Any], axis: str) -> float | None:
    decision_error = _linear_decision_error(feedback, axis)
    raw_error = _linear_raw_error(feedback, axis)
    if decision_error is None:
        return raw_error
    if raw_error is None:
        return decision_error
    if abs(raw_error) <= XY_TRACK_TOLERANCE_M:
        return raw_error
    if decision_error * raw_error < 0:
        return raw_error
    if abs(raw_error) < abs(decision_error):
        return raw_error
    return decision_error


def _feedback_final_errors(feedback: dict[str, Any] | None) -> dict[str, float | None] | None:
    if feedback is None:
        return None
    return {
        "x_m": round(float(feedback["decision_x_error_m"]), 4)
        if feedback.get("decision_x_error_m") is not None
        else None,
        "y_m": round(float(feedback["decision_y_error_m"]), 4)
        if feedback.get("decision_y_error_m") is not None
        else None,
        "heading_deg": round(float(feedback["raw_heading_error_deg"]), 3)
        if feedback.get("raw_heading_error_deg") is not None
        else None,
    }


def _preposition_result(
    aligned: bool,
    *,
    reason: str | None = None,
    axis: str | None = None,
    detail: str | None = None,
    feedback: dict[str, Any] | None = None,
) -> PrepositionResult:
    return PrepositionResult(
        aligned=aligned,
        reason=reason,
        axis=axis,
        detail=detail,
        final_errors=_feedback_final_errors(feedback),
    )


def _status_numeric(status: dict[str, Any] | None, key: str) -> float | None:
    if not isinstance(status, dict):
        return None
    value = status.get(key)
    if isinstance(value, (int, float)):
        return float(value)
    return None


def _marked_pose_epoch(value: Any) -> bool:
    if not isinstance(value, (int, float)):
        return False
    try:
        return int(float(value)) > 0
    except (TypeError, ValueError):
        return False


def _recorded_heading_is_zero_reference(value: Any) -> bool:
    if not isinstance(value, (int, float)):
        return False
    aligned = align_degrees_near_reference(float(value), 0.0)
    return abs(aligned) <= RECORDED_HEADING_ZERO_TOLERANCE_DEG


def _heading_reference_block_reason(feedback: dict[str, Any]) -> str | None:
    if feedback.get("recorded_heading_deg") is None:
        return None
    if feedback.get("live_heading_deg") is None:
        return "live-gyro-unavailable"
    live_pose_epoch = feedback.get("live_pose_epoch")
    if live_pose_epoch is None:
        return "live-pose-epoch-missing"
    if not _marked_pose_epoch(live_pose_epoch):
        return "live-pose-origin-unzeroed"

    recorded_pose_epoch = feedback.get("recorded_pose_epoch")
    if recorded_pose_epoch is None:
        if _recorded_heading_is_zero_reference(feedback.get("recorded_heading_deg")):
            return None
        return "recorded-pose-epoch-missing"
    if not _marked_pose_epoch(recorded_pose_epoch) and not _recorded_heading_is_zero_reference(
        feedback.get("recorded_heading_deg")
    ):
        return "recorded-pose-origin-unzeroed"
    return None


def _heading_reference_block_detail(reason: str) -> str:
    if reason in {"live-pose-epoch-missing", "live-pose-origin-unzeroed"}:
        return "The live VEX gyro has not been zeroed for this replay; press Zero VEX Gyro at the recorded start heading, then replay after moving the base."
    if reason in {"recorded-pose-epoch-missing", "recorded-pose-origin-unzeroed"}:
        return "The recording has a nonzero gyro heading, but it was not tied to a marked VEX gyro zero origin."
    if reason == "live-gyro-unavailable":
        return "Live VEX gyro telemetry is unavailable."
    return "Recorded gyro heading exists, but the live VEX gyro origin is not ready for replay."


def _first_recorded_value(samples: list[dict[str, Any]], key: str) -> float | None:
    for sample in samples:
        state = sample.get("state") if isinstance(sample, dict) else None
        if isinstance(state, dict):
            value = _state_numeric(state, key)
            if value is not None:
                return value
    return None


def recorded_state_has_sensor_reference(state: dict[str, Any]) -> bool:
    return any(_state_numeric(state, key) is not None for key in RECORDED_SENSOR_REFERENCE_KEYS)


@dataclass
class SensorReplayCommand:
    mode: str
    motion: dict[str, float]
    ecu_targets: dict[str, float] | None
    command_dt_ms: int
    correction: dict[str, Any]


@dataclass
class PrepositionResult:
    aligned: bool
    reason: str | None = None
    axis: str | None = None
    detail: str | None = None
    final_errors: dict[str, float | None] | None = None

    def __bool__(self) -> bool:
        return self.aligned


@dataclass
class WheelCorrectionModel:
    linear_deg_per_m: float
    turn_deg_per_deg: float

    def offsets_for_correction(self, correction_motion: dict[str, float], dt_s: float) -> dict[str, float]:
        dt = max(float(dt_s), 0.0)
        x_vel = float(correction_motion.get("x.vel", 0.0) or 0.0)
        y_vel = float(correction_motion.get("y.vel", 0.0) or 0.0)
        theta_vel = float(correction_motion.get("theta.vel", 0.0) or 0.0)
        offsets: dict[str, float] = {}
        for key in VEX_MOTOR_STATE_KEYS:
            sign_y, sign_theta = WHEEL_CORRECTION_SIGNS[key]
            wheel_deg_s = (
                self.linear_deg_per_m * (x_vel + (sign_y * y_vel))
                + self.turn_deg_per_deg * (sign_theta * theta_vel)
            )
            offsets[key] = wheel_deg_s * dt
        return offsets


@dataclass
class UltrasonicStabilityFilter:
    window_size: int = ULTRASONIC_FILTER_WINDOW
    min_samples: int = ULTRASONIC_FILTER_MIN_SAMPLES
    max_step_m: float = ULTRASONIC_FILTER_MAX_STEP_M
    change_confirmations: int = ULTRASONIC_FILTER_CHANGE_CONFIRMATIONS
    recent_values_m: deque[float] = field(default_factory=lambda: deque(maxlen=ULTRASONIC_FILTER_WINDOW))
    accepted_value_m: float | None = None
    pending_value_m: float | None = None
    pending_count: int = 0

    def reset(self) -> None:
        self.recent_values_m.clear()
        self.accepted_value_m = None
        self.pending_value_m = None
        self.pending_count = 0

    def observe(self, value_m: float | None) -> float | None:
        if value_m is None:
            self.pending_value_m = None
            self.pending_count = 0
            return None

        self.recent_values_m.append(float(value_m))
        if not self.recent_values_m:
            return None

        candidate_m = float(median(self.recent_values_m))
        if self.accepted_value_m is None:
            self.accepted_value_m = candidate_m
            self.pending_value_m = None
            self.pending_count = 0
            return self.accepted_value_m

        delta_m = candidate_m - self.accepted_value_m
        if abs(delta_m) <= self.max_step_m:
            self.accepted_value_m = candidate_m
        else:
            direction = 1.0 if delta_m > 0 else -1.0
            self.accepted_value_m += direction * self.max_step_m

        self.pending_value_m = None
        self.pending_count = 0
        return self.accepted_value_m


def fit_wheel_correction_model(samples: list[dict[str, Any]]) -> WheelCorrectionModel | None:
    sum_aa = 0.0
    sum_ab = 0.0
    sum_bb = 0.0
    sum_ay = 0.0
    sum_by = 0.0
    row_count = 0
    previous_state: dict[str, Any] | None = None
    previous_t_s: float | None = None

    for sample in samples:
        if not isinstance(sample, dict):
            continue
        state = sample.get("state")
        raw_t_s = sample.get("t_s")
        if not isinstance(state, dict) or not isinstance(raw_t_s, (int, float)):
            continue
        if previous_state is None or previous_t_s is None:
            previous_state = state
            previous_t_s = float(raw_t_s)
            continue

        dt_s = float(raw_t_s) - previous_t_s
        previous_targets = _extract_targets(previous_state)
        targets = _extract_targets(state)
        if dt_s <= 0 or previous_targets is None or targets is None:
            previous_state = state
            previous_t_s = float(raw_t_s)
            continue

        motion = _extract_motion(state, 1.0)
        for key in VEX_MOTOR_STATE_KEYS:
            sign_y, sign_theta = WHEEL_CORRECTION_SIGNS[key]
            a = motion["x.vel"] + (sign_y * motion["y.vel"])
            b = sign_theta * motion["theta.vel"]
            y = (targets[key] - previous_targets[key]) / dt_s
            sum_aa += a * a
            sum_ab += a * b
            sum_bb += b * b
            sum_ay += a * y
            sum_by += b * y
            row_count += 1

        previous_state = state
        previous_t_s = float(raw_t_s)

    determinant = (sum_aa * sum_bb) - (sum_ab * sum_ab)
    if row_count < 4 or abs(determinant) < 1e-9:
        return None

    linear_deg_per_m = ((sum_ay * sum_bb) - (sum_by * sum_ab)) / determinant
    turn_deg_per_deg = ((sum_by * sum_aa) - (sum_ay * sum_ab)) / determinant
    return WheelCorrectionModel(
        linear_deg_per_m=float(linear_deg_per_m),
        turn_deg_per_deg=float(turn_deg_per_deg),
    )


class SensorAwareReplayState:
    def __init__(
        self,
        samples: list[dict[str, Any]],
        *,
        replay_mode: str,
        speed: float,
        control_config: dict[str, Any] | None,
    ) -> None:
        self.samples = samples
        self.requested_mode = "ecu" if replay_mode == "ecu" else "drive"
        self.speed = max(float(speed), 0.01)
        tuning = control_config.get("tuning") if isinstance(control_config, dict) else {}
        self.max_linear_speed_mps = max(float(tuning.get("maxLinearSpeedMps", 0.35) or 0.35), 0.05)
        self.max_turn_speed_dps = max(float(tuning.get("maxTurnSpeedDps", 90.0) or 90.0), 5.0)
        self.max_linear_correction_mps = self.max_linear_speed_mps * XY_CORRECTION_MAX_LINEAR_RATIO
        self.max_turn_correction_dps = self.max_turn_speed_dps * THETA_CORRECTION_MAX_TURN_RATIO
        self.wheel_model = fit_wheel_correction_model(samples)
        self.ecu_offsets_deg = {key: 0.0 for key in VEX_MOTOR_STATE_KEYS}
        self.first_targets = self._find_first_targets()
        self.ultrasonic_filters = {
            "x": UltrasonicStabilityFilter(),
            "y": UltrasonicStabilityFilter(),
        }

    @property
    def effective_mode(self) -> str:
        if self.requested_mode == "ecu" and self.first_targets is not None:
            return "ecu"
        return "drive"

    def _find_first_targets(self) -> dict[str, float] | None:
        for sample in self.samples:
            state = sample.get("state") if isinstance(sample, dict) else None
            if isinstance(state, dict):
                targets = _extract_targets(state)
                if targets is not None:
                    return targets
        return None

    def prepare(self) -> None:
        self.ecu_offsets_deg = {key: 0.0 for key in VEX_MOTOR_STATE_KEYS}
        for axis_filter in self.ultrasonic_filters.values():
            axis_filter.reset()

    def _build_feedback_context(
        self,
        state: dict[str, Any],
        *,
        live_sensor_status: dict[str, dict[str, Any]],
        live_gyro_status: dict[str, Any],
    ) -> dict[str, Any]:
        recorded_x_m = _state_numeric(state, ULTRASONIC_X_KEY)
        recorded_y_m = _state_numeric(state, ULTRASONIC_Y_KEY)
        recorded_heading_deg = _state_numeric(state, GYRO_ROTATION_KEY)
        recorded_pose_epoch = _state_numeric(state, VEX_POSE_EPOCH_KEY)

        live_x_raw_m = _valid_ultrasonic_value(live_sensor_status.get(ULTRASONIC_X_KEY))
        live_y_raw_m = _valid_ultrasonic_value(live_sensor_status.get(ULTRASONIC_Y_KEY))
        live_x_m = self.ultrasonic_filters["x"].observe(live_x_raw_m)
        live_y_m = self.ultrasonic_filters["y"].observe(live_y_raw_m)
        live_heading_deg = _status_value(live_gyro_status)
        live_pose_epoch = _status_numeric(live_gyro_status, "pose_epoch")
        heading_reference_block_reason = _heading_reference_block_reason(
            {
                "recorded_heading_deg": recorded_heading_deg,
                "recorded_pose_epoch": recorded_pose_epoch,
                "live_heading_deg": live_heading_deg,
                "live_pose_epoch": live_pose_epoch,
            }
        )
        heading_origin_valid = recorded_heading_deg is not None and heading_reference_block_reason is None

        raw_x_error_m = (
            live_x_raw_m - recorded_x_m
            if live_x_raw_m is not None and recorded_x_m is not None
            else None
        )
        raw_y_error_m = (
            live_y_raw_m - recorded_y_m
            if live_y_raw_m is not None and recorded_y_m is not None
            else None
        )
        decision_x_error_m = (
            live_x_m - recorded_x_m
            if live_x_m is not None and recorded_x_m is not None
            else None
        )
        decision_y_error_m = (
            live_y_m - recorded_y_m
            if live_y_m is not None and recorded_y_m is not None
            else None
        )

        raw_heading_error_deg: float | None = None
        if heading_origin_valid and live_heading_deg is not None:
            aligned_target_deg = align_degrees_near_reference(recorded_heading_deg, live_heading_deg)
            raw_heading_error_deg = aligned_target_deg - live_heading_deg

        x_error_m = _apply_tolerance(decision_x_error_m, XY_TRACK_TOLERANCE_M)
        y_error_m = _apply_tolerance(decision_y_error_m, XY_TRACK_TOLERANCE_M)
        heading_error_deg = _apply_tolerance(raw_heading_error_deg, THETA_TRACK_TOLERANCE_DEG)

        available_axes: list[str] = []
        aligned_axes: list[str] = []
        feedback_only = False
        if decision_x_error_m is not None:
            available_axes.append("x")
            if abs(decision_x_error_m) <= XY_TRACK_TOLERANCE_M:
                aligned_axes.append("x")
            if abs(decision_x_error_m) >= XY_RECENTER_THRESHOLD_M:
                feedback_only = True
        if decision_y_error_m is not None:
            available_axes.append("y")
            if abs(decision_y_error_m) <= XY_TRACK_TOLERANCE_M:
                aligned_axes.append("y")
            if abs(decision_y_error_m) >= XY_RECENTER_THRESHOLD_M:
                feedback_only = True
        if raw_heading_error_deg is not None:
            available_axes.append("heading")
            if abs(raw_heading_error_deg) <= THETA_TRACK_TOLERANCE_DEG:
                aligned_axes.append("heading")
            if abs(raw_heading_error_deg) >= THETA_RECENTER_THRESHOLD_DEG:
                feedback_only = True

        return {
            "recorded_x_m": recorded_x_m,
            "recorded_y_m": recorded_y_m,
            "recorded_heading_deg": recorded_heading_deg,
            "recorded_pose_epoch": recorded_pose_epoch,
            "live_x_raw_m": live_x_raw_m,
            "live_y_raw_m": live_y_raw_m,
            "live_x_m": live_x_m,
            "live_y_m": live_y_m,
            "live_heading_deg": live_heading_deg,
            "live_pose_epoch": live_pose_epoch,
            "heading_origin_valid": heading_origin_valid,
            "raw_x_error_m": raw_x_error_m,
            "raw_y_error_m": raw_y_error_m,
            "decision_x_error_m": decision_x_error_m,
            "decision_y_error_m": decision_y_error_m,
            "raw_heading_error_deg": raw_heading_error_deg,
            "x_error_m": x_error_m,
            "y_error_m": y_error_m,
            "heading_error_deg": heading_error_deg,
            "available_axes": available_axes,
            "aligned_axes": aligned_axes,
            "aligned": bool(available_axes) and heading_reference_block_reason is None and len(aligned_axes) == len(available_axes),
            "feedback_only": feedback_only,
            "heading_reference_block_reason": heading_reference_block_reason,
        }

    def feedback_context(
        self,
        state: dict[str, Any],
        *,
        live_sensor_status: dict[str, dict[str, Any]],
        live_gyro_status: dict[str, Any],
    ) -> dict[str, Any]:
        return self._build_feedback_context(
            state,
            live_sensor_status=live_sensor_status,
            live_gyro_status=live_gyro_status,
        )

    def has_feedback(
        self,
        state: dict[str, Any],
        *,
        live_sensor_status: dict[str, dict[str, Any]],
        live_gyro_status: dict[str, Any],
    ) -> bool:
        return bool(
            self._build_feedback_context(
                state,
                live_sensor_status=live_sensor_status,
                live_gyro_status=live_gyro_status,
            )["available_axes"]
        )

    def aligned_to_recorded_state(
        self,
        state: dict[str, Any],
        *,
        live_sensor_status: dict[str, dict[str, Any]],
        live_gyro_status: dict[str, Any],
    ) -> bool:
        return bool(
            self._build_feedback_context(
                state,
                live_sensor_status=live_sensor_status,
                live_gyro_status=live_gyro_status,
            )["aligned"]
        )

    def build_command(
        self,
        state: dict[str, Any],
        *,
        command_dt_s: float,
        live_sensor_status: dict[str, dict[str, Any]],
        live_gyro_status: dict[str, Any],
        force_feedback_only: bool = False,
        active_axes: set[str] | None = None,
        feedback: dict[str, Any] | None = None,
    ) -> SensorReplayCommand:
        motion = _extract_motion(state, self.speed)
        correction = {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}
        if feedback is None:
            feedback = self._build_feedback_context(
                state,
                live_sensor_status=live_sensor_status,
                live_gyro_status=live_gyro_status,
            )
        axis_filter = set(active_axes or ())

        x_error_m = feedback["x_error_m"]
        y_error_m = feedback["y_error_m"]
        heading_error_deg = feedback["heading_error_deg"]
        if x_error_m is not None and (not axis_filter or "x" in axis_filter):
            correction["x.vel"] = _bounded_signed_correction(
                x_error_m * XY_CORRECTION_GAIN_MPS_PER_M,
                limit=self.max_linear_correction_mps,
                floor=XY_CORRECTION_MIN_LINEAR_MPS,
            )

        if y_error_m is not None and (not axis_filter or "y" in axis_filter):
            correction["y.vel"] = _bounded_signed_correction(
                y_error_m * XY_CORRECTION_GAIN_MPS_PER_M,
                limit=self.max_linear_correction_mps,
                floor=XY_CORRECTION_MIN_LINEAR_MPS,
            )

        if heading_error_deg is not None and (not axis_filter or "heading" in axis_filter):
            correction["theta.vel"] = _bounded_signed_correction(
                heading_error_deg * THETA_CORRECTION_GAIN_DPS_PER_DEG,
                limit=self.max_turn_correction_dps,
                floor=THETA_CORRECTION_MIN_TURN_DPS,
            )

        feedback_only = bool(force_feedback_only or feedback["feedback_only"])
        motion_seed = dict.fromkeys(BASE_STATE_KEYS, 0.0) if feedback_only else motion
        corrected_motion = {
            "x.vel": clamp(
                motion_seed["x.vel"] + correction["x.vel"],
                -self.max_linear_speed_mps,
                self.max_linear_speed_mps,
            ),
            "y.vel": clamp(
                motion_seed["y.vel"] + correction["y.vel"],
                -self.max_linear_speed_mps,
                self.max_linear_speed_mps,
            ),
            "theta.vel": clamp(
                motion_seed["theta.vel"] + correction["theta.vel"],
                -self.max_turn_speed_dps,
                self.max_turn_speed_dps,
            ),
        }

        correction_meta: dict[str, Any] = {
            "recorded": {
                "x_m": feedback["recorded_x_m"],
                "y_m": feedback["recorded_y_m"],
                "heading_deg": feedback["recorded_heading_deg"],
                "pose_epoch": feedback["recorded_pose_epoch"],
            },
            "live": {
                "x_m": feedback["live_x_m"],
                "y_m": feedback["live_y_m"],
                "heading_deg": feedback["live_heading_deg"],
                "pose_epoch": feedback["live_pose_epoch"],
            },
            "live_raw": {
                "x_m": feedback["live_x_raw_m"],
                "y_m": feedback["live_y_raw_m"],
            },
            "applied": {
                "x.vel": round(correction["x.vel"], 4),
                "y.vel": round(correction["y.vel"], 4),
                "theta.vel": round(correction["theta.vel"], 4),
            },
            "errors": {
                "x_m": round(float(feedback["decision_x_error_m"]), 4)
                if feedback["decision_x_error_m"] is not None
                else None,
                "y_m": round(float(feedback["decision_y_error_m"]), 4)
                if feedback["decision_y_error_m"] is not None
                else None,
                "heading_deg": round(float(feedback["raw_heading_error_deg"]), 3)
                if feedback["raw_heading_error_deg"] is not None
                else None,
            },
            "errors_raw": {
                "x_m": round(float(feedback["raw_x_error_m"]), 4)
                if feedback["raw_x_error_m"] is not None
                else None,
                "y_m": round(float(feedback["raw_y_error_m"]), 4)
                if feedback["raw_y_error_m"] is not None
                else None,
            },
            "wheel_model": {
                "linear_deg_per_m": round(self.wheel_model.linear_deg_per_m, 4) if self.wheel_model is not None else None,
                "turn_deg_per_deg": round(self.wheel_model.turn_deg_per_deg, 4) if self.wheel_model is not None else None,
            },
            "aligned": feedback["aligned"],
            "aligned_axes": list(feedback["aligned_axes"]),
            "available_axes": list(feedback["available_axes"]),
            "feedback_only": feedback_only,
            "heading_origin_valid": feedback["heading_origin_valid"],
        }
        if axis_filter:
            correction_meta["active_axes"] = sorted(axis_filter)

        if self.effective_mode != "ecu":
            return SensorReplayCommand(
                mode="drive",
                motion=corrected_motion,
                ecu_targets=None,
                command_dt_ms=max(int(round(max(command_dt_s, 0.02) * 1000.0)), 20),
                correction=correction_meta,
            )

        recorded_targets = _extract_targets(state)
        if recorded_targets is None:
            return SensorReplayCommand(
                mode="drive",
                motion=corrected_motion,
                ecu_targets=None,
                command_dt_ms=max(int(round(max(command_dt_s, 0.02) * 1000.0)), 20),
                correction={**correction_meta, "fallback": "missing-recorded-wheel-targets"},
            )

        adjusted_targets = dict(recorded_targets)
        if self.wheel_model is not None:
            delta_offsets = self.wheel_model.offsets_for_correction(correction, command_dt_s)
            for key in VEX_MOTOR_STATE_KEYS:
                self.ecu_offsets_deg[key] += delta_offsets[key]
                adjusted_targets[key] += self.ecu_offsets_deg[key]
        else:
            correction_meta["fallback"] = "missing-wheel-model"

        correction_meta["ecu_offsets_deg"] = {
            key: round(self.ecu_offsets_deg[key], 3) for key in VEX_MOTOR_STATE_KEYS
        }
        return SensorReplayCommand(
            mode="ecu",
            motion=corrected_motion,
            ecu_targets=adjusted_targets,
            command_dt_ms=max(int(round(max(command_dt_s, 0.02) * 1000.0)), 20),
            correction=correction_meta,
        )

    def build_replay_command(
        self,
        state: dict[str, Any],
        *,
        command_dt_s: float,
    ) -> SensorReplayCommand:
        motion = _extract_motion(state, self.speed)
        correction_meta: dict[str, Any] = {
            "phase": "replay",
            "strategy": "start-alignment-only",
        }
        command_dt_ms = max(int(round(max(command_dt_s, 0.02) * 1000.0)), 20)

        if self.effective_mode != "ecu":
            return SensorReplayCommand(
                mode="drive",
                motion=motion,
                ecu_targets=None,
                command_dt_ms=command_dt_ms,
                correction=correction_meta,
            )

        recorded_targets = _extract_targets(state)
        if recorded_targets is None:
            return SensorReplayCommand(
                mode="drive",
                motion=motion,
                ecu_targets=None,
                command_dt_ms=command_dt_ms,
                correction={**correction_meta, "fallback": "missing-recorded-wheel-targets"},
            )

        return SensorReplayCommand(
            mode="ecu",
            motion=motion,
            ecu_targets=dict(recorded_targets),
            command_dt_ms=command_dt_ms,
            correction=correction_meta,
        )


def preposition_vex_base(
    vex_base_bridge: Any,
    targets: dict[str, float] | None,
    *,
    timeout_s: float = PREPOSITION_TIMEOUT_S,
    tolerance_deg: float = PREPOSITION_TOLERANCE_DEG,
) -> bool:
    if targets is None:
        return True

    deadline = time.time() + max(float(timeout_s), 0.0)
    zero_motion = dict.fromkeys(BASE_STATE_KEYS, 0.0)
    while time.time() <= deadline:
        vex_base_bridge.poll()
        state = vex_base_bridge.current_state()
        settled = all(
            abs(float(state.get(key, 0.0) or 0.0) - float(targets[key])) <= tolerance_deg
            for key in VEX_MOTOR_STATE_KEYS
        )
        if settled:
            return True

        vex_base_bridge.send_ecu_targets(
            targets,
            motion=zero_motion,
            command_dt_ms=PREPOSITION_COMMAND_DT_MS,
            ttl_ms=PREPOSITION_COMMAND_TTL_MS,
        )
        time.sleep(PREPOSITION_RESEND_INTERVAL_S)

    return False


def preposition_vex_base_to_recorded_state(
    vex_base_bridge: Any,
    observation_reader: Any,
    replay_state: SensorAwareReplayState,
    start_state: dict[str, Any],
    *,
    timeout_s: float = SENSOR_PREPOSITION_TIMEOUT_S,
    settle_s: float = SENSOR_PREPOSITION_SETTLE_S,
    should_stop: Any | None = None,
) -> PrepositionResult:
    observation = vex_base_bridge.merge_observation(observation_reader.get_observation())
    distance_status = observation_reader.get_sensor_status_snapshot()
    gyro_status = vex_base_bridge.gyro_status_snapshot()
    replay_state.prepare()
    feedback = replay_state.feedback_context(
        start_state,
        live_sensor_status=distance_status,
        live_gyro_status=gyro_status,
    )
    heading_block_reason = _heading_reference_block_reason(feedback)
    if heading_block_reason is not None:
        vex_base_bridge.send_hold(ttl_ms=PREPOSITION_COMMAND_TTL_MS)
        print(
            f"{SENSOR_REPLAY_LOG_PREFIX} "
            + json.dumps(
                {
                    "phase": "preposition",
                    "event": "stopped",
                    "reason": heading_block_reason,
                    "axis": "heading",
                    "recorded_pose_epoch": feedback.get("recorded_pose_epoch"),
                    "live_pose_epoch": feedback.get("live_pose_epoch"),
                },
                separators=(",", ":"),
            ),
            flush=True,
        )
        return _preposition_result(
            False,
            reason=heading_block_reason,
            axis="heading",
            detail=_heading_reference_block_detail(heading_block_reason),
            feedback=feedback,
        )
    if not feedback["available_axes"]:
        vex_base_bridge.send_hold(ttl_ms=PREPOSITION_COMMAND_TTL_MS)
        print(
            f"{SENSOR_REPLAY_LOG_PREFIX} "
            + json.dumps(
                {
                    "phase": "preposition",
                    "event": "skipped",
                    "reason": "no-live-feedback",
                },
                separators=(",", ":"),
            ),
            flush=True,
        )
        return _preposition_result(False, reason="no-live-feedback", detail="No live gyro or ultrasonic feedback was available.")

    timeout = max(float(timeout_s), 0.0)
    deadline = time.time() + timeout if timeout > 0 else None
    aligned_since: float | None = None
    aligned_confirmations = 0
    last_command_at: float | None = None
    progress_axis: str | None = None
    best_axis_error: float | None = None
    previous_axis_error: float | None = None
    commands_without_progress = 0
    total_motion_commands = 0
    accepted_axes: set[str] = set()
    close_cross_counts: dict[str, int] = {}
    error_increase_counts: dict[str, int] = {}
    axis_motion_counts: dict[str, int] = {}
    while deadline is None or time.time() <= deadline:
        if callable(should_stop) and should_stop():
            vex_base_bridge.send_hold(ttl_ms=PREPOSITION_COMMAND_TTL_MS)
            print(
                f"{SENSOR_REPLAY_LOG_PREFIX} "
                + json.dumps(
                    {
                        "phase": "preposition",
                        "event": "stopped",
                        "reason": "user-stop",
                    },
                    separators=(",", ":"),
                ),
                flush=True,
            )
            return _preposition_result(False, reason="user-stop", detail="Replay was stopped by the user.")

        vex_base_bridge.merge_observation(observation_reader.get_observation())
        distance_status = observation_reader.get_sensor_status_snapshot()
        gyro_status = vex_base_bridge.gyro_status_snapshot()
        feedback = replay_state.feedback_context(
            start_state,
            live_sensor_status=distance_status,
            live_gyro_status=gyro_status,
        )
        heading_block_reason = _heading_reference_block_reason(feedback)
        if heading_block_reason is not None:
            vex_base_bridge.send_hold(ttl_ms=PREPOSITION_COMMAND_TTL_MS)
            print(
                f"{SENSOR_REPLAY_LOG_PREFIX} "
                + json.dumps(
                    {
                        "phase": "preposition",
                        "event": "stopped",
                        "reason": heading_block_reason,
                        "axis": "heading",
                        "recorded_pose_epoch": feedback.get("recorded_pose_epoch"),
                        "live_pose_epoch": feedback.get("live_pose_epoch"),
                    },
                    separators=(",", ":"),
                ),
                flush=True,
            )
            return _preposition_result(
                False,
                reason=heading_block_reason,
                axis="heading",
                detail=_heading_reference_block_detail(heading_block_reason),
                feedback=feedback,
            )

        active_axis: str | None = None
        active_raw_error: float | None = None
        for axis in PREPOSITION_STAGE_ORDER:
            if axis not in feedback["available_axes"]:
                continue
            if axis == "heading":
                raw_error = feedback["raw_heading_error_deg"]
                tolerance = THETA_TRACK_TOLERANCE_DEG
            elif axis == "x":
                raw_error = _linear_effective_error(feedback, axis)
                tolerance = XY_TRACK_TOLERANCE_M
            else:
                raw_error = _linear_effective_error(feedback, axis)
                tolerance = XY_TRACK_TOLERANCE_M
            if raw_error is None:
                continue
            raw_error_float = float(raw_error)
            if axis in accepted_axes:
                if axis != "heading" or abs(raw_error_float) <= _axis_cross_accept_tolerance(axis):
                    continue
                accepted_axes.discard(axis)
            if abs(raw_error_float) > tolerance:
                active_axis = axis
                active_raw_error = raw_error_float
                break

        if active_axis is None:
            if aligned_since is None:
                aligned_since = time.time()
                aligned_confirmations = 0
            aligned_confirmations += 1
            vex_base_bridge.send_hold(ttl_ms=PREPOSITION_COMMAND_TTL_MS)
            if (
                aligned_confirmations >= PREPOSITION_ALIGNED_CONFIRMATIONS
                and (time.time() - aligned_since) >= settle_s
            ):
                print(
                    f"{SENSOR_REPLAY_LOG_PREFIX} "
                    + json.dumps(
                        {
                            "phase": "preposition",
                            "event": "aligned",
                            "final_errors": _feedback_final_errors(feedback),
                        },
                        separators=(",", ":"),
                    ),
                    flush=True,
                )
                return _preposition_result(True, feedback=feedback)
            continue
        else:
            aligned_since = None
            aligned_confirmations = 0
            abs_error = abs(float(active_raw_error or 0.0))
            progress_epsilon = _axis_progress_epsilon(active_axis)
            min_progress = (
                PREPOSITION_MIN_PROGRESS_DEG
                if active_axis == "heading"
                else PREPOSITION_MIN_PROGRESS_M
            )
            increase_limit = (
                PREPOSITION_ERROR_INCREASE_DEG
                if active_axis == "heading"
                else PREPOSITION_ERROR_INCREASE_M
            )
            if progress_axis != active_axis:
                progress_axis = active_axis
                best_axis_error = abs_error
                previous_axis_error = active_raw_error
                commands_without_progress = 0
                close_cross_counts[active_axis] = 0
                error_increase_counts[active_axis] = 0
                axis_motion_counts.setdefault(active_axis, 0)
                print(
                    f"{SENSOR_REPLAY_LOG_PREFIX} "
                    + json.dumps(
                        {
                            "phase": "preposition",
                            "event": "axis-start",
                            "axis": active_axis,
                            "current_error": round(abs_error, 4),
                        },
                        separators=(",", ":"),
                    ),
                    flush=True,
                )
            elif (
                previous_axis_error is not None
                and active_raw_error is not None
                and (previous_axis_error * active_raw_error) < 0
            ):
                crossed_from_error = previous_axis_error
                if active_axis != "heading":
                    close_cross_counts[active_axis] = close_cross_counts.get(active_axis, 0) + 1
                    vex_base_bridge.send_hold(ttl_ms=PREPOSITION_COMMAND_TTL_MS)
                    if abs(active_raw_error) <= PREPOSITION_LINEAR_CROSS_ACCEPT_M:
                        accepted_axes.add(active_axis)
                        best_axis_error = abs_error
                        previous_axis_error = active_raw_error
                        commands_without_progress = 0
                        error_increase_counts[active_axis] = 0
                        aligned_since = time.time()
                        print(
                            f"{SENSOR_REPLAY_LOG_PREFIX} "
                            + json.dumps(
                                {
                                    "phase": "preposition",
                                    "event": "axis-accepted",
                                    "reason": "crossed-target-linear",
                                    "axis": active_axis,
                                    "previous_error": round(crossed_from_error, 4),
                                    "current_error": round(active_raw_error, 4),
                                    "acceptance_tolerance": PREPOSITION_LINEAR_CROSS_ACCEPT_M,
                                    "cross_count": close_cross_counts[active_axis],
                                },
                                separators=(",", ":"),
                            ),
                            flush=True,
                        )
                        time.sleep(SENSOR_PREPOSITION_LINEAR_SETTLE_S)
                        continue

                    if abs(active_raw_error) <= PREPOSITION_LINEAR_CROSS_TRIM_M:
                        previous_axis_error = active_raw_error
                        best_axis_error = max(
                            min(best_axis_error or abs_error, abs_error),
                            PREPOSITION_LINEAR_CROSS_TRIM_M,
                        )
                        commands_without_progress = 0
                        error_increase_counts[active_axis] = 0
                        print(
                            f"{SENSOR_REPLAY_LOG_PREFIX} "
                            + json.dumps(
                                {
                                    "phase": "preposition",
                                    "event": "axis-trim",
                                    "reason": "crossed-target-linear",
                                    "axis": active_axis,
                                    "previous_error": round(crossed_from_error, 4),
                                    "current_error": round(active_raw_error, 4),
                                    "acceptance_tolerance": PREPOSITION_LINEAR_CROSS_ACCEPT_M,
                                    "trim_tolerance": PREPOSITION_LINEAR_CROSS_TRIM_M,
                                    "cross_count": close_cross_counts[active_axis],
                                },
                                separators=(",", ":"),
                            ),
                            flush=True,
                        )
                        time.sleep(SENSOR_PREPOSITION_LINEAR_SETTLE_S)
                        continue

                    if close_cross_counts[active_axis] < PREPOSITION_LINEAR_CROSS_CONFIRMATIONS:
                        previous_axis_error = crossed_from_error
                        commands_without_progress = 0
                        error_increase_counts[active_axis] = 0
                        print(
                            f"{SENSOR_REPLAY_LOG_PREFIX} "
                            + json.dumps(
                                {
                                    "phase": "preposition",
                                    "event": "axis-recheck",
                                    "reason": "crossed-target-linear-large",
                                    "axis": active_axis,
                                    "previous_error": round(crossed_from_error, 4),
                                    "current_error": round(active_raw_error, 4),
                                    "trim_tolerance": PREPOSITION_LINEAR_CROSS_TRIM_M,
                                    "cross_count": close_cross_counts[active_axis],
                                },
                                separators=(",", ":"),
                            ),
                            flush=True,
                        )
                        time.sleep(SENSOR_PREPOSITION_LINEAR_SETTLE_S)
                        continue

                    vex_base_bridge.send_hold(ttl_ms=PREPOSITION_COMMAND_TTL_MS)
                    print(
                        f"{SENSOR_REPLAY_LOG_PREFIX} "
                        + json.dumps(
                            {
                                "phase": "preposition",
                                "event": "stopped",
                                "reason": "crossed-target-linear-large",
                                "axis": active_axis,
                                "previous_error": round(crossed_from_error, 4),
                                "current_error": round(active_raw_error, 4),
                                "trim_tolerance": PREPOSITION_LINEAR_CROSS_TRIM_M,
                                "cross_count": close_cross_counts[active_axis],
                            },
                            separators=(",", ":"),
                        ),
                        flush=True,
                    )
                    return _preposition_result(
                        False,
                        reason="crossed-target",
                        axis=active_axis,
                        detail=f"{active_axis} crossed the target by {abs(active_raw_error):.4f}, beyond the trim tolerance.",
                        feedback=feedback,
                    )
                if abs(active_raw_error) <= _axis_cross_accept_tolerance(active_axis):
                    close_cross_counts[active_axis] = close_cross_counts.get(active_axis, 0) + 1
                    best_axis_error = abs_error
                    previous_axis_error = active_raw_error
                    commands_without_progress = 0
                    error_increase_counts[active_axis] = 0
                    vex_base_bridge.send_hold(ttl_ms=PREPOSITION_COMMAND_TTL_MS)
                    if close_cross_counts[active_axis] < PREPOSITION_MAX_CLOSE_CROSSES:
                        print(
                            f"{SENSOR_REPLAY_LOG_PREFIX} "
                            + json.dumps(
                                {
                                    "phase": "preposition",
                                    "event": "axis-trim",
                                    "reason": "crossed-target-close",
                                    "axis": active_axis,
                                    "previous_error": round(crossed_from_error, 4),
                                    "current_error": round(active_raw_error, 4),
                                    "acceptance_tolerance": round(_axis_cross_accept_tolerance(active_axis), 4),
                                    "cross_count": close_cross_counts[active_axis],
                                },
                                separators=(",", ":"),
                            ),
                            flush=True,
                        )
                        time.sleep(
                            SENSOR_PREPOSITION_HEADING_SETTLE_S
                            if active_axis == "heading"
                            else SENSOR_PREPOSITION_LINEAR_SETTLE_S
                        )
                        continue
                    accepted_axes.add(active_axis)
                    aligned_since = time.time()
                    print(
                        f"{SENSOR_REPLAY_LOG_PREFIX} "
                        + json.dumps(
                            {
                                "phase": "preposition",
                                "event": "axis-accepted",
                                "reason": "crossed-target-close",
                                "axis": active_axis,
                                "previous_error": round(crossed_from_error, 4),
                                "current_error": round(active_raw_error, 4),
                                "acceptance_tolerance": round(_axis_cross_accept_tolerance(active_axis), 4),
                                "cross_count": close_cross_counts[active_axis],
                            },
                            separators=(",", ":"),
                        ),
                        flush=True,
                    )
                    time.sleep(
                        SENSOR_PREPOSITION_HEADING_SETTLE_S
                        if active_axis == "heading"
                        else SENSOR_PREPOSITION_LINEAR_SETTLE_S
                    )
                    continue
                if abs(active_raw_error) <= PREPOSITION_HEADING_CROSS_TRIM_DEG:
                    close_cross_counts[active_axis] = close_cross_counts.get(active_axis, 0) + 1
                    best_axis_error = min(best_axis_error or abs_error, abs_error)
                    previous_axis_error = active_raw_error
                    commands_without_progress = 0
                    error_increase_counts[active_axis] = 0
                    vex_base_bridge.send_hold(ttl_ms=PREPOSITION_COMMAND_TTL_MS)
                    if close_cross_counts[active_axis] <= PREPOSITION_HEADING_WIDE_CROSS_CONFIRMATIONS:
                        print(
                            f"{SENSOR_REPLAY_LOG_PREFIX} "
                            + json.dumps(
                                {
                                    "phase": "preposition",
                                    "event": "axis-trim",
                                    "reason": "crossed-target-heading-wide",
                                    "axis": active_axis,
                                    "previous_error": round(crossed_from_error, 4),
                                    "current_error": round(active_raw_error, 4),
                                    "acceptance_tolerance": round(_axis_cross_accept_tolerance(active_axis), 4),
                                    "trim_tolerance": PREPOSITION_HEADING_CROSS_TRIM_DEG,
                                    "cross_count": close_cross_counts[active_axis],
                                },
                                separators=(",", ":"),
                            ),
                            flush=True,
                        )
                        time.sleep(SENSOR_PREPOSITION_HEADING_SETTLE_S)
                        continue
                vex_base_bridge.send_hold(ttl_ms=PREPOSITION_COMMAND_TTL_MS)
                print(
                    f"{SENSOR_REPLAY_LOG_PREFIX} "
                    + json.dumps(
                        {
                            "phase": "preposition",
                            "event": "stopped",
                            "reason": "crossed-target",
                            "axis": active_axis,
                            "previous_error": round(previous_axis_error, 4),
                            "current_error": round(active_raw_error, 4),
                        },
                        separators=(",", ":"),
                    ),
                    flush=True,
                )
                return _preposition_result(
                    False,
                    reason="crossed-target",
                    axis=active_axis,
                    detail=f"{active_axis} crossed the target by {abs(active_raw_error):.4f}, beyond the safe close-crossing tolerance.",
                    feedback=feedback,
                )
            elif (
                previous_axis_error is not None
                and abs_error < abs(float(previous_axis_error)) - progress_epsilon
            ):
                if best_axis_error is None or abs_error < best_axis_error:
                    best_axis_error = abs_error
                previous_axis_error = active_raw_error
                commands_without_progress = 0
                close_cross_counts[active_axis] = 0
                error_increase_counts[active_axis] = 0
            elif best_axis_error is not None and abs_error < best_axis_error - min_progress:
                best_axis_error = abs_error
                previous_axis_error = active_raw_error
                commands_without_progress = 0
                close_cross_counts[active_axis] = 0
                error_increase_counts[active_axis] = 0
            elif best_axis_error is not None and abs_error > best_axis_error + increase_limit:
                error_increase_counts[active_axis] = error_increase_counts.get(active_axis, 0) + 1
                if error_increase_counts[active_axis] < PREPOSITION_ERROR_INCREASE_CONFIRMATIONS:
                    previous_axis_error = active_raw_error
                    commands_without_progress = 0
                    vex_base_bridge.send_hold(ttl_ms=PREPOSITION_COMMAND_TTL_MS)
                    print(
                        f"{SENSOR_REPLAY_LOG_PREFIX} "
                        + json.dumps(
                            {
                                "phase": "preposition",
                                "event": "axis-recheck",
                                "reason": "error-increased",
                                "axis": active_axis,
                                "best_abs_error": round(best_axis_error, 4),
                                "current_abs_error": round(abs_error, 4),
                                "increase_count": error_increase_counts[active_axis],
                                "required_count": PREPOSITION_ERROR_INCREASE_CONFIRMATIONS,
                            },
                            separators=(",", ":"),
                        ),
                        flush=True,
                    )
                    time.sleep(
                        SENSOR_PREPOSITION_HEADING_SETTLE_S
                        if active_axis == "heading"
                        else SENSOR_PREPOSITION_LINEAR_SETTLE_S
                    )
                    continue
                vex_base_bridge.send_hold(ttl_ms=PREPOSITION_COMMAND_TTL_MS)
                print(
                    f"{SENSOR_REPLAY_LOG_PREFIX} "
                    + json.dumps(
                        {
                            "phase": "preposition",
                            "event": "stopped",
                            "reason": "error-increased",
                            "axis": active_axis,
                            "best_abs_error": round(best_axis_error, 4),
                            "current_abs_error": round(abs_error, 4),
                            "increase_count": error_increase_counts[active_axis],
                        },
                        separators=(",", ":"),
                    ),
                    flush=True,
                )
                return _preposition_result(
                    False,
                    reason="error-increased",
                    axis=active_axis,
                    detail=f"{active_axis} error increased from best {best_axis_error:.4f} to {abs_error:.4f}.",
                    feedback=feedback,
                )
            else:
                previous_axis_error = active_raw_error
                commands_without_progress += 1
                if commands_without_progress >= PREPOSITION_MAX_AXIS_COMMANDS_WITHOUT_PROGRESS:
                    vex_base_bridge.send_hold(ttl_ms=PREPOSITION_COMMAND_TTL_MS)
                    print(
                        f"{SENSOR_REPLAY_LOG_PREFIX} "
                        + json.dumps(
                            {
                                "phase": "preposition",
                                "event": "stopped",
                                "reason": "no-progress",
                                "axis": active_axis,
                                "best_abs_error": round(best_axis_error or abs_error, 4),
                                "current_abs_error": round(abs_error, 4),
                                "commands_without_progress": commands_without_progress,
                                "progress_epsilon": round(progress_epsilon, 4),
                            },
                            separators=(",", ":"),
                        ),
                        flush=True,
                    )
                    return _preposition_result(
                        False,
                        reason="no-progress",
                        axis=active_axis,
                        detail=f"{active_axis} did not improve by at least {progress_epsilon:.4f} for {commands_without_progress} correction pulses.",
                        feedback=feedback,
                    )
            if (
                PREPOSITION_MAX_TOTAL_MOTION_COMMANDS > 0
                and total_motion_commands >= PREPOSITION_MAX_TOTAL_MOTION_COMMANDS
            ):
                vex_base_bridge.send_hold(ttl_ms=PREPOSITION_COMMAND_TTL_MS)
                print(
                    f"{SENSOR_REPLAY_LOG_PREFIX} "
                    + json.dumps(
                        {
                            "phase": "preposition",
                            "event": "stopped",
                            "reason": "motion-budget-exhausted",
                            "axis": active_axis,
                            "total_motion_commands": total_motion_commands,
                            "current_abs_error": round(abs_error, 4),
                        },
                        separators=(",", ":"),
                    ),
                    flush=True,
                )
                return _preposition_result(
                    False,
                    reason="motion-budget-exhausted",
                    axis=active_axis,
                    detail=f"{active_axis} used the configured motion-command budget before alignment.",
                    feedback=feedback,
                )

        command_now = time.perf_counter()
        command_dt_s = (
            command_now - last_command_at
            if last_command_at is not None
            else PREPOSITION_RESEND_INTERVAL_S
        )
        command = replay_state.build_command(
            start_state,
            command_dt_s=command_dt_s,
            live_sensor_status=distance_status,
            live_gyro_status=gyro_status,
            force_feedback_only=True,
            active_axes={active_axis},
            feedback=feedback,
        )
        if active_axis != "x":
            command.motion["x.vel"] = 0.0
        if active_axis != "y":
            command.motion["y.vel"] = 0.0
        if active_axis != "heading":
            command.motion["theta.vel"] = 0.0
        else:
            command.motion["theta.vel"] = _preposition_heading_turn_dps(
                float(active_raw_error or 0.0),
                max_turn_speed_dps=replay_state.max_turn_speed_dps,
            )
        command.motion = _limit_motion_by_wheel_percent(
            command.motion,
            max_linear_speed_mps=replay_state.max_linear_speed_mps,
            max_turn_speed_dps=replay_state.max_turn_speed_dps,
            max_abs_wheel_percent=PREPOSITION_MAX_WHEEL_PERCENT,
        )
        # Start-position correction must stay sensor-driven. Do not use wheel
        # position targets here; a bad or stale target can keep driving even
        # when the live ultrasonic/gyro readings are not improving.
        command_ttl_ms = max(
            _preposition_pulse_ttl_ms(
                active_axis,
                abs_error,
                axis_motion_count=axis_motion_counts.get(active_axis, 0),
            ),
            PREPOSITION_HEADING_COMMAND_TTL_MS
            if active_axis == "heading"
            else PREPOSITION_COMMAND_TTL_MS,
        )
        nonzero_motion_axes = [
            axis_name
            for axis_name, key in (("x", "x.vel"), ("y", "y.vel"), ("heading", "theta.vel"))
            if abs(float(command.motion.get(key, 0.0) or 0.0)) > 1e-6
        ]
        print(
            f"{SENSOR_REPLAY_LOG_PREFIX} "
            + json.dumps(
                {
                    "phase": "preposition",
                    "event": "motion-command",
                    "axis": active_axis,
                    "nonzero_axes": nonzero_motion_axes,
                    "ttl_ms": command_ttl_ms,
                    "axis_motion_count": axis_motion_counts.get(active_axis, 0),
                    "motion": {key: round(float(value), 4) for key, value in command.motion.items()},
                    "wheel_pct": _drive_wheel_percentages(
                        command.motion,
                        max_linear_speed_mps=replay_state.max_linear_speed_mps,
                        max_turn_speed_dps=replay_state.max_turn_speed_dps,
                    ),
                    "errors": {
                        "decision": round(active_raw_error, 4),
                        "raw": round(_linear_raw_error(feedback, active_axis), 4)
                        if active_axis != "heading" and _linear_raw_error(feedback, active_axis) is not None
                        else None,
                    },
                },
                separators=(",", ":"),
            ),
            flush=True,
        )
        if not vex_base_bridge.send_motion(command.motion, ttl_ms=command_ttl_ms):
            vex_base_bridge.send_hold(ttl_ms=PREPOSITION_COMMAND_TTL_MS)
            print(
                f"{SENSOR_REPLAY_LOG_PREFIX} "
                + json.dumps(
                    {
                        "phase": "preposition",
                        "event": "stopped",
                        "reason": "command-write-failed",
                        "axis": active_axis,
                        "current_abs_error": round(abs_error, 4),
                    },
                    separators=(",", ":"),
                ),
                flush=True,
            )
            return _preposition_result(
                False,
                reason="command-write-failed",
                axis=active_axis,
                detail=f"The VEX Brain did not accept the {active_axis} correction command, so the base could not move.",
                feedback=feedback,
            )
        total_motion_commands += 1
        axis_motion_counts[active_axis] = axis_motion_counts.get(active_axis, 0) + 1
        last_command_at = command_now
        time.sleep(max(SENSOR_PREPOSITION_RESEND_INTERVAL_S, command_ttl_ms / 1000.0))
        vex_base_bridge.send_hold(ttl_ms=PREPOSITION_COMMAND_TTL_MS)
        time.sleep(
            SENSOR_PREPOSITION_HEADING_SETTLE_S
            if active_axis == "heading"
            else SENSOR_PREPOSITION_LINEAR_SETTLE_S
        )

    vex_base_bridge.send_hold(ttl_ms=PREPOSITION_COMMAND_TTL_MS)
    return _preposition_result(False, reason="timeout", detail="Prepositioning reached its configured timeout.")


def format_sensor_replay_log(
    *,
    elapsed_s: float,
    replay_mode: str,
    correction: dict[str, Any],
) -> str:
    return (
        f"{SENSOR_REPLAY_LOG_PREFIX} "
        + json.dumps(
            {
                "elapsed_s": round(float(elapsed_s), 3),
                "mode": replay_mode,
                "correction": correction,
            },
            separators=(",", ":"),
        )
    )
