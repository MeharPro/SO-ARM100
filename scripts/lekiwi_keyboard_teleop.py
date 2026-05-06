#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import math
import os
import time
from dataclasses import dataclass

from lekiwi_runtime import (
    ACTION_COMMAND_SOURCE_KEY,
    COMMAND_SOURCE_KEYBOARD,
    VEX_CONTROL_MODE_KEY,
    VEX_PIN5_SERVO_POSITION_KEY,
    align_degrees_near_reference,
    parse_position_limits_json,
)
from lekiwi_leader_support import (
    connect_leader_noninteractive,
    detect_leader_port,
    disconnect_device_safely,
)
from lerobot.robots.lekiwi import LeKiwiClient, LeKiwiClientConfig
from lerobot.utils.robot_utils import precise_sleep

try:
    from lerobot.teleoperators.so_leader import SO100Leader, SO100LeaderConfig
except Exception:  # pragma: no cover - available in the runtime lerobot env
    SO100Leader = None
    SO100LeaderConfig = None

try:
    from pynput import keyboard as pynput_keyboard
except Exception:  # pragma: no cover - runtime dependency in the lerobot env
    pynput_keyboard = None

try:
    import Quartz
except Exception:  # pragma: no cover - runtime dependency in the lerobot env
    Quartz = None

FPS = 30
OBSERVATION_POLL_INTERVAL_S = 1.0 / 30.0
KEYBOARD_BASE_LINEAR_SPEED_LIMIT_MPS = 0.06
KEYBOARD_BASE_TURN_SPEED_LIMIT_DPS = 18.0
VEX_DRIVE_CONTROL_MODE = "drive"
VEX_ECU_CONTROL_MODE = "ecu"
VEX_CONTROL_MODES = (VEX_DRIVE_CONTROL_MODE, VEX_ECU_CONTROL_MODE)
ARM_SOURCE_LEADER = "leader"
ARM_SOURCE_KEYBOARD = "keyboard"
ARM_SOURCE_NONE = "none"
ARM_SOURCES = (ARM_SOURCE_LEADER, ARM_SOURCE_KEYBOARD, ARM_SOURCE_NONE)
KEYBOARD_BASE_INITIAL_SPEED_RATIO = 0.25
KEYBOARD_BASE_ACCELERATION_S = 2.5
ENTER_DOUBLE_CLICK_MAX_GAP_S = 0.45
ENTER_SINGLE_HOLD_DELAY_S = 0.12
ENTER_CLOCKWISE_KEY = "enter_clockwise"
ENTER_COUNTER_CLOCKWISE_KEY = "enter_counter_clockwise"
LIMIT_LOCK_ERROR_DEG = 2.0
LIMIT_LOCK_PROGRESS_EPSILON_DEG = 0.2
LIMIT_LOCK_TIMEOUT_S = 0.3
LIMIT_REARM_MARGIN_DEG = 1.5
LIMIT_CONTACT_MARGIN_DEG = 0.75

MAC_VK_TO_KEY = {
    0: "a",
    1: "s",
    11: "b",
    2: "d",
    3: "f",
    4: "h",
    5: "g",
    6: "z",
    7: "x",
    8: "c",
    9: "v",
    12: "q",
    13: "w",
    14: "e",
    15: "r",
    16: "y",
    17: "t",
    29: "0",
    36: "enter",
    31: "o",
    32: "u",
    35: "p",
    34: "i",
    37: "l",
    38: "j",
    40: "k",
    45: "n",
    53: "esc",
    76: "enter",
    123: "arrow_left",
    124: "arrow_right",
    125: "arrow_down",
    126: "arrow_up",
}

QUARTZ_EVENT_SOURCE_IDS = tuple(
    state_id
    for state_id in (
        getattr(Quartz, "kCGEventSourceStateCombinedSessionState", None) if Quartz is not None else None,
        getattr(Quartz, "kCGEventSourceStateHIDSystemState", None) if Quartz is not None else None,
    )
    if state_id is not None
)

ARM_BINDINGS = (
    ("arm_shoulder_pan.pos", ("q",), ("a",), 45.0, 0.0),
    ("arm_shoulder_lift.pos", ("w",), ("s",), 45.0, 0.0),
    ("arm_elbow_flex.pos", ("e",), ("d",), 45.0, 0.0),
    ("arm_wrist_flex.pos", ("r", "y"), ("f", "h"), 60.0, 0.0),
    ("arm_wrist_roll.pos", ("t",), ("g",), 90.0, 0.0),
    ("arm_gripper.pos", ("z", "c"), ("x",), 80.0, 10.0),
)

VEX_PIN5_SERVO_KEY_BINDINGS = {
    "n": "start",
    "b": "up",
    "v": "down",
}

BASE_BINDINGS = (
    ("x.vel", ("arrow_left", "l"), ("arrow_right", "j")),
    ("y.vel", ("arrow_up", "i"), ("arrow_down", "k")),
    ("theta.vel", ("p", ENTER_CLOCKWISE_KEY), ("o", "u", ENTER_COUNTER_CLOCKWISE_KEY)),
)
BASE_CONTROL_KEYS = {
    "0",
    "enter",
    *(
        key
        for _axis, positive_keys, negative_keys in BASE_BINDINGS
        for key in (*positive_keys, *negative_keys)
    ),
}
CONTROL_KEYS = {
    "esc",
    "0",
    "enter",
    *(
        key
        for _joint, positive_keys, negative_keys, *_rest in ARM_BINDINGS
        for key in (*positive_keys, *negative_keys)
    ),
    *(
        key
        for _axis, positive_keys, negative_keys in BASE_BINDINGS
        for key in (*positive_keys, *negative_keys)
    ),
    *VEX_PIN5_SERVO_KEY_BINDINGS.keys(),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Keyboard-only LeKiwi follower teleop with no leader arm."
    )
    parser.add_argument("--remote-host", default=os.getenv("LEKIWI_PI_HOST"))
    parser.add_argument("--robot-id", default=os.getenv("LEKIWI_ROBOT_ID", "follow-mobile"))
    parser.add_argument("--keyboard-id", default=os.getenv("LEKIWI_KEYBOARD_ID", "my_laptop_keyboard"))
    parser.add_argument("--fps", type=int, default=int(os.getenv("LEKIWI_FPS", FPS)))
    parser.add_argument("--base-linear-speed", type=float, default=KEYBOARD_BASE_LINEAR_SPEED_LIMIT_MPS)
    parser.add_argument("--base-turn-speed", type=float, default=KEYBOARD_BASE_TURN_SPEED_LIMIT_DPS)
    parser.add_argument("--drive-base-linear-speed", type=float, default=None)
    parser.add_argument("--drive-base-turn-speed", type=float, default=None)
    parser.add_argument("--initial-vex-control-mode", choices=VEX_CONTROL_MODES, default=VEX_DRIVE_CONTROL_MODE)
    parser.add_argument("--leader-port", default=os.getenv("LEKIWI_LEADER_PORT", "off"))
    parser.add_argument("--leader-id", default=os.getenv("LEKIWI_LEADER_ID", "leader"))
    parser.add_argument("--joint-limits-json", default="{}")
    parser.add_argument("--disable-arm-input", action="store_true")
    parser.add_argument("--disable-base-input", action="store_true")
    parser.add_argument("--arm-source", choices=ARM_SOURCES, default=ARM_SOURCE_KEYBOARD)
    parser.add_argument("--vex-keyboard-calibration-json", default="{}")
    parser.add_argument("--print-every", type=int, default=10)
    parser.add_argument("--connect-timeout-s", type=int, default=10)
    return parser.parse_args()


def axis_from_keys(
    pressed: set[str],
    positive_keys: tuple[str, ...],
    negative_keys: tuple[str, ...],
) -> float:
    return float(any(key in pressed for key in positive_keys)) - float(
        any(key in pressed for key in negative_keys)
    )


def clamp_keyboard_base_speed(value: float, limit: float) -> float:
    if not isinstance(value, (int, float)):
        return 0.0
    if not math.isfinite(float(value)):
        return 0.0
    return max(0.0, min(float(value), float(limit)))


def clamp_unit_interval(value: float) -> float:
    if not isinstance(value, (int, float)):
        return 0.0
    if not math.isfinite(float(value)):
        return 0.0
    return max(0.0, min(float(value), 1.0))


def finite_positive(value: float | None, fallback: float) -> float:
    if isinstance(value, (int, float)) and math.isfinite(float(value)) and float(value) > 0.0:
        return float(value)
    return float(fallback)


def configure_low_latency_zmq(robot: object) -> None:
    zmq_module = getattr(robot, "_zmq", None)
    if zmq_module is None:
        return

    sockets = (
        (getattr(robot, "zmq_cmd_socket", None), ("CONFLATE", "LINGER", "SNDHWM", "IMMEDIATE")),
        (getattr(robot, "zmq_observation_socket", None), ("CONFLATE", "LINGER", "RCVHWM")),
    )
    for socket, option_names in sockets:
        if socket is None:
            continue
        for name in option_names:
            option = getattr(zmq_module, name, None)
            if option is None:
                continue
            value = 0 if name == "LINGER" else 1
            try:
                socket.setsockopt(option, value)
            except Exception:
                pass


def send_latest_action(robot: object, action: dict[str, object]) -> bool:
    zmq_module = getattr(robot, "_zmq", None)
    socket = getattr(robot, "zmq_cmd_socket", None)
    if zmq_module is None or socket is None:
        getattr(robot, "send_action")(action)
        return True

    try:
        socket.send_string(json.dumps(action), flags=getattr(zmq_module, "NOBLOCK", 0))
        return True
    except Exception as exc:
        again_type = getattr(zmq_module, "Again", None)
        if isinstance(again_type, type) and isinstance(exc, again_type):
            return False
        raise


def _normalize_direction_sign(value: object) -> float:
    return -1.0 if value == -1 or value == -1.0 or value == "-1" else 1.0


def normalize_keyboard_direction_calibration(raw_json: str | None) -> dict[str, float]:
    try:
        payload = json.loads(raw_json or "{}")
    except Exception:
        payload = {}
    if not isinstance(payload, dict):
        payload = {}
    return {
        "x.vel": _normalize_direction_sign(payload.get("xSign", 1)),
        "y.vel": _normalize_direction_sign(payload.get("ySign", 1)),
        "theta.vel": _normalize_direction_sign(payload.get("thetaSign", 1)),
    }


def keyboard_arm_active(pressed: set[str]) -> bool:
    return any(
        key in pressed
        for _joint, positive_keys, negative_keys, *_rest in ARM_BINDINGS
        for key in (*positive_keys, *negative_keys)
    )


def filter_base_control_keys(pressed: set[str]) -> set[str]:
    return {key for key in pressed if key not in BASE_CONTROL_KEYS}


class EnterRotationClickState:
    def __init__(
        self,
        *,
        double_click_max_gap_s: float = ENTER_DOUBLE_CLICK_MAX_GAP_S,
        single_hold_delay_s: float = ENTER_SINGLE_HOLD_DELAY_S,
    ) -> None:
        self.double_click_max_gap_s = max(float(double_click_max_gap_s), 0.0)
        self.single_hold_delay_s = max(float(single_hold_delay_s), 0.0)
        self._enter_was_down = False
        self._press_started_at: float | None = None
        self._quick_tap_released_at: float | None = None
        self._active_virtual_key: str | None = None

    def update(self, pressed: set[str], now_s: float) -> set[str]:
        next_pressed = set(pressed)
        enter_down = "enter" in pressed

        if (
            self._quick_tap_released_at is not None
            and now_s - self._quick_tap_released_at > self.double_click_max_gap_s
        ):
            self._quick_tap_released_at = None

        if enter_down and not self._enter_was_down:
            self._press_started_at = now_s
            if (
                self._quick_tap_released_at is not None
                and now_s - self._quick_tap_released_at <= self.double_click_max_gap_s
            ):
                self._active_virtual_key = ENTER_COUNTER_CLOCKWISE_KEY
                self._quick_tap_released_at = None
            else:
                self._active_virtual_key = None

        if enter_down:
            if (
                self._active_virtual_key is None
                and self._press_started_at is not None
                and now_s - self._press_started_at >= self.single_hold_delay_s
            ):
                self._active_virtual_key = ENTER_CLOCKWISE_KEY
            if self._active_virtual_key is not None:
                next_pressed.add(self._active_virtual_key)
        elif self._enter_was_down:
            if self._active_virtual_key is None:
                self._quick_tap_released_at = now_s
            else:
                self._quick_tap_released_at = None
            self._press_started_at = None
            self._active_virtual_key = None

        self._enter_was_down = enter_down
        next_pressed.discard("enter")
        return next_pressed


class VexControlModeState:
    def __init__(self, initial_mode: str) -> None:
        self.mode = initial_mode if initial_mode in VEX_CONTROL_MODES else VEX_DRIVE_CONTROL_MODE

    def update(self, newly_pressed: set[str]) -> bool:
        if "0" not in newly_pressed:
            return False
        self.mode = VEX_ECU_CONTROL_MODE if self.mode == VEX_DRIVE_CONTROL_MODE else VEX_DRIVE_CONTROL_MODE
        return True

    def speed_pair(
        self,
        *,
        drive_linear_speed: float,
        drive_turn_speed: float,
        ecu_linear_speed: float,
        ecu_turn_speed: float,
    ) -> tuple[float, float]:
        if self.mode == VEX_ECU_CONTROL_MODE:
            return ecu_linear_speed, ecu_turn_speed
        return drive_linear_speed, drive_turn_speed


def _poll_quartz_pressed_keys() -> set[str]:
    if Quartz is None or not QUARTZ_EVENT_SOURCE_IDS:
        return set()

    pressed: set[str] = set()
    for vk, alias in MAC_VK_TO_KEY.items():
        try:
            if any(bool(Quartz.CGEventSourceKeyState(source_id, vk)) for source_id in QUARTZ_EVENT_SOURCE_IDS):
                pressed.add(alias)
        except Exception:
            continue
    return pressed


def _normalize_key_symbols(key: object) -> tuple[str, ...]:
    symbols: list[str] = []
    if pynput_keyboard is not None:
        special_key_aliases = (
            (getattr(pynput_keyboard.Key, "enter", None), "enter"),
            (getattr(pynput_keyboard.Key, "esc", None), "esc"),
            (getattr(pynput_keyboard.Key, "up", None), "arrow_up"),
            (getattr(pynput_keyboard.Key, "down", None), "arrow_down"),
            (getattr(pynput_keyboard.Key, "left", None), "arrow_left"),
            (getattr(pynput_keyboard.Key, "right", None), "arrow_right"),
        )
        for special_key, alias in special_key_aliases:
            if special_key is not None and key == special_key:
                symbols.append(alias)

    char = getattr(key, "char", None)
    if isinstance(char, str) and char:
        symbols.append(char.lower())

    vk = getattr(key, "vk", None)
    if isinstance(vk, int):
        alias = MAC_VK_TO_KEY.get(vk)
        if alias:
            symbols.append(alias)
        else:
            try:
                normalized = chr(vk)
            except Exception:
                normalized = ""
            if normalized and normalized.isprintable():
                symbols.append(normalized.lower())

    deduped: list[str] = []
    for symbol in symbols:
        if symbol and symbol not in deduped:
            deduped.append(symbol)
    return tuple(deduped)


class PressedKeyTracker:
    def __init__(self) -> None:
        self.current_pressed: dict[str, bool] = {}
        self.listener = None
        self._escape_down = False

    @property
    def is_connected(self) -> bool:
        listener_ready = (
            pynput_keyboard is not None
            and self.listener is not None
            and self.listener.is_alive()
        )
        return listener_ready or Quartz is not None

    def connect(self) -> None:
        if pynput_keyboard is None and Quartz is None:
            raise SystemExit(
                "Neither pynput nor Quartz keyboard capture is available in the active lerobot environment."
            )
        if pynput_keyboard is not None:
            self.listener = pynput_keyboard.Listener(
                on_press=self._on_press,
                on_release=self._on_release,
            )
            self.listener.start()

    def _on_press(self, key: object) -> None:
        for normalized in _normalize_key_symbols(key):
            self.current_pressed[normalized] = True

    def _on_release(self, key: object) -> None:
        for normalized in _normalize_key_symbols(key):
            self.current_pressed[normalized] = False
        if pynput_keyboard is not None and key == pynput_keyboard.Key.esc:
            print("ESC pressed. Stopping keyboard backup teleop.", flush=True)
            self.disconnect()

    def get_pressed(self) -> set[str]:
        quartz_pressed = _poll_quartz_pressed_keys()
        listener_pressed = {
            key
            for key, is_pressed in self.current_pressed.items()
            if is_pressed and key in CONTROL_KEYS
        }

        # Quartz can under-report simultaneous key combos in some launch contexts.
        # Merge it with pynput so arm/base combos like move+grip still register.
        if Quartz is not None:
            pressed = {key for key in quartz_pressed if key in CONTROL_KEYS} | set(listener_pressed)
        else:
            pressed = listener_pressed

        escape_pressed = "esc" in pressed
        if escape_pressed and not self._escape_down:
            self._escape_down = True
            print("ESC pressed. Stopping keyboard backup teleop.", flush=True)
            self.disconnect()
        elif not escape_pressed:
            self._escape_down = False

        pressed.discard("esc")
        return pressed

    def disconnect(self) -> None:
        if self.listener is not None:
            self.listener.stop()
            self.listener = None


def seed_arm_targets(robot: LeKiwiClient) -> dict[str, float]:
    observation = robot.get_observation()
    missing = [joint for joint, *_rest in ARM_BINDINGS if joint not in observation]
    if missing:
        available = ", ".join(sorted(map(str, observation.keys())))
        raise RuntimeError(
            "The Pi host observation is missing arm joints: "
            + ", ".join(missing)
            + f". Available keys: {available}"
        )

    return {
        joint: float(observation[joint])
        for joint, *_rest in ARM_BINDINGS
    }


def read_arm_positions(
    robot: LeKiwiClient,
    fallback: dict[str, float] | None = None,
) -> dict[str, float]:
    observation = robot.get_observation()
    positions: dict[str, float] = {}
    for joint, *_rest in ARM_BINDINGS:
        value = observation.get(joint)
        if isinstance(value, (int, float)):
            positions[joint] = float(value)
        elif fallback is not None and joint in fallback:
            positions[joint] = float(fallback[joint])
    return positions


class LeaderActionMapper:
    def __init__(self) -> None:
        self.last_wrist_roll: float | None = None

    def build_arm_action(self, leader_action: dict[str, float]) -> dict[str, float]:
        arm_action = {f"arm_{key}": value for key, value in leader_action.items()}
        wrist_key = "arm_wrist_roll.pos"
        if wrist_key in arm_action:
            wrist_value = float(arm_action[wrist_key])
            if self.last_wrist_roll is not None:
                wrist_value = align_degrees_near_reference(wrist_value, self.last_wrist_roll)
            self.last_wrist_roll = wrist_value
            arm_action[wrist_key] = wrist_value

        return {
            joint: float(arm_action[joint])
            for joint, *_rest in ARM_BINDINGS
            if joint in arm_action and isinstance(arm_action[joint], (int, float))
        }


@dataclass
class JointDirectionLimitState:
    last_observed: float | None = None
    positive_candidate_since: float | None = None
    negative_candidate_since: float | None = None
    positive_locked_at: float | None = None
    negative_locked_at: float | None = None


class JointDirectionLimiter:
    def __init__(self, position_limits: dict[str, tuple[float, float]]) -> None:
        self.position_limits = position_limits
        self.states = {
            joint: JointDirectionLimitState()
            for joint, *_rest in ARM_BINDINGS
        }

    def seed(self, observed_positions: dict[str, float]) -> None:
        for joint, value in observed_positions.items():
            state = self.states.get(joint)
            if state is not None:
                state.last_observed = float(value)

    def is_locked(self, joint: str, direction: str) -> bool:
        state = self.states[joint]
        if direction == "positive":
            return state.positive_locked_at is not None
        return state.negative_locked_at is not None

    def clamp_target(self, joint: str, target: float, observed: float | None) -> float:
        if observed is None:
            return float(target)
        clamped = float(target)
        if self.is_locked(joint, "positive"):
            clamped = min(clamped, float(observed))
        if self.is_locked(joint, "negative"):
            clamped = max(clamped, float(observed))
        return clamped

    def update(
        self,
        observed_positions: dict[str, float],
        arm_targets: dict[str, float],
        pressed: set[str],
        now_s: float,
    ) -> list[str]:
        events: list[str] = []
        for joint, positive_keys, negative_keys, *_rest in ARM_BINDINGS:
            observed = observed_positions.get(joint)
            if not isinstance(observed, (int, float)):
                continue

            observed_value = float(observed)
            state = self.states[joint]

            if (
                state.positive_locked_at is not None
                and observed_value <= state.positive_locked_at - LIMIT_REARM_MARGIN_DEG
            ):
                state.positive_locked_at = None
                events.append(f"[limit] {joint} positive input re-enabled.")
            if (
                state.negative_locked_at is not None
                and observed_value >= state.negative_locked_at + LIMIT_REARM_MARGIN_DEG
            ):
                state.negative_locked_at = None
                events.append(f"[limit] {joint} negative input re-enabled.")

            previous_observed = state.last_observed
            progress = (
                observed_value - previous_observed
                if previous_observed is not None
                else None
            )

            positive_active = any(key in pressed for key in positive_keys)
            negative_active = any(key in pressed for key in negative_keys)
            target = float(arm_targets.get(joint, observed_value))
            low, high = self.position_limits.get(joint, (None, None))

            if state.positive_locked_at is None:
                positive_error = max(target - observed_value, 0.0)
                near_high_limit = (
                    isinstance(high, (int, float))
                    and observed_value >= float(high) - LIMIT_CONTACT_MARGIN_DEG
                )
                moving_positive = (
                    progress is not None
                    and progress >= LIMIT_LOCK_PROGRESS_EPSILON_DEG
                )
                should_watch_positive = (
                    positive_active
                    and not negative_active
                    and (near_high_limit or (positive_error >= LIMIT_LOCK_ERROR_DEG and not moving_positive))
                )
                if should_watch_positive:
                    if state.positive_candidate_since is None:
                        state.positive_candidate_since = now_s
                    elif now_s - state.positive_candidate_since >= LIMIT_LOCK_TIMEOUT_S:
                        state.positive_locked_at = observed_value
                        state.positive_candidate_since = None
                        events.append(
                            f"[limit] {joint} positive input blocked at {observed_value:.2f} deg; reverse input is still allowed."
                        )
                else:
                    state.positive_candidate_since = None
            else:
                state.positive_candidate_since = None

            if state.negative_locked_at is None:
                negative_error = max(observed_value - target, 0.0)
                near_low_limit = (
                    isinstance(low, (int, float))
                    and observed_value <= float(low) + LIMIT_CONTACT_MARGIN_DEG
                )
                moving_negative = (
                    progress is not None
                    and (-progress) >= LIMIT_LOCK_PROGRESS_EPSILON_DEG
                )
                should_watch_negative = (
                    negative_active
                    and not positive_active
                    and (near_low_limit or (negative_error >= LIMIT_LOCK_ERROR_DEG and not moving_negative))
                )
                if should_watch_negative:
                    if state.negative_candidate_since is None:
                        state.negative_candidate_since = now_s
                    elif now_s - state.negative_candidate_since >= LIMIT_LOCK_TIMEOUT_S:
                        state.negative_locked_at = observed_value
                        state.negative_candidate_since = None
                        events.append(
                            f"[limit] {joint} negative input blocked at {observed_value:.2f} deg; reverse input is still allowed."
                        )
                else:
                    state.negative_candidate_since = None
            else:
                state.negative_candidate_since = None

            state.last_observed = observed_value

        return events


def build_base_action(
    pressed: set[str],
    linear_speed: float,
    turn_speed: float,
    axis_speed_scales: dict[str, float] | None = None,
    linear_speed_limit: float = KEYBOARD_BASE_LINEAR_SPEED_LIMIT_MPS,
    turn_speed_limit: float = KEYBOARD_BASE_TURN_SPEED_LIMIT_DPS,
    direction_signs: dict[str, float] | None = None,
) -> dict[str, float]:
    limited_linear_speed = clamp_keyboard_base_speed(
        linear_speed,
        linear_speed_limit,
    )
    limited_turn_speed = clamp_keyboard_base_speed(
        turn_speed,
        turn_speed_limit,
    )
    return {
        axis: axis_from_keys(pressed, positive_keys, negative_keys)
        * (limited_turn_speed if axis == "theta.vel" else limited_linear_speed)
        * clamp_unit_interval((axis_speed_scales or {}).get(axis, 1.0))
        * (direction_signs or {}).get(axis, 1.0)
        for axis, positive_keys, negative_keys in BASE_BINDINGS
    }


class KeyboardBaseRamp:
    def __init__(self) -> None:
        self.active_since: dict[str, float] = {}
        self.active_sign: dict[str, float] = {}

    def update(
        self,
        pressed: set[str],
        *,
        linear_speed: float,
        turn_speed: float,
        now_s: float,
        linear_speed_limit: float = KEYBOARD_BASE_LINEAR_SPEED_LIMIT_MPS,
        turn_speed_limit: float = KEYBOARD_BASE_TURN_SPEED_LIMIT_DPS,
        direction_signs: dict[str, float] | None = None,
    ) -> dict[str, float]:
        scales: dict[str, float] = {}
        active_axes: set[str] = set()
        for axis, positive_keys, negative_keys in BASE_BINDINGS:
            sign = axis_from_keys(pressed, positive_keys, negative_keys)
            if sign == 0.0:
                self.active_since.pop(axis, None)
                self.active_sign.pop(axis, None)
                continue

            active_axes.add(axis)
            if self.active_sign.get(axis) != sign:
                self.active_sign[axis] = sign
                self.active_since[axis] = now_s

            held_s = max(now_s - self.active_since.get(axis, now_s), 0.0)
            ramp_progress = clamp_unit_interval(held_s / KEYBOARD_BASE_ACCELERATION_S)
            scales[axis] = (
                KEYBOARD_BASE_INITIAL_SPEED_RATIO
                + (1.0 - KEYBOARD_BASE_INITIAL_SPEED_RATIO) * ramp_progress
            )

        for axis in tuple(self.active_since):
            if axis not in active_axes:
                self.active_since.pop(axis, None)
                self.active_sign.pop(axis, None)

        return build_base_action(
            pressed,
            linear_speed=linear_speed,
            turn_speed=turn_speed,
            axis_speed_scales=scales,
            linear_speed_limit=linear_speed_limit,
            turn_speed_limit=turn_speed_limit,
            direction_signs=direction_signs,
        )


def update_arm_targets(
    arm_targets: dict[str, float],
    observed_positions: dict[str, float],
    pressed: set[str],
    newly_pressed: set[str],
    dt_s: float,
    position_limits: dict[str, tuple[float, float]],
    direction_limiter: JointDirectionLimiter,
) -> dict[str, float]:
    next_targets = dict(arm_targets)
    for joint, positive_keys, negative_keys, rate_deg_s, tap_step in ARM_BINDINGS:
        observed = observed_positions.get(joint)
        if isinstance(observed, (int, float)):
            next_targets[joint] = direction_limiter.clamp_target(
                joint,
                next_targets[joint],
                float(observed),
            )

        allowed_positive_keys = () if direction_limiter.is_locked(joint, "positive") else positive_keys
        allowed_negative_keys = () if direction_limiter.is_locked(joint, "negative") else negative_keys
        delta = axis_from_keys(pressed, allowed_positive_keys, allowed_negative_keys) * rate_deg_s * dt_s
        delta += axis_from_keys(newly_pressed, allowed_positive_keys, allowed_negative_keys) * tap_step
        if delta:
            next_value = float(next_targets[joint]) + delta
            joint_limits = position_limits.get(joint)
            if joint_limits is not None:
                low, high = joint_limits
                next_value = min(max(next_value, low), high)
            next_targets[joint] = next_value
    return next_targets


def apply_arm_authority(
    arm_targets: dict[str, float],
    observed_positions: dict[str, float],
    pressed: set[str],
    newly_pressed: set[str],
    dt_s: float,
    position_limits: dict[str, tuple[float, float]],
    direction_limiter: JointDirectionLimiter,
    *,
    keyboard_arm_input_enabled: bool,
    leader_arm_action: dict[str, float] | None,
) -> dict[str, float]:
    if leader_arm_action is not None:
        next_targets = dict(arm_targets)
        next_targets.update(leader_arm_action)
        return next_targets
    if keyboard_arm_input_enabled:
        return update_arm_targets(
            arm_targets,
            observed_positions,
            pressed,
            newly_pressed,
            dt_s,
            position_limits,
            direction_limiter,
        )
    return dict(arm_targets)


def summarize_action(action: dict[str, float]) -> str:
    arm_parts = [
        f"{joint.removeprefix('arm_')}={action[joint]:7.2f}"
        for joint, *_rest in ARM_BINDINGS
        if joint in action
    ]
    arm_summary = " ".join(arm_parts) if arm_parts else "arm=not-commanded"
    base_parts = [
        f"{axis}={action[axis]:6.2f}"
        for axis, *_rest in BASE_BINDINGS
        if axis in action
    ]
    base_summary = " ".join(base_parts) if base_parts else "base=not-commanded"
    return f"{arm_summary} | {base_summary}"


def vex_pin5_servo_position_from_keys(newly_pressed: set[str]) -> str | None:
    for key, position in VEX_PIN5_SERVO_KEY_BINDINGS.items():
        if key in newly_pressed:
            return position
    return None


def should_send_arm_targets(configured_arm_source: str, leader_connected: bool) -> bool:
    if configured_arm_source == ARM_SOURCE_KEYBOARD:
        return True
    if configured_arm_source == ARM_SOURCE_LEADER and leader_connected:
        return True
    return False


def build_teleop_action(
    arm_targets: dict[str, float],
    base_action: dict[str, float],
    *,
    mode: str,
    include_arm_targets: bool,
    include_base_action: bool = True,
    vex_pin5_servo_position: str | None = None,
) -> dict[str, float | str]:
    action: dict[str, float | str] = {}
    if include_arm_targets:
        action.update(arm_targets)
    if include_base_action:
        action.update(base_action)
    action[ACTION_COMMAND_SOURCE_KEY] = COMMAND_SOURCE_KEYBOARD
    if include_base_action:
        action[VEX_CONTROL_MODE_KEY] = mode
    if vex_pin5_servo_position is not None:
        action[VEX_PIN5_SERVO_POSITION_KEY] = vex_pin5_servo_position
    return action


def print_controls(
    leader_enabled: bool,
    mode: str,
    arm_input_enabled: bool = True,
    base_input_enabled: bool = True,
) -> None:
    if leader_enabled:
        label = "Leader arm + keyboard base teleop" if base_input_enabled else "Leader arm + VEX controller base teleop"
    elif arm_input_enabled:
        label = "Keyboard arm + keyboard base teleop" if base_input_enabled else "Keyboard arm + VEX controller base teleop"
    else:
        label = "Keyboard base teleop" if base_input_enabled else "VEX controller base teleop"
    base_mode = f" VEX base mode: {mode.upper()}." if base_input_enabled else " Keyboard VEX base input is disabled."
    print(f"{label} is live.{base_mode}", flush=True)
    if leader_enabled:
        print("Arm: leader arm owns the follower; keyboard arm keys are ignored.", flush=True)
    elif arm_input_enabled:
        print(
            "Arm: Q/A shoulder pan, W/S shoulder lift, E/D elbow, "
            "R/F wrist flex (also Y/H), T/G wrist roll, Z/X gripper (also C).",
            flush=True,
        )
    else:
        print("Arm: not commanded by keyboard teleop; keyboard arm keys are ignored.", flush=True)
    if base_input_enabled:
        print(
            "Base: ArrowUp/ArrowDown forward-back (Y), ArrowLeft/ArrowRight strafe (X), O/P rotate. "
            "Enter hold rotates clockwise; quick Enter tap then hold rotates counter-clockwise. "
            "Legacy I/K and J/L are fallback translation keys; U is legacy rotate-left. "
            "VEX Pin 5 servo: N start, B -90 deg, V -180 deg. "
            "Press 0 to toggle Drive/ECU speed, Esc to stop keyboard capture.",
            flush=True,
        )
    else:
        print(
            "Base: keyboard VEX base keys are ignored; use the VEX controller for base motion. "
            "VEX Pin 5 servo: N start, B -90 deg, V -180 deg. "
            "Esc stops keyboard capture.",
            flush=True,
        )


def main() -> None:
    args = parse_args()
    if not args.remote_host:
        raise SystemExit("--remote-host or LEKIWI_PI_HOST is required.")
    position_limits = parse_position_limits_json(args.joint_limits_json)

    robot = LeKiwiClient(
        LeKiwiClientConfig(
            remote_ip=args.remote_host,
            id=args.robot_id,
            connect_timeout_s=args.connect_timeout_s,
        )
    )
    keyboard = PressedKeyTracker()
    leader = None
    leader_connected = False
    leader_mapper = LeaderActionMapper()
    keyboard_connected = False
    ecu_linear_speed = finite_positive(args.base_linear_speed, KEYBOARD_BASE_LINEAR_SPEED_LIMIT_MPS)
    ecu_turn_speed = finite_positive(args.base_turn_speed, KEYBOARD_BASE_TURN_SPEED_LIMIT_DPS)
    drive_linear_speed = finite_positive(args.drive_base_linear_speed, ecu_linear_speed)
    drive_turn_speed = finite_positive(args.drive_base_turn_speed, ecu_turn_speed)
    mode_state = VexControlModeState(args.initial_vex_control_mode)
    configured_arm_source = ARM_SOURCE_NONE if args.disable_arm_input else args.arm_source
    keyboard_arm_input_enabled = configured_arm_source == ARM_SOURCE_KEYBOARD
    keyboard_base_input_enabled = not args.disable_base_input
    leader_requested = (
        configured_arm_source == ARM_SOURCE_LEADER
        and str(args.leader_port or "off").strip().lower() != "off"
    )
    direction_signs = normalize_keyboard_direction_calibration(args.vex_keyboard_calibration_json)

    loop_idx = 0
    last_loop_t = time.perf_counter()
    arm_targets: dict[str, float] | None = None
    observed_arm_positions: dict[str, float] = {}
    last_pressed: set[str] = set()
    next_observation_poll_t = 0.0
    base_ramp = KeyboardBaseRamp()
    enter_rotation = EnterRotationClickState()
    dropped_send_count = 0
    next_drop_log_t = 0.0

    try:
        print(f"Connecting to LeKiwi host at {args.remote_host}", flush=True)
        robot.connect()
        configure_low_latency_zmq(robot)
        if leader_requested:
            try:
                if SO100Leader is None or SO100LeaderConfig is None:
                    raise RuntimeError("SO100 leader support is unavailable in the active lerobot environment.")
                leader_port = detect_leader_port(args.leader_port)
                leader = SO100Leader(SO100LeaderConfig(port=leader_port, id=args.leader_id, use_degrees=True))
                connect_leader_noninteractive(
                    leader,
                    calibrate_hint=(
                        "Run the dashboard's Mac calibration, or run "
                        "`lerobot-calibrate --teleop.type=so100_leader --teleop.port <port> "
                        f"--teleop.id {args.leader_id}`."
                    ),
                )
                leader_connected = True
                print(f"Leader arm connected on {leader_port}.", flush=True)
            except SystemExit as exc:
                leader = None
                print(f"[leader] unavailable, continuing with keyboard backup only: {exc}", flush=True)
            except Exception as exc:
                leader = None
                print(f"[leader] unavailable, continuing with keyboard backup only: {exc}", flush=True)

        try:
            keyboard.connect()
            keyboard_connected = keyboard.is_connected
        except SystemExit:
            keyboard_connected = False
            if not leader_connected:
                raise
            print("[keyboard] capture unavailable; leader arm remains live.", flush=True)

        if configured_arm_source == ARM_SOURCE_LEADER and not leader_connected:
            print("[leader] requested but unavailable; arm input remains disabled for safety.", flush=True)

        if not keyboard_connected and not leader_connected:
            raise SystemExit(
                "Neither keyboard capture nor the leader arm started. On macOS, grant Input Monitoring/Accessibility access to the terminal/app running this process, or connect the leader arm."
            )

        arm_targets = seed_arm_targets(robot)
        observed_arm_positions = dict(arm_targets)
        direction_limiter = JointDirectionLimiter(position_limits)
        direction_limiter.seed(observed_arm_positions)
        next_observation_poll_t = time.perf_counter()
        print_controls(
            leader_connected,
            mode_state.mode,
            keyboard_arm_input_enabled,
            keyboard_base_input_enabled,
        )

        while keyboard.is_connected or leader_connected:
            loop_start_t = time.perf_counter()
            raw_pressed = keyboard.get_pressed() if keyboard.is_connected else set()
            if not keyboard_base_input_enabled:
                raw_pressed = filter_base_control_keys(raw_pressed)
            pressed = enter_rotation.update(raw_pressed, loop_start_t)
            newly_pressed = pressed - last_pressed
            last_pressed = pressed
            arm_pressed = pressed if keyboard_arm_input_enabled else set()
            arm_newly_pressed = newly_pressed if keyboard_arm_input_enabled else set()
            dt_s = max(loop_start_t - last_loop_t, 0.0)
            last_loop_t = loop_start_t
            if keyboard_base_input_enabled and mode_state.update(newly_pressed):
                print(f"VEX base mode switched to {mode_state.mode.upper()} speed.", flush=True)

            if loop_start_t >= next_observation_poll_t:
                try:
                    observed_arm_positions = read_arm_positions(
                        robot,
                        fallback=observed_arm_positions or arm_targets,
                    )
                    for event in direction_limiter.update(
                        observed_arm_positions,
                        arm_targets,
                        arm_pressed,
                        loop_start_t,
                    ):
                        print(event, flush=True)
                except Exception:
                    pass
                next_observation_poll_t = loop_start_t + OBSERVATION_POLL_INTERVAL_S

            leader_arm_action: dict[str, float] | None = None
            if leader_connected and leader is not None:
                try:
                    leader_arm_action = leader_mapper.build_arm_action(leader.get_action())
                except Exception as exc:
                    print(f"[leader] action failed; keyboard backup remains live: {exc}", flush=True)
                    disconnect_device_safely(leader, "leader arm")
                    leader = None
                    leader_connected = False

            arm_targets = apply_arm_authority(
                arm_targets,
                observed_arm_positions,
                arm_pressed,
                arm_newly_pressed,
                dt_s,
                position_limits,
                direction_limiter,
                keyboard_arm_input_enabled=keyboard_arm_input_enabled,
                leader_arm_action=leader_arm_action,
            )

            linear_speed, turn_speed = mode_state.speed_pair(
                drive_linear_speed=drive_linear_speed,
                drive_turn_speed=drive_turn_speed,
                ecu_linear_speed=ecu_linear_speed,
                ecu_turn_speed=ecu_turn_speed,
            )
            if keyboard_base_input_enabled:
                base_action = base_ramp.update(
                    pressed,
                    linear_speed=linear_speed,
                    turn_speed=turn_speed,
                    linear_speed_limit=linear_speed,
                    turn_speed_limit=turn_speed,
                    now_s=loop_start_t,
                    direction_signs=direction_signs,
                )
            else:
                base_ramp.update(
                    set(),
                    linear_speed=linear_speed,
                    turn_speed=turn_speed,
                    now_s=loop_start_t,
                )
                base_action = {}
            vex_pin5_servo_position = vex_pin5_servo_position_from_keys(newly_pressed)
            if vex_pin5_servo_position is not None:
                print(f"VEX Pin 5 servo -> {vex_pin5_servo_position}.", flush=True)
            include_arm_targets = should_send_arm_targets(configured_arm_source, leader_connected)
            action = build_teleop_action(
                arm_targets,
                base_action,
                mode=mode_state.mode,
                include_arm_targets=include_arm_targets,
                include_base_action=keyboard_base_input_enabled,
                vex_pin5_servo_position=vex_pin5_servo_position,
            )
            if include_arm_targets:
                arm_targets = {
                    joint: float(action[joint])
                    for joint, *_rest in ARM_BINDINGS
                }

            if loop_idx % args.print_every == 0:
                active_keys = "".join(sorted(pressed)) or "-"
                leader_state = "on" if leader_connected else "off"
                print(
                    f"{summarize_action(action)} | keys={active_keys} | vex={mode_state.mode} | leader={leader_state}",
                    flush=True,
                )

            if send_latest_action(robot, action):
                dropped_send_count = 0
            else:
                dropped_send_count += 1
                if loop_start_t >= next_drop_log_t:
                    print(
                        f"[network] command socket is full; dropped {dropped_send_count} stale frame(s).",
                        flush=True,
                    )
                    next_drop_log_t = loop_start_t + 1.0
            loop_idx += 1
            precise_sleep(max(1.0 / args.fps - (time.perf_counter() - loop_start_t), 0.0))
    except KeyboardInterrupt:
        print("\nStopping keyboard backup teleop.", flush=True)
    finally:
        if arm_targets is not None and robot.is_connected:
            try:
                stop_action = {
                }
                if keyboard_base_input_enabled:
                    stop_action.update({"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0})
                if should_send_arm_targets(configured_arm_source, leader_connected):
                    stop_action.update(arm_targets)
                robot.send_action(stop_action)
            except Exception:
                pass
        keyboard.disconnect()
        if leader is not None:
            disconnect_device_safely(leader, "leader arm")
        if robot.is_connected:
            robot.disconnect()


if __name__ == "__main__":
    main()
