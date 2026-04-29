#!/usr/bin/env python3

from __future__ import annotations

import argparse
import math
import os
import time
from dataclasses import dataclass

from lekiwi_runtime import (
    ACTION_COMMAND_SOURCE_KEY,
    COMMAND_SOURCE_KEYBOARD,
    VEX_CONTROL_MODE_KEY,
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
KEYBOARD_BASE_INITIAL_SPEED_RATIO = 0.25
KEYBOARD_BASE_ACCELERATION_S = 2.5
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
    31: "o",
    32: "u",
    34: "i",
    37: "l",
    38: "j",
    40: "k",
    45: "n",
    53: "esc",
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
    ("arm_gripper.pos", ("z", "c", "b"), ("x", "v", "n"), 80.0, 10.0),
)

BASE_BINDINGS = (
    ("x.vel", ("l",), ("j",)),
    ("y.vel", ("i",), ("k",)),
    ("theta.vel", ("o",), ("u",)),
)
CONTROL_KEYS = {
    "esc",
    "0",
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


def keyboard_arm_active(pressed: set[str]) -> bool:
    return any(
        key in pressed
        for _joint, positive_keys, negative_keys, *_rest in ARM_BINDINGS
        for key in (*positive_keys, *negative_keys)
    )


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


def summarize_action(action: dict[str, float]) -> str:
    arm_summary = " ".join(
        f"{joint.removeprefix('arm_')}={action[joint]:7.2f}"
        for joint, *_rest in ARM_BINDINGS
    )
    base_summary = " ".join(
        f"{axis}={action[axis]:6.2f}"
        for axis, *_rest in BASE_BINDINGS
    )
    return f"{arm_summary} | {base_summary}"


def print_controls(leader_enabled: bool, mode: str) -> None:
    label = "Keyboard + Leader teleop" if leader_enabled else "Keyboard backup teleop"
    print(f"{label} is live. VEX base mode: {mode.upper()}.", flush=True)
    print(
        "Arm: Q/A shoulder pan, W/S shoulder lift, E/D elbow, "
        "R/F wrist flex (also Y/H), T/G wrist roll, Z/X gripper (also C/V and B/N).",
        flush=True,
    )
    print(
        "Base: I/K forward-back (Y), J/L left-right (X), U/O turn. "
        "Press 0 to toggle Drive/ECU speed, Esc to stop keyboard capture.",
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
    leader_requested = str(args.leader_port or "off").strip().lower() != "off"
    keyboard_connected = False
    ecu_linear_speed = finite_positive(args.base_linear_speed, KEYBOARD_BASE_LINEAR_SPEED_LIMIT_MPS)
    ecu_turn_speed = finite_positive(args.base_turn_speed, KEYBOARD_BASE_TURN_SPEED_LIMIT_DPS)
    drive_linear_speed = finite_positive(args.drive_base_linear_speed, ecu_linear_speed)
    drive_turn_speed = finite_positive(args.drive_base_turn_speed, ecu_turn_speed)
    mode_state = VexControlModeState(args.initial_vex_control_mode)

    loop_idx = 0
    last_loop_t = time.perf_counter()
    arm_targets: dict[str, float] | None = None
    observed_arm_positions: dict[str, float] = {}
    last_pressed: set[str] = set()
    next_observation_poll_t = 0.0
    base_ramp = KeyboardBaseRamp()

    try:
        print(f"Connecting to LeKiwi host at {args.remote_host}", flush=True)
        robot.connect()
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

        if not keyboard_connected and not leader_connected:
            raise SystemExit(
                "Neither keyboard capture nor the leader arm started. On macOS, grant Input Monitoring/Accessibility access to the terminal/app running this process, or connect the leader arm."
            )

        arm_targets = seed_arm_targets(robot)
        observed_arm_positions = dict(arm_targets)
        direction_limiter = JointDirectionLimiter(position_limits)
        direction_limiter.seed(observed_arm_positions)
        next_observation_poll_t = time.perf_counter()
        print_controls(leader_connected, mode_state.mode)

        while keyboard.is_connected or leader_connected:
            loop_start_t = time.perf_counter()
            pressed = keyboard.get_pressed() if keyboard.is_connected else set()
            newly_pressed = pressed - last_pressed
            last_pressed = pressed
            dt_s = max(loop_start_t - last_loop_t, 0.0)
            last_loop_t = loop_start_t
            if mode_state.update(newly_pressed):
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
                        pressed,
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

            if leader_arm_action is not None and not keyboard_arm_active(pressed):
                arm_targets.update(leader_arm_action)
            else:
                arm_targets = update_arm_targets(
                    arm_targets,
                    observed_arm_positions,
                    pressed,
                    newly_pressed,
                    dt_s,
                    position_limits,
                    direction_limiter,
                )

            linear_speed, turn_speed = mode_state.speed_pair(
                drive_linear_speed=drive_linear_speed,
                drive_turn_speed=drive_turn_speed,
                ecu_linear_speed=ecu_linear_speed,
                ecu_turn_speed=ecu_turn_speed,
            )
            base_action = base_ramp.update(
                pressed,
                linear_speed=linear_speed,
                turn_speed=turn_speed,
                linear_speed_limit=linear_speed,
                turn_speed_limit=turn_speed,
                now_s=loop_start_t,
            )
            action = {
                **arm_targets,
                **base_action,
                ACTION_COMMAND_SOURCE_KEY: COMMAND_SOURCE_KEYBOARD,
                VEX_CONTROL_MODE_KEY: mode_state.mode,
            }
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

            robot.send_action(action)
            loop_idx += 1
            precise_sleep(max(1.0 / args.fps - (time.perf_counter() - loop_start_t), 0.0))
    except KeyboardInterrupt:
        print("\nStopping keyboard backup teleop.", flush=True)
    finally:
        if arm_targets is not None and robot.is_connected:
            try:
                robot.send_action(
                    {
                        **arm_targets,
                        "x.vel": 0.0,
                        "y.vel": 0.0,
                        "theta.vel": 0.0,
                    }
                )
            except Exception:
                pass
        keyboard.disconnect()
        if leader is not None:
            disconnect_device_safely(leader, "leader arm")
        if robot.is_connected:
            robot.disconnect()


if __name__ == "__main__":
    main()
