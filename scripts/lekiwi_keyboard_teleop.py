#!/usr/bin/env python3

from __future__ import annotations

import argparse
import os
import time
from dataclasses import dataclass

from lekiwi_runtime import (
    ACTION_COMMAND_SOURCE_KEY,
    COMMAND_SOURCE_KEYBOARD,
    parse_position_limits_json,
)
from lerobot.robots.lekiwi import LeKiwiClient, LeKiwiClientConfig
from lerobot.utils.robot_utils import precise_sleep

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
    ("x.vel", ("j",), ("l",)),
    ("y.vel", ("i",), ("k",)),
    ("theta.vel", ("u",), ("o",)),
)
CONTROL_KEYS = {
    "esc",
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
    parser.add_argument("--base-linear-speed", type=float, default=0.2)
    parser.add_argument("--base-turn-speed", type=float, default=60.0)
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
) -> dict[str, float]:
    return {
        "x.vel": axis_from_keys(pressed, ("j",), ("l",)) * linear_speed,
        "y.vel": axis_from_keys(pressed, ("i",), ("k",)) * linear_speed,
        "theta.vel": axis_from_keys(pressed, ("u",), ("o",)) * turn_speed,
    }


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


def print_controls() -> None:
    print("Keyboard backup teleop is live.", flush=True)
    print(
        "Arm: Q/A shoulder pan, W/S shoulder lift, E/D elbow, "
        "R/F wrist flex (also Y/H), T/G wrist roll, Z/X gripper (also C/V and B/N).",
        flush=True,
    )
    print("Base: I/K forward-back (Y), J/L left-right (X), U/O turn. Press Esc to stop keyboard control.", flush=True)


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

    loop_idx = 0
    last_loop_t = time.perf_counter()
    arm_targets: dict[str, float] | None = None
    observed_arm_positions: dict[str, float] = {}
    last_pressed: set[str] = set()
    next_observation_poll_t = 0.0

    try:
        print(f"Connecting to LeKiwi host at {args.remote_host}", flush=True)
        robot.connect()
        keyboard.connect()
        if not keyboard.is_connected:
            raise SystemExit(
                "Keyboard capture did not start. On macOS, grant Input Monitoring/Accessibility access to the terminal or app running this process."
            )

        arm_targets = seed_arm_targets(robot)
        observed_arm_positions = dict(arm_targets)
        direction_limiter = JointDirectionLimiter(position_limits)
        direction_limiter.seed(observed_arm_positions)
        next_observation_poll_t = time.perf_counter()
        print_controls()

        while keyboard.is_connected:
            loop_start_t = time.perf_counter()
            pressed = keyboard.get_pressed()
            newly_pressed = pressed - last_pressed
            last_pressed = pressed
            dt_s = max(loop_start_t - last_loop_t, 0.0)
            last_loop_t = loop_start_t

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

            arm_targets = update_arm_targets(
                arm_targets,
                observed_arm_positions,
                pressed,
                newly_pressed,
                dt_s,
                position_limits,
                direction_limiter,
            )
            base_action = build_base_action(
                pressed,
                linear_speed=args.base_linear_speed,
                turn_speed=args.base_turn_speed,
            )
            action = {
                **arm_targets,
                **base_action,
                ACTION_COMMAND_SOURCE_KEY: COMMAND_SOURCE_KEYBOARD,
            }
            arm_targets = {
                joint: float(action[joint])
                for joint, *_rest in ARM_BINDINGS
            }

            if loop_idx % args.print_every == 0:
                active_keys = "".join(sorted(pressed)) or "-"
                print(f"{summarize_action(action)} | keys={active_keys}", flush=True)

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
        if robot.is_connected:
            robot.disconnect()


if __name__ == "__main__":
    main()
