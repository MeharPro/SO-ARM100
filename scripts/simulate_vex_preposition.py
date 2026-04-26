#!/usr/bin/env python3

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any

SCRIPTS_DIR = Path(__file__).resolve().parent
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

import lekiwi_sensor_replay as sensor_replay
from lekiwi_sensor_replay import (
    GYRO_ROTATION_KEY,
    SENSOR_PREPOSITION_TIMEOUT_S,
    ULTRASONIC_X_KEY,
    ULTRASONIC_Y_KEY,
    VEX_POSE_EPOCH_KEY,
    SensorAwareReplayState,
    preposition_vex_base_to_recorded_state,
)


def build_sample() -> dict[str, Any]:
    return {
        "t_s": 0.0,
        "state": {
            ULTRASONIC_X_KEY: 1.0,
            ULTRASONIC_Y_KEY: 2.0,
            GYRO_ROTATION_KEY: 0.0,
            VEX_POSE_EPOCH_KEY: 1,
            "x.vel": 0.0,
            "y.vel": 0.0,
            "theta.vel": 0.0,
            "vex_front_right.pos": 0.0,
            "vex_front_left.pos": 0.0,
            "vex_rear_right.pos": 0.0,
            "vex_rear_left.pos": 0.0,
        },
    }


class ObservationReader:
    def __init__(self, state: dict[str, float]) -> None:
        self.state = state

    def get_observation(self) -> dict[str, float]:
        return {}

    def get_sensor_status_snapshot(self) -> dict[str, dict[str, float | str]]:
        return {
            ULTRASONIC_X_KEY: {"state": "online", "value": self.state["x"]},
            ULTRASONIC_Y_KEY: {"state": "online", "value": self.state["y"]},
        }


class SimulatedVexBridge:
    def __init__(self, state: dict[str, float], *, y_sign: float = -1.0, stuck: bool = False) -> None:
        self.state = state
        self.y_sign = y_sign
        self.stuck = stuck
        self.commands: list[dict[str, float]] = []
        self.motion_ttls_ms: list[int] = []

    def merge_observation(self, observation: dict[str, float]) -> dict[str, float]:
        return dict(observation)

    def gyro_status_snapshot(self) -> dict[str, float | str]:
        return {
            "state": "online",
            "value": self.state["heading"],
            "pose_epoch": self.state["pose_epoch"],
        }

    def send_motion(self, motion: dict[str, float], ttl_ms: int | None = None) -> bool:
        command = {
            "x.vel": float(motion.get("x.vel", 0.0) or 0.0),
            "y.vel": float(motion.get("y.vel", 0.0) or 0.0),
            "theta.vel": float(motion.get("theta.vel", 0.0) or 0.0),
        }
        self.commands.append(command)
        effective_ttl_ms = max(int(ttl_ms or sensor_replay.PREPOSITION_COMMAND_TTL_MS), 20)
        self.motion_ttls_ms.append(effective_ttl_ms)
        if not self.stuck:
            dt_s = effective_ttl_ms / 1000.0
            self.state["heading"] += command["theta.vel"] * dt_s
            self.state["x"] -= command["x.vel"] * dt_s
            self.state["y"] += self.y_sign * command["y.vel"] * dt_s
        return True

    def send_hold(self, ttl_ms: int = 1200) -> bool:
        self.commands.append({"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0})
        return True


def run_case(
    name: str,
    *,
    initial_y_m: float,
    y_sign: float = -1.0,
    stuck: bool = False,
) -> dict[str, Any]:
    sample = build_sample()
    live_state = {"x": 1.0, "y": initial_y_m, "heading": 0.0, "pose_epoch": 1}
    replay_state = SensorAwareReplayState(
        [sample],
        replay_mode="drive",
        speed=1.0,
        control_config={"tuning": {"maxLinearSpeedMps": 0.35, "maxTurnSpeedDps": 90.0}},
    )
    bridge = SimulatedVexBridge(live_state, y_sign=y_sign, stuck=stuck)
    reader = ObservationReader(live_state)
    original_sleep = sensor_replay.time.sleep
    sensor_replay.time.sleep = lambda _seconds: None
    try:
        aligned = preposition_vex_base_to_recorded_state(
            bridge,
            reader,
            replay_state,
            sample["state"],
            timeout_s=SENSOR_PREPOSITION_TIMEOUT_S,
            settle_s=0.0,
        )
    finally:
        sensor_replay.time.sleep = original_sleep
    return {
        "case": name,
        "aligned": bool(aligned),
        "reason": getattr(aligned, "reason", None),
        "axis": getattr(aligned, "axis", None),
        "detail": getattr(aligned, "detail", None),
        "commands": len(bridge.commands),
        "motion_commands": len(bridge.motion_ttls_ms),
        "first_motion_ttl_ms": bridge.motion_ttls_ms[0] if bridge.motion_ttls_ms else None,
        "final": {key: round(value, 4) for key, value in live_state.items()},
        "last_command": bridge.commands[-1] if bridge.commands else None,
    }


def main() -> None:
    results = [
        run_case("too-far-correct-sign", initial_y_m=2.12, y_sign=-1.0),
        run_case("too-close-correct-sign", initial_y_m=1.88, y_sign=-1.0),
        run_case("wrong-y-sign", initial_y_m=2.12, y_sign=1.0),
        run_case("stuck-ultrasonic", initial_y_m=2.12, stuck=True),
    ]
    print(json.dumps(results, indent=2, sort_keys=True))
    expected = {
        "too-far-correct-sign": True,
        "too-close-correct-sign": True,
        "wrong-y-sign": False,
        "stuck-ultrasonic": False,
    }
    if any(result["aligned"] is not expected[result["case"]] for result in results):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
