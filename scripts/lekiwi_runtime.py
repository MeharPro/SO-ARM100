#!/usr/bin/env python3

import argparse
import json
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


def wrap_degrees(value: float) -> float:
    return ((float(value) + 180.0) % 360.0) - 180.0


def normalize_arm_action(action: dict[str, float]) -> dict[str, float]:
    normalized = dict(action)
    wrist_key = "arm_wrist_roll.pos"
    if wrist_key in normalized:
        normalized[wrist_key] = wrap_degrees(normalized[wrist_key])
    return normalized


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
