#!/usr/bin/env python3

from __future__ import annotations

import argparse
import glob
import hashlib
import json
import logging
import subprocess
import time
from pathlib import Path
from typing import Any

from lekiwi_runtime import BASE_STATE_KEYS, iso_timestamp_from_epoch

try:
    import serial
except Exception:  # pragma: no cover - resolved at runtime on the Pi
    serial = None

VEX_PORT_GLOB = "/dev/serial/by-id/*VEX_Robotics_V5_Brain*"
VEX_PREFERRED_SUFFIXES = ("if02", "if00")
VEX_COMM_SUFFIXES = ("if00", "if01")
DEFAULT_COMMAND_TIMEOUT_S = 0.35
DEFAULT_TELEMETRY_SLOT = 8
DEFAULT_REPLAY_SLOT = 7
REQUIRED_VEX_MIXER_VERSION = 8
VEX_IDLE_MOTION_EPSILON = 1e-4
VEX_CONTROL_MODE_KEY = "__vex_control_mode__"
VEX_DRIVE_CONTROL_MODE = "drive"
VEX_ECU_CONTROL_MODE = "ecu"
KEYBOARD_BASE_LINEAR_SPEED_LIMIT_MPS = 0.06
KEYBOARD_BASE_TURN_SPEED_LIMIT_DPS = 18.0
DEFAULT_PROGRAM_CACHE_DIR = "~/.lekiwi-vex/programs"
DEFAULT_REPLAY_CACHE_DIR = "~/.lekiwi-vex/replays"
DEFAULT_VEXCOM_PATH_GLOBS = (
    "~/vex_tools/vexcom",
    "~/.config/Code/User/globalStorage/vexrobotics.vexcode/tools/vexcom/*/linux-arm64/vexcom",
)
VEX_STATE_ALIASES = {
    "x.vel": ("x.vel", "x_vel", "vx"),
    "y.vel": ("y.vel", "y_vel", "vy"),
    "theta.vel": ("theta.vel", "theta_vel", "omega", "omega_deg_s"),
}
VEX_MOTOR_STATE_KEYS = (
    "vex_front_right.pos",
    "vex_front_left.pos",
    "vex_rear_right.pos",
    "vex_rear_left.pos",
)
VEX_INERTIAL_STATE_KEYS = (
    "vex_inertial_rotation.deg",
    "vex_inertial_heading.deg",
    "vex_inertial_rate_z.dps",
)
VEX_POSE_STATE_KEYS = (
    "vex_pose_epoch",
    "vex_mixer_version",
)
VEX_MOTOR_STATE_ALIASES = {
    "vex_front_right.pos": ("vex_front_right.pos", "frontRight.pos", "front_right.pos"),
    "vex_front_left.pos": ("vex_front_left.pos", "frontLeft.pos", "front_left.pos"),
    "vex_rear_right.pos": ("vex_rear_right.pos", "rearRight.pos", "rear_right.pos"),
    "vex_rear_left.pos": ("vex_rear_left.pos", "rearLeft.pos", "rear_left.pos"),
}
VEX_INERTIAL_STATE_ALIASES = {
    "vex_inertial_rotation.deg": (
        "vex_inertial_rotation.deg",
        "vexInertial.rotationDeg",
        "inertial.rotation",
    ),
    "vex_inertial_heading.deg": (
        "vex_inertial_heading.deg",
        "vexInertial.headingDeg",
        "inertial.heading",
    ),
    "vex_inertial_rate_z.dps": (
        "vex_inertial_rate_z.dps",
        "vexInertial.rateZDps",
        "inertial.rate_z",
    ),
}
VEX_POSE_STATE_ALIASES = {
    "vex_pose_epoch": ("vex_pose_epoch", "vexPose.epoch", "pose.epoch"),
    "vex_mixer_version": ("vex_mixer_version", "vexMixer.version", "mixer.version"),
}
VALID_VEX_AXES = {"axis1", "axis2", "axis3", "axis4"}
DEFAULT_VEX_CONTROL_CONFIG = {
    "inertial": {
        "port": 4,
    },
    "motors": {
        "frontRight": {"port": 1, "reversed": True},
        "frontLeft": {"port": 2, "reversed": False},
        "rearRight": {"port": 9, "reversed": True},
        "rearLeft": {"port": 10, "reversed": False},
    },
    "controls": {
        "forwardAxis": "axis3",
        "strafeAxis": "axis4",
        "turnAxis": "axis1",
        "invertForward": False,
        "invertStrafe": True,
        "invertTurn": False,
    },
    "tuning": {
        "deadbandPercent": 5,
        "maxLinearSpeedMps": 0.35,
        "maxTurnSpeedDps": 90.0,
        "ecuLinearSpeedMps": KEYBOARD_BASE_LINEAR_SPEED_LIMIT_MPS,
        "ecuTurnSpeedDps": KEYBOARD_BASE_TURN_SPEED_LIMIT_DPS,
    },
    "keyboardCalibration": {
        "xSign": 1,
        "ySign": 1,
        "thetaSign": 1,
        "calibratedAtIso": None,
        "notes": "",
    },
    "manualIdleStoppingMode": "hold",
}


def _numeric_base_motion_value(motion: dict[str, Any], key: str) -> float:
    value = motion.get(key, 0.0)
    return float(value) if isinstance(value, (int, float)) else 0.0


def base_motion_is_idle(motion: dict[str, Any], *, epsilon: float = VEX_IDLE_MOTION_EPSILON) -> bool:
    return all(abs(_numeric_base_motion_value(motion, key)) <= epsilon for key in BASE_STATE_KEYS)


class VexBaseIdleHoldController:
    def __init__(
        self,
        *,
        epsilon: float = VEX_IDLE_MOTION_EPSILON,
        hold_ttl_ms: int = 1200,
    ) -> None:
        self.epsilon = max(float(epsilon), 0.0)
        self.hold_ttl_ms = max(int(hold_ttl_ms), 50)
        self._holding = False
        self._last_control_mode: str | None = None

    def reset(self) -> None:
        self._holding = False

    def mark_holding(self) -> None:
        self._holding = True

    def _send_control_mode_if_requested(
        self,
        vex_base_bridge: "VexBaseBridge",
        motion: dict[str, Any],
    ) -> bool:
        requested_mode = str(motion.get(VEX_CONTROL_MODE_KEY, "")).strip().lower()
        if requested_mode not in {VEX_DRIVE_CONTROL_MODE, VEX_ECU_CONTROL_MODE}:
            return True
        if requested_mode == self._last_control_mode:
            return True

        send_control_mode = getattr(vex_base_bridge, "send_control_mode", None)
        if not callable(send_control_mode):
            return False

        sent = bool(send_control_mode(requested_mode))
        if sent:
            self._last_control_mode = requested_mode
        return sent

    def send(self, vex_base_bridge: "VexBaseBridge", motion: dict[str, Any], ttl_ms: int | None = None) -> bool:
        mode_sent = self._send_control_mode_if_requested(vex_base_bridge, motion)
        if base_motion_is_idle(motion, epsilon=self.epsilon):
            if self._holding:
                return mode_sent
            sent = vex_base_bridge.send_hold(ttl_ms=self.hold_ttl_ms)
            if sent:
                self.mark_holding()
            return mode_sent and sent

        self.reset()
        return mode_sent and vex_base_bridge.send_motion(motion, ttl_ms=ttl_ms)


def add_vex_base_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--vex-base-port",
        default="auto",
        help="VEX V5 Brain console serial path, 'auto', or 'off'.",
    )
    parser.add_argument("--vex-base-baudrate", type=int, default=115200)
    parser.add_argument("--vex-base-stale-after-s", type=float, default=0.35)
    parser.add_argument("--vex-base-command-timeout-s", type=float, default=DEFAULT_COMMAND_TIMEOUT_S)
    parser.add_argument("--vex-telemetry-slot", type=int, default=DEFAULT_TELEMETRY_SLOT)
    parser.add_argument("--vex-telemetry-program-name", default="Base Telemetry")
    parser.add_argument("--vex-replay-slot", type=int, default=DEFAULT_REPLAY_SLOT)
    parser.add_argument("--vex-program-cache-dir", default=DEFAULT_PROGRAM_CACHE_DIR)
    parser.add_argument("--vex-replay-cache-dir", default=DEFAULT_REPLAY_CACHE_DIR)
    parser.add_argument("--vex-vexcom-path", default="auto")
    parser.add_argument("--vex-control-config-json", default="{}")


def _sort_vex_ports(paths: list[str]) -> list[str]:
    return _sort_ports_by_suffixes(paths, VEX_PREFERRED_SUFFIXES)


def _sort_ports_by_suffixes(paths: list[str], suffixes: tuple[str, ...]) -> list[str]:
    def rank(path: str) -> tuple[int, str]:
        for index, suffix in enumerate(suffixes):
            if suffix in Path(path).name:
                return (index, path)
        return (len(suffixes), path)

    return sorted(paths, key=rank)


def detect_vex_port(requested_port: str) -> str | None:
    normalized = requested_port.strip() if requested_port else "auto"
    if normalized.lower() == "off":
        return None
    if normalized.lower() != "auto":
        return normalized

    candidates = _sort_vex_ports(glob.glob(VEX_PORT_GLOB))
    if not candidates:
        return None
    return candidates[0]


def detect_vex_comm_port() -> str | None:
    candidates = _sort_ports_by_suffixes(glob.glob(VEX_PORT_GLOB), VEX_COMM_SUFFIXES)
    if not candidates:
        return None
    return candidates[0]


def detect_vexcom_path(requested_path: str) -> str | None:
    normalized = requested_path.strip() if requested_path else "auto"
    if normalized.lower() == "off":
        return None
    if normalized.lower() != "auto":
        candidate = Path(normalized).expanduser()
        if candidate.exists():
            return str(candidate)
        return None

    for pattern in DEFAULT_VEXCOM_PATH_GLOBS:
        for candidate in glob.glob(str(Path(pattern).expanduser())):
            path = Path(candidate)
            if path.exists():
                return str(path)
    return None


def _extract_numeric(payload: dict[str, Any], aliases: tuple[str, ...]) -> float:
    for key in aliases:
        value = payload.get(key)
        if isinstance(value, (int, float)):
            return float(value)
    return 0.0


def _extract_optional_numeric(payload: dict[str, Any], aliases: tuple[str, ...]) -> float | None:
    for key in aliases:
        value = payload.get(key)
        if isinstance(value, (int, float)):
            return float(value)
    return None


def normalize_vex_control_config(raw: dict[str, Any] | None) -> dict[str, Any]:
    payload = raw if isinstance(raw, dict) else {}
    inertial = payload.get("inertial") if isinstance(payload.get("inertial"), dict) else {}
    motors = payload.get("motors") if isinstance(payload.get("motors"), dict) else {}
    controls = payload.get("controls") if isinstance(payload.get("controls"), dict) else {}
    tuning = payload.get("tuning") if isinstance(payload.get("tuning"), dict) else {}
    keyboard_calibration = payload.get("keyboardCalibration") if isinstance(payload.get("keyboardCalibration"), dict) else {}

    def port_value(group: dict[str, Any], key: str, fallback: int) -> int:
        candidate = group.get(key)
        if isinstance(candidate, dict):
            candidate = candidate.get("port", fallback)
        try:
            numeric = int(candidate)
        except Exception:
            numeric = fallback
        return max(1, min(21, numeric))

    def reversed_value(group: dict[str, Any], key: str, fallback: bool) -> bool:
        candidate = group.get(key)
        if isinstance(candidate, dict):
            candidate = candidate.get("reversed", fallback)
        return bool(candidate)

    def axis_value(key: str, fallback: str) -> str:
        candidate = controls.get(key, fallback)
        if isinstance(candidate, str) and candidate in VALID_VEX_AXES:
            return candidate
        return fallback

    def bool_value(group: dict[str, Any], key: str, fallback: bool) -> bool:
        candidate = group.get(key, fallback)
        return bool(candidate)

    def sign_value(group: dict[str, Any], key: str, fallback: int) -> int:
        return -1 if group.get(key, fallback) in {-1, -1.0, "-1"} else 1

    def stopping_mode_value(value: Any, fallback: str) -> str:
        return value if value in {"hold", "brake", "coast"} else fallback

    def float_value(group: dict[str, Any], key: str, fallback: float, low: float, high: float) -> float:
        candidate = group.get(key, fallback)
        try:
            numeric = float(candidate)
        except Exception:
            numeric = fallback
        return max(low, min(high, numeric))

    defaults = DEFAULT_VEX_CONTROL_CONFIG
    return {
        "inertial": {
            "port": port_value(inertial, "port", defaults["inertial"]["port"]),
        },
        "motors": {
            "frontRight": {
                "port": port_value(motors, "frontRight", defaults["motors"]["frontRight"]["port"]),
                "reversed": reversed_value(motors, "frontRight", defaults["motors"]["frontRight"]["reversed"]),
            },
            "frontLeft": {
                "port": port_value(motors, "frontLeft", defaults["motors"]["frontLeft"]["port"]),
                "reversed": reversed_value(motors, "frontLeft", defaults["motors"]["frontLeft"]["reversed"]),
            },
            "rearRight": {
                "port": port_value(motors, "rearRight", defaults["motors"]["rearRight"]["port"]),
                "reversed": reversed_value(motors, "rearRight", defaults["motors"]["rearRight"]["reversed"]),
            },
            "rearLeft": {
                "port": port_value(motors, "rearLeft", defaults["motors"]["rearLeft"]["port"]),
                "reversed": reversed_value(motors, "rearLeft", defaults["motors"]["rearLeft"]["reversed"]),
            },
        },
        "controls": {
            "forwardAxis": axis_value("forwardAxis", defaults["controls"]["forwardAxis"]),
            "strafeAxis": axis_value("strafeAxis", defaults["controls"]["strafeAxis"]),
            "turnAxis": axis_value("turnAxis", defaults["controls"]["turnAxis"]),
            "invertForward": bool_value(controls, "invertForward", defaults["controls"]["invertForward"]),
            "invertStrafe": bool_value(controls, "invertStrafe", defaults["controls"]["invertStrafe"]),
            "invertTurn": bool_value(controls, "invertTurn", defaults["controls"]["invertTurn"]),
        },
        "tuning": {
            "deadbandPercent": int(
                round(
                    float_value(
                        tuning,
                        "deadbandPercent",
                        defaults["tuning"]["deadbandPercent"],
                        0.0,
                        30.0,
                    )
                )
            ),
            "maxLinearSpeedMps": float_value(
                tuning,
                "maxLinearSpeedMps",
                defaults["tuning"]["maxLinearSpeedMps"],
                0.05,
                2.0,
            ),
            "maxTurnSpeedDps": float_value(
                tuning,
                "maxTurnSpeedDps",
                defaults["tuning"]["maxTurnSpeedDps"],
                5.0,
                360.0,
            ),
            "ecuLinearSpeedMps": float_value(
                tuning,
                "ecuLinearSpeedMps",
                defaults["tuning"]["ecuLinearSpeedMps"],
                0.01,
                2.0,
            ),
            "ecuTurnSpeedDps": float_value(
                tuning,
                "ecuTurnSpeedDps",
                defaults["tuning"]["ecuTurnSpeedDps"],
                1.0,
                360.0,
            ),
        },
        "keyboardCalibration": {
            "xSign": sign_value(keyboard_calibration, "xSign", defaults["keyboardCalibration"]["xSign"]),
            "ySign": sign_value(keyboard_calibration, "ySign", defaults["keyboardCalibration"]["ySign"]),
            "thetaSign": sign_value(keyboard_calibration, "thetaSign", defaults["keyboardCalibration"]["thetaSign"]),
            "calibratedAtIso": keyboard_calibration.get("calibratedAtIso")
            if isinstance(keyboard_calibration.get("calibratedAtIso"), str)
            else defaults["keyboardCalibration"]["calibratedAtIso"],
            "notes": keyboard_calibration.get("notes")
            if isinstance(keyboard_calibration.get("notes"), str)
            else defaults["keyboardCalibration"]["notes"],
        },
        "manualIdleStoppingMode": stopping_mode_value(
            payload.get("manualIdleStoppingMode"),
            defaults["manualIdleStoppingMode"],
        ),
    }


def _motor_definition_lines(config: dict[str, Any]) -> str:
    motors = config["motors"]
    return "\n".join(
        [
            f'front_right = Motor(Ports.PORT{motors["frontRight"]["port"]}, GearSetting.RATIO_18_1, {str(bool(motors["frontRight"]["reversed"]))})',
            f'front_left = Motor(Ports.PORT{motors["frontLeft"]["port"]}, GearSetting.RATIO_18_1, {str(bool(motors["frontLeft"]["reversed"]))})',
            f'rear_right = Motor(Ports.PORT{motors["rearRight"]["port"]}, GearSetting.RATIO_18_1, {str(bool(motors["rearRight"]["reversed"]))})',
            f'rear_left = Motor(Ports.PORT{motors["rearLeft"]["port"]}, GearSetting.RATIO_18_1, {str(bool(motors["rearLeft"]["reversed"]))})',
        ]
    )


def _axis_expression(axis_name: str, inverted: bool) -> str:
    expr = f"controller_1.{axis_name}.position()"
    return f"-({expr})" if inverted else expr


def _inertial_definition_lines(config: dict[str, Any]) -> str:
    inertial = config["inertial"]
    return "\n".join(
        [
            f'inertial_1 = Inertial(Ports.PORT{inertial["port"]})',
            "",
            "def inertial_available():",
            "    return inertial_1.installed()",
            "",
            "def initialize_inertial():",
            "    if not inertial_available():",
            "        return",
            "    inertial_1.calibrate()",
            "    while inertial_1.is_calibrating():",
            "        wait(25, MSEC)",
            "    inertial_1.reset_rotation()",
            "    inertial_1.reset_heading()",
            "",
            "def read_inertial_state():",
            "    if not inertial_available():",
            '        return {"vex_inertial_rotation.deg": 0.0, "vex_inertial_heading.deg": 0.0, "vex_inertial_rate_z.dps": 0.0}',
            "    return {",
            '        "vex_inertial_rotation.deg": inertial_1.rotation(DEGREES),',
            '        "vex_inertial_heading.deg": inertial_1.heading(DEGREES),',
            '        "vex_inertial_rate_z.dps": inertial_1.gyro_rate(AxisType.ZAXIS, VelocityUnits.DPS),',
            "    }",
        ]
    )


def build_vex_telemetry_program_source(control_config: dict[str, Any] | None) -> str:
    config = normalize_vex_control_config(control_config)
    tuning = config["tuning"]
    controls = config["controls"]
    motor_lines = _motor_definition_lines(config)
    inertial_lines = _inertial_definition_lines(config)
    forward_expr = _axis_expression(controls["forwardAxis"], bool(controls["invertForward"]))
    strafe_expr = _axis_expression(controls["strafeAxis"], bool(controls["invertStrafe"]))
    turn_expr = _axis_expression(controls["turnAxis"], bool(controls["invertTurn"]))
    return f"""#vex:disable=repl
from vex import *
import sys

brain = Brain()
controller_1 = Controller(PRIMARY)

{motor_lines}
{inertial_lines}

MAX_LINEAR_SPEED_MPS = {float(tuning["maxLinearSpeedMps"]):.4f}
MAX_TURN_SPEED_DPS = {float(tuning["maxTurnSpeedDps"]):.4f}
ECU_LINEAR_SPEED_MPS = {float(tuning["ecuLinearSpeedMps"]):.4f}
ECU_TURN_SPEED_DPS = {float(tuning["ecuTurnSpeedDps"]):.4f}
VEX_MIXER_VERSION = {REQUIRED_VEX_MIXER_VERSION}
DEADBAND_PERCENT = {int(tuning["deadbandPercent"])}
TELEMETRY_INTERVAL_MS = 50
SCREEN_KEEPALIVE_INTERVAL_MS = 1000
REMOTE_COMMAND_MIN_TTL_MS = 50
ECU_IDLE_TOLERANCE_DEG = 0.75
ECU_COMMAND_TOLERANCE_DEG = 0.25
ECU_MIN_SPEED_DPS = 30.0
ECU_MAX_SPEED_DPS = 360.0
REMOTE_HOLD_TTL_MS = 1200

MOTOR_ORDER = (
    ("vex_front_right.pos", front_right),
    ("vex_front_left.pos", front_left),
    ("vex_rear_right.pos", rear_right),
    ("vex_rear_left.pos", rear_left),
)

remote_takeover = True
remote_mode = "hold"
remote_motion = {{"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}}
remote_targets = {{}}
remote_dt_ms = 20
remote_expires_ms = 0
last_targets = {{}}
pose_epoch = 0
controller_control_mode = "{VEX_DRIVE_CONTROL_MODE}"
serial_command_lines = []
serial_reader_thread = None


def clamp(value, low, high):
    return max(low, min(high, value))


def apply_deadband(value):
    if abs(value) < DEADBAND_PERCENT:
        return 0
    return value


def zero_motion():
    return {{"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}}


def motion_is_active(motion):
    return (
        abs(motion["x.vel"]) > 0.0001
        or abs(motion["y.vel"]) > 0.0001
        or abs(motion["theta.vel"]) > 0.0001
    )


def controller_motion():
    linear_speed = ECU_LINEAR_SPEED_MPS if controller_control_mode == "{VEX_ECU_CONTROL_MODE}" else MAX_LINEAR_SPEED_MPS
    turn_speed = ECU_TURN_SPEED_DPS if controller_control_mode == "{VEX_ECU_CONTROL_MODE}" else MAX_TURN_SPEED_DPS
    forward_pct = apply_deadband({forward_expr})
    strafe_pct = apply_deadband({strafe_expr})
    turn_pct = apply_deadband({turn_expr})
    return {{
        "x.vel": (strafe_pct / 100.0) * linear_speed,
        "y.vel": (forward_pct / 100.0) * linear_speed,
        "theta.vel": (turn_pct / 100.0) * turn_speed,
    }}


def motion_to_percent(motion):
    return (
        clamp((motion["y.vel"] / MAX_LINEAR_SPEED_MPS) * 100.0, -100.0, 100.0),
        clamp((motion["x.vel"] / MAX_LINEAR_SPEED_MPS) * 100.0, -100.0, 100.0),
        clamp((motion["theta.vel"] / MAX_TURN_SPEED_DPS) * 100.0, -100.0, 100.0),
    )


def spin_drive_motor(motor, percent):
    if abs(percent) <= 0.01:
        motor.stop(HOLD)
        return
    motor.set_velocity(abs(percent), PERCENT)
    motor.spin(FORWARD if percent >= 0 else REVERSE)


def apply_drive_motion(motion):
    forward_pct, strafe_pct, turn_pct = motion_to_percent(motion)
    front_left_pct = forward_pct + turn_pct + strafe_pct
    rear_left_pct = forward_pct + turn_pct - strafe_pct
    front_right_pct = forward_pct - turn_pct - strafe_pct
    rear_right_pct = forward_pct - turn_pct + strafe_pct

    spin_drive_motor(front_left, front_left_pct)
    spin_drive_motor(rear_left, rear_left_pct)
    spin_drive_motor(front_right, front_right_pct)
    spin_drive_motor(rear_right, rear_right_pct)


def read_motor_positions():
    return {{
        "vex_front_right.pos": front_right.position(DEGREES),
        "vex_front_left.pos": front_left.position(DEGREES),
        "vex_rear_right.pos": rear_right.position(DEGREES),
        "vex_rear_left.pos": rear_left.position(DEGREES),
    }}


def hold_drive():
    front_left.stop(HOLD)
    rear_left.stop(HOLD)
    front_right.stop(HOLD)
    rear_right.stop(HOLD)


def parse_float(token, fallback=0.0):
    try:
        return float(token)
    except Exception:
        return fallback


def parse_int(token, fallback=0):
    try:
        return int(token)
    except Exception:
        return fallback


def clear_remote_command(release_controller=False):
    global remote_takeover
    global remote_mode
    global remote_motion
    global remote_targets
    global remote_dt_ms
    global remote_expires_ms
    global last_targets

    if release_controller:
        remote_takeover = False
    remote_mode = ""
    remote_motion = zero_motion()
    remote_targets = {{}}
    remote_dt_ms = 20
    remote_expires_ms = 0
    last_targets = {{}}


def set_pose_origin():
    global pose_epoch
    if inertial_available():
        inertial_1.reset_rotation()
        inertial_1.reset_heading()
    pose_epoch += 1


def handle_command_line(line):
    global remote_takeover
    global remote_mode
    global remote_motion
    global remote_targets
    global remote_dt_ms
    global remote_expires_ms
    global last_targets
    global controller_control_mode

    if not line:
        return

    text = line.strip()
    if not text.startswith("!"):
        return

    parts = text.split()
    if not parts:
        return

    now_ms = brain.timer.system()
    command = parts[0]
    if command == "!mode" and len(parts) >= 2:
        requested_mode = parts[1].strip().lower()
        if requested_mode in ("{VEX_DRIVE_CONTROL_MODE}", "{VEX_ECU_CONTROL_MODE}"):
            controller_control_mode = requested_mode
        return

    if command == "!release":
        clear_remote_command(release_controller=True)
        return

    if command == "!hold":
        remote_takeover = True
        remote_mode = "hold"
        remote_motion = zero_motion()
        remote_targets = read_motor_positions()
        remote_dt_ms = 20
        remote_expires_ms = now_ms + max(
            parse_int(parts[1], REMOTE_HOLD_TTL_MS) if len(parts) >= 2 else REMOTE_HOLD_TTL_MS,
            REMOTE_COMMAND_MIN_TTL_MS,
        )
        last_targets = dict(remote_targets)
        hold_drive()
        return

    if command == "!origin":
        remote_takeover = True
        remote_mode = "origin"
        remote_motion = zero_motion()
        remote_targets = read_motor_positions()
        remote_dt_ms = 20
        remote_expires_ms = now_ms + max(
            parse_int(parts[1], REMOTE_HOLD_TTL_MS) if len(parts) >= 2 else REMOTE_HOLD_TTL_MS,
            REMOTE_COMMAND_MIN_TTL_MS,
        )
        last_targets = dict(remote_targets)
        set_pose_origin()
        hold_drive()
        return

    if command == "!velocity" and len(parts) >= 5:
        remote_takeover = True
        remote_mode = "velocity"
        remote_motion = {{
            "x.vel": parse_float(parts[1]),
            "y.vel": parse_float(parts[2]),
            "theta.vel": parse_float(parts[3]),
        }}
        remote_targets = {{}}
        remote_dt_ms = 20
        remote_expires_ms = now_ms + max(parse_int(parts[4], 0), REMOTE_COMMAND_MIN_TTL_MS)
        last_targets = {{}}
        return

    if command == "!ecu" and len(parts) >= 10:
        remote_takeover = True
        remote_mode = "ecu"
        remote_targets = {{
            "vex_front_right.pos": parse_float(parts[1]),
            "vex_front_left.pos": parse_float(parts[2]),
            "vex_rear_right.pos": parse_float(parts[3]),
            "vex_rear_left.pos": parse_float(parts[4]),
        }}
        remote_motion = {{
            "x.vel": parse_float(parts[5]),
            "y.vel": parse_float(parts[6]),
            "theta.vel": parse_float(parts[7]),
        }}
        remote_dt_ms = max(parse_int(parts[8], 20), 20)
        remote_expires_ms = now_ms + max(parse_int(parts[9], 0), REMOTE_COMMAND_MIN_TTL_MS)
        return


def remote_command_active(now_ms):
    return remote_takeover and remote_mode != "" and now_ms <= remote_expires_ms


def serial_command_reader():
    serial_buffer = ""
    while True:
        try:
            chunk = sys.stdin.read(1)
        except Exception:
            wait(20, MSEC)
            continue

        if not chunk:
            wait(5, MSEC)
            continue
        if isinstance(chunk, bytes):
            chunk = chunk.decode("utf-8", "ignore")
        serial_buffer += chunk

        while "\\n" in serial_buffer:
            line, serial_buffer = serial_buffer.split("\\n", 1)
            if line:
                serial_command_lines.append(line)


def start_serial_command_reader():
    global serial_reader_thread
    if serial_reader_thread is None:
        serial_reader_thread = Thread(serial_command_reader)


def poll_serial_commands():
    while len(serial_command_lines) > 0:
        handle_command_line(serial_command_lines.pop(0))


def apply_remote_ecu():
    if len(remote_targets) != 4:
        hold_drive()
        return

    positions = read_motor_positions()
    dt_s = max(remote_dt_ms / 1000.0, 0.02)
    for key, motor in MOTOR_ORDER:
        target = remote_targets.get(key)
        if target is None:
            continue

        position_error = target - positions.get(key, 0.0)
        prior_target = last_targets.get(key)
        delta = position_error if prior_target is None else target - prior_target
        if abs(position_error) <= ECU_IDLE_TOLERANCE_DEG and abs(delta) <= ECU_IDLE_TOLERANCE_DEG:
            if motor.is_done():
                motor.stop(HOLD)
            last_targets[key] = target
            continue

        speed_dps = clamp(abs(delta) / dt_s, ECU_MIN_SPEED_DPS, ECU_MAX_SPEED_DPS)
        should_command = (
            prior_target is None
            or abs(prior_target - target) > ECU_COMMAND_TOLERANCE_DEG
            or (abs(position_error) > ECU_IDLE_TOLERANCE_DEG and motor.is_done())
        )
        if should_command:
            motor.set_stopping(HOLD)
            motor.spin_to_position(target, DEGREES, speed_dps, VelocityUnits.DPS, wait=False)
            last_targets[key] = target
        elif motor.is_done():
            motor.stop(HOLD)


def print_motion(motion, source):
    positions = read_motor_positions()
    inertial_state = read_inertial_state()
    theta_vel = inertial_state["vex_inertial_rate_z.dps"] if inertial_available() else motion["theta.vel"]
    print(
        '{{"x.vel":%.4f,"y.vel":%.4f,"theta.vel":%.4f,"vex_front_right.pos":%.4f,"vex_front_left.pos":%.4f,"vex_rear_right.pos":%.4f,"vex_rear_left.pos":%.4f,"vex_inertial_rotation.deg":%.4f,"vex_inertial_heading.deg":%.4f,"vex_inertial_rate_z.dps":%.4f,"vex_pose_epoch":%d,"vex_mixer_version":%d,"vex_control_mode":"%s","source":"%s"}}'
        % (
            motion["x.vel"],
            motion["y.vel"],
            theta_vel,
            positions["vex_front_right.pos"],
            positions["vex_front_left.pos"],
            positions["vex_rear_right.pos"],
            positions["vex_rear_left.pos"],
            inertial_state["vex_inertial_rotation.deg"],
            inertial_state["vex_inertial_heading.deg"],
            inertial_state["vex_inertial_rate_z.dps"],
            pose_epoch,
            VEX_MIXER_VERSION,
            controller_control_mode,
            source,
        )
    )


def display_keepalive(source):
    try:
        brain.screen.clear_screen()
        brain.screen.set_cursor(1, 1)
        brain.screen.print("Base Telemetry ON")
        brain.screen.set_cursor(2, 1)
        brain.screen.print("Mode: " + controller_control_mode)
        brain.screen.set_cursor(3, 1)
        brain.screen.print("Src: " + source)
        brain.screen.set_cursor(4, 1)
        brain.screen.print("Hold power button to shut down")
        brain.screen.set_cursor(5, 1)
        brain.screen.print("Uptime ms: " + str(brain.timer.system()))
    except Exception:
        pass


def main():
    last_telemetry_ms = -TELEMETRY_INTERVAL_MS
    last_screen_ms = -SCREEN_KEEPALIVE_INTERVAL_MS
    initialize_inertial()
    start_serial_command_reader()
    wait(30, MSEC)
    print("\\033[2J")

    while True:
        now_ms = brain.timer.system()
        poll_serial_commands()
        if remote_takeover:
            if remote_command_active(now_ms):
                active_motion = remote_motion
                if remote_mode == "ecu":
                    apply_remote_ecu()
                    source = "pi-ecu"
                elif remote_mode == "hold":
                    hold_drive()
                    source = "pi-hold"
                elif remote_mode == "origin":
                    hold_drive()
                    source = "pi-origin"
                else:
                    apply_drive_motion(active_motion)
                    source = "pi-drive"
            else:
                active_motion = controller_motion()
                if motion_is_active(active_motion):
                    clear_remote_command(release_controller=True)
                    apply_drive_motion(active_motion)
                    source = "controller"
                elif remote_mode in ("hold", "origin"):
                    active_motion = zero_motion()
                    hold_drive()
                    source = "pi-origin" if remote_mode == "origin" else "pi-hold"
                else:
                    active_motion = zero_motion()
                    hold_drive()
                    source = "pi-timeout"
        else:
            active_motion = controller_motion()
            apply_drive_motion(active_motion)
            source = "controller"

        if now_ms - last_telemetry_ms >= TELEMETRY_INTERVAL_MS:
            print_motion(active_motion, source)
            last_telemetry_ms = now_ms
        if now_ms - last_screen_ms >= SCREEN_KEEPALIVE_INTERVAL_MS:
            display_keepalive(source)
            last_screen_ms = now_ms

        wait(20, MSEC)


main()
"""


class VexBaseBridge:
    def __init__(
        self,
        *,
        requested_port: str,
        baudrate: int,
        stale_after_s: float,
        command_timeout_s: float,
        logger: logging.Logger,
    ) -> None:
        self.requested_port = requested_port
        self.baudrate = int(baudrate)
        self.stale_after_s = max(float(stale_after_s), 0.0)
        self.command_timeout_s = max(float(command_timeout_s), 0.05)
        self.logger = logger
        self.port: str | None = None
        self.serial_handle: Any | None = None
        self.buffer = ""
        self.latest_motion = dict.fromkeys(BASE_STATE_KEYS, 0.0)
        self.latest_extra_state: dict[str, float] = {}
        self.latest_payload: dict[str, Any] | None = None
        self.latest_update_time = 0.0
        self.last_command_sent_at = 0.0
        self.last_read_error_log_at = 0.0
        self.status_message = "VEX base bridge disabled."

    @property
    def enabled(self) -> bool:
        return self.serial_handle is not None

    def connect(self) -> None:
        if serial is None:
            self.status_message = "VEX base bridge unavailable: pyserial is not installed."
            return

        self.close()
        self.port = detect_vex_port(self.requested_port)
        if self.port is None:
            self.status_message = "VEX base bridge idle: no V5 Brain serial port detected."
            return

        try:
            self.serial_handle = serial.Serial(self.port, baudrate=self.baudrate, timeout=0)
        except Exception as exc:
            self.serial_handle = None
            self.status_message = f"VEX base bridge failed to open {self.port}: {exc}"
            return

        self.buffer = ""
        self.latest_motion = dict.fromkeys(BASE_STATE_KEYS, 0.0)
        self.latest_extra_state = {}
        self.latest_payload = None
        self.latest_update_time = 0.0
        self.last_command_sent_at = 0.0
        self.last_read_error_log_at = 0.0
        self.status_message = f"VEX base bridge reading telemetry from {self.port}."
        self.logger.info(self.status_message)

    def close(self) -> None:
        if self.serial_handle is None:
            return
        try:
            self.serial_handle.close()
        finally:
            self.serial_handle = None

    def poll(self) -> dict[str, float]:
        if self.serial_handle is None:
            return dict.fromkeys(BASE_STATE_KEYS, 0.0)

        try:
            available = int(getattr(self.serial_handle, "in_waiting", 0) or 0)
            if available > 0:
                chunk = self.serial_handle.read(available).decode("utf-8", errors="ignore")
                self.buffer += chunk
        except Exception as exc:
            now = time.time()
            if now - self.last_read_error_log_at >= 2.0:
                self.logger.warning("VEX base bridge read failed on %s: %s", self.port, exc)
                self.last_read_error_log_at = now
            self.status_message = f"VEX base bridge lost telemetry on {self.port}: {exc}"
            self.close()
            return self.current_motion()

        while "\n" in self.buffer:
            line, self.buffer = self.buffer.split("\n", 1)
            self._handle_line(line.strip())

        return self.current_motion()

    def current_motion(self) -> dict[str, float]:
        if not self.latest_update_time:
            return dict.fromkeys(BASE_STATE_KEYS, 0.0)
        if self.stale_after_s > 0 and (time.time() - self.latest_update_time) > self.stale_after_s:
            return dict.fromkeys(BASE_STATE_KEYS, 0.0)
        return dict(self.latest_motion)

    def current_state(self) -> dict[str, float]:
        return {
            **self.latest_extra_state,
            **self.current_motion(),
        }

    def gyro_status_snapshot(self) -> dict[str, Any]:
        source = None
        if isinstance(self.latest_payload, dict):
            payload_source = self.latest_payload.get("source")
            if isinstance(payload_source, str) and payload_source.strip():
                source = payload_source.strip()

        base_status = {
            "state": "missing",
            "connected": False,
            "value": None,
            "unit": "deg",
            "pose_epoch": self.latest_extra_state.get("vex_pose_epoch"),
            "updated_at": None,
            "source": source,
            "message": self.status_message,
        }
        if self.serial_handle is None:
            return base_status

        if not self.latest_update_time:
            return {
                **base_status,
                "state": "waiting",
                "connected": True,
                "message": "Waiting for VEX inertial telemetry from the Brain.",
            }

        rotation = self.latest_extra_state.get("vex_inertial_rotation.deg")
        stale_after_s = max(self.stale_after_s * 3.0, 1.0)
        age_s = time.time() - self.latest_update_time
        updated_at = iso_timestamp_from_epoch(self.latest_update_time)

        if not isinstance(rotation, (int, float)):
            return {
                **base_status,
                "state": "stale" if age_s > stale_after_s else "waiting",
                "connected": True,
                "updated_at": updated_at,
                "message": "VEX telemetry is active, but inertial rotation is missing from the stream.",
            }

        if age_s > stale_after_s:
            return {
                **base_status,
                "state": "stale",
                "connected": True,
                "value": round(float(rotation), 3),
                "updated_at": updated_at,
                "message": f"Gyro rotation last updated {age_s:.1f}s ago.",
            }

        stream_source = source or "telemetry"
        return {
            **base_status,
            "state": "online",
            "connected": True,
            "value": round(float(rotation), 3),
            "updated_at": updated_at,
            "source": stream_source,
            "message": f"Gyro rotation is streaming ({stream_source}).",
        }

    def merge_observation(self, observation: dict[str, Any]) -> dict[str, Any]:
        merged = dict(observation)
        self.poll()
        merged.update(self.current_state())
        return merged

    def merge_action(self, action: dict[str, Any]) -> dict[str, float]:
        merged = dict(action)
        merged.update(self.current_motion())
        return {key: float(merged.get(key, 0.0)) if key in BASE_STATE_KEYS else merged[key] for key in merged}

    def send_motion(self, motion: dict[str, Any], ttl_ms: int | None = None) -> bool:
        ttl_ms = max(int(ttl_ms if ttl_ms is not None else self.command_timeout_s * 1000), 50)
        line = "!velocity {:.4f} {:.4f} {:.4f} {}".format(
            float(motion.get("x.vel", 0.0) or 0.0),
            float(motion.get("y.vel", 0.0) or 0.0),
            float(motion.get("theta.vel", 0.0) or 0.0),
            ttl_ms,
        )
        return self._write_command_line(line)

    def send_control_mode(self, mode: str) -> bool:
        normalized = str(mode or "").strip().lower()
        if normalized not in {VEX_DRIVE_CONTROL_MODE, VEX_ECU_CONTROL_MODE}:
            return False
        return self._write_command_line(f"!mode {normalized}")

    def send_ecu_targets(
        self,
        targets: dict[str, Any],
        *,
        motion: dict[str, Any] | None = None,
        command_dt_ms: int = 20,
        ttl_ms: int | None = None,
    ) -> bool:
        motion_payload = motion or {}
        ttl = max(int(ttl_ms if ttl_ms is not None else self.command_timeout_s * 1000), 50)
        dt_ms = max(int(command_dt_ms), 20)
        line = "!ecu {:.2f} {:.2f} {:.2f} {:.2f} {:.4f} {:.4f} {:.4f} {} {}".format(
            float(targets.get("vex_front_right.pos", 0.0) or 0.0),
            float(targets.get("vex_front_left.pos", 0.0) or 0.0),
            float(targets.get("vex_rear_right.pos", 0.0) or 0.0),
            float(targets.get("vex_rear_left.pos", 0.0) or 0.0),
            float(motion_payload.get("x.vel", 0.0) or 0.0),
            float(motion_payload.get("y.vel", 0.0) or 0.0),
            float(motion_payload.get("theta.vel", 0.0) or 0.0),
            dt_ms,
            ttl,
        )
        return self._write_command_line(line)

    def send_stop(self) -> bool:
        sent_zero = self.send_motion(dict.fromkeys(BASE_STATE_KEYS, 0.0))
        sent_release = self.release_control()
        return sent_zero or sent_release

    def send_hold(self, ttl_ms: int = 1200) -> bool:
        ttl = max(int(ttl_ms), 50)
        return self._write_command_line(f"!hold {ttl}")

    def set_pose_origin(self, ttl_ms: int = 1200, timeout_s: float = 1.5) -> bool:
        probe_started_at = time.time()
        ttl = max(int(ttl_ms), 50)
        if not self._write_command_line(f"!origin {ttl}"):
            return False
        return self.wait_for_source(
            {"pi-origin"},
            since_s=probe_started_at,
            timeout_s=timeout_s,
        )

    def release_control(self) -> bool:
        return self._write_command_line("!release")

    def wait_for_update(self, timeout_s: float = 5.0, poll_interval_s: float = 0.05) -> bool:
        deadline = time.time() + max(float(timeout_s), 0.0)
        while time.time() <= deadline:
            self.poll()
            if self.latest_update_time:
                return True
            time.sleep(max(float(poll_interval_s), 0.01))
        return False

    def wait_for_source(
        self,
        expected_sources: set[str],
        *,
        since_s: float,
        timeout_s: float = 1.0,
        poll_interval_s: float = 0.05,
    ) -> bool:
        deadline = time.time() + max(float(timeout_s), 0.0)
        while time.time() <= deadline:
            self.poll()
            payload = self.latest_payload if isinstance(self.latest_payload, dict) else {}
            source = payload.get("source")
            if (
                self.latest_update_time >= since_s
                and isinstance(source, str)
                and source in expected_sources
            ):
                return True
            time.sleep(max(float(poll_interval_s), 0.01))
        return False

    def remote_command_stream_active(self, *, timeout_s: float = 1.0) -> bool:
        if self.serial_handle is None:
            return False
        if not self.wait_for_update(timeout_s=timeout_s):
            return False
        probe_started_at = time.time()
        if not self.send_motion(dict.fromkeys(BASE_STATE_KEYS, 0.0), ttl_ms=350):
            return False
        accepted = self.wait_for_source(
            {"pi-drive"},
            since_s=probe_started_at,
            timeout_s=timeout_s,
        )
        self.send_hold(ttl_ms=350)
        return accepted

    def mixer_version_current(self) -> bool:
        version = self.latest_extra_state.get("vex_mixer_version")
        return isinstance(version, (int, float)) and int(version) >= REQUIRED_VEX_MIXER_VERSION

    def _handle_line(self, line: str) -> None:
        if not line:
            return

        try:
            payload = json.loads(line)
        except json.JSONDecodeError:
            return

        if not isinstance(payload, dict):
            return

        source = payload.get("base")
        if isinstance(source, dict):
            payload = source

        motion_present = False
        motion = {
            key: 0.0
            for key in VEX_STATE_ALIASES
        }
        for key, aliases in VEX_STATE_ALIASES.items():
            motion[key] = _extract_numeric(payload, aliases)
            if not motion_present and any(alias in payload for alias in aliases):
                motion_present = True

        extra_state = {}
        for key, aliases in VEX_MOTOR_STATE_ALIASES.items():
            value = _extract_optional_numeric(payload, aliases)
            if value is not None:
                extra_state[key] = value
        for key, aliases in VEX_INERTIAL_STATE_ALIASES.items():
            value = _extract_optional_numeric(payload, aliases)
            if value is not None:
                extra_state[key] = value
        for key, aliases in VEX_POSE_STATE_ALIASES.items():
            value = _extract_optional_numeric(payload, aliases)
            if value is not None:
                extra_state[key] = value

        if not motion_present and not extra_state:
            return

        self.latest_motion = motion
        if extra_state:
            self.latest_extra_state = extra_state
        self.latest_payload = payload
        self.latest_update_time = time.time()

    def _write_command_line(self, line: str) -> bool:
        if self.serial_handle is None:
            return False

        try:
            serialized = line.strip() + "\n"
            self.serial_handle.write(serialized.encode("utf-8"))
            self.last_command_sent_at = time.time()
            return True
        except Exception as exc:
            self.logger.warning("VEX base bridge write failed on %s: %s", self.port, exc)
            return False


def _extract_replay_samples(samples: list[dict[str, Any]]) -> list[tuple[int, float, float, float]]:
    replay_samples: list[tuple[int, float, float, float]] = []
    previous_motion: tuple[float, float, float] | None = None
    previous_t_ms: int | None = None

    for sample in samples:
        if not isinstance(sample, dict):
            continue
        state = sample.get("state")
        raw_t_s = sample.get("t_s")
        if not isinstance(state, dict) or not isinstance(raw_t_s, (int, float)):
            continue

        motion = (
            round(_extract_numeric(state, VEX_STATE_ALIASES["x.vel"]), 4),
            round(_extract_numeric(state, VEX_STATE_ALIASES["y.vel"]), 4),
            round(_extract_numeric(state, VEX_STATE_ALIASES["theta.vel"]), 4),
        )
        t_ms = max(0, int(round(float(raw_t_s) * 1000)))
        if previous_motion is None or motion != previous_motion or t_ms != previous_t_ms:
            replay_samples.append((t_ms, *motion))
            previous_motion = motion
            previous_t_ms = t_ms

    if not replay_samples:
        replay_samples.append((0, 0.0, 0.0, 0.0))
    elif replay_samples[0][0] != 0:
        replay_samples.insert(0, (0, replay_samples[0][1], replay_samples[0][2], replay_samples[0][3]))
    return replay_samples


def _extract_ecu_replay_samples(
    samples: list[dict[str, Any]],
) -> list[tuple[int, float, float, float, float, float, float, float]]:
    replay_samples: list[tuple[int, float, float, float, float, float, float, float]] = []
    previous_signature: tuple[float, float, float, float, float, float, float] | None = None
    previous_t_ms: int | None = None

    for sample in samples:
        if not isinstance(sample, dict):
            continue
        state = sample.get("state")
        raw_t_s = sample.get("t_s")
        if not isinstance(state, dict) or not isinstance(raw_t_s, (int, float)):
            continue

        signature = (
            round(_extract_numeric(state, VEX_MOTOR_STATE_ALIASES["vex_front_right.pos"]), 2),
            round(_extract_numeric(state, VEX_MOTOR_STATE_ALIASES["vex_front_left.pos"]), 2),
            round(_extract_numeric(state, VEX_MOTOR_STATE_ALIASES["vex_rear_right.pos"]), 2),
            round(_extract_numeric(state, VEX_MOTOR_STATE_ALIASES["vex_rear_left.pos"]), 2),
            round(_extract_numeric(state, VEX_STATE_ALIASES["x.vel"]), 4),
            round(_extract_numeric(state, VEX_STATE_ALIASES["y.vel"]), 4),
            round(_extract_numeric(state, VEX_STATE_ALIASES["theta.vel"]), 4),
        )
        t_ms = max(0, int(round(float(raw_t_s) * 1000)))
        if previous_signature is None or signature != previous_signature or t_ms != previous_t_ms:
            replay_samples.append((t_ms, *signature))
            previous_signature = signature
            previous_t_ms = t_ms

    if not replay_samples:
        replay_samples.append((0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
    elif replay_samples[0][0] != 0:
        replay_samples.insert(
            0,
            (
                0,
                replay_samples[0][1],
                replay_samples[0][2],
                replay_samples[0][3],
                replay_samples[0][4],
                replay_samples[0][5],
                replay_samples[0][6],
                replay_samples[0][7],
            ),
        )
    return replay_samples


def _extract_vex_start_positions(samples: list[dict[str, Any]]) -> dict[str, float] | None:
    for sample in samples:
        if not isinstance(sample, dict):
            continue
        state = sample.get("state")
        if not isinstance(state, dict):
            continue

        positions = {
            key: float(state[key])
            for key in VEX_MOTOR_STATE_KEYS
            if isinstance(state.get(key), (int, float))
        }
        if len(positions) == len(VEX_MOTOR_STATE_KEYS):
            return positions
        if positions:
            break

    return None


def build_vex_drive_replay_program_source(
    samples: list[tuple[int, float, float, float]],
    control_config: dict[str, Any] | None,
    start_positions: dict[str, float] | None = None,
) -> str:
    config = normalize_vex_control_config(control_config)
    tuning = config["tuning"]
    controls = config["controls"]
    motor_lines = _motor_definition_lines(config)
    inertial_lines = _inertial_definition_lines(config)
    forward_expr = _axis_expression(controls["forwardAxis"], bool(controls["invertForward"]))
    strafe_expr = _axis_expression(controls["strafeAxis"], bool(controls["invertStrafe"]))
    turn_expr = _axis_expression(controls["turnAxis"], bool(controls["invertTurn"]))
    sample_rows = ",\n    ".join(
        f"({t_ms}, {x_vel:.4f}, {y_vel:.4f}, {theta_vel:.4f})"
        for t_ms, x_vel, y_vel, theta_vel in samples
    )
    if start_positions:
        start_position_rows = ",\n    ".join(
            f'"{key}": {float(start_positions[key]):.4f}'
            for key in VEX_MOTOR_STATE_KEYS
            if key in start_positions
        )
        start_positions_literal = "{\n    " + start_position_rows + "\n}"
    else:
        start_positions_literal = "{}"
    return f"""from vex import *

brain = Brain()
controller_1 = Controller(PRIMARY)

{motor_lines}
{inertial_lines}

MAX_LINEAR_SPEED_MPS = {float(tuning["maxLinearSpeedMps"]):.4f}
MAX_TURN_SPEED_DPS = {float(tuning["maxTurnSpeedDps"]):.4f}
DEADBAND_PERCENT = {int(tuning["deadbandPercent"])}
TELEMETRY_INTERVAL_MS = 50
PREPOSITION_TIMEOUT_MS = 6000
PREPOSITION_TOLERANCE_DEG = 6.0
PREPOSITION_MIN_SPEED_PCT = 15.0
PREPOSITION_MAX_SPEED_PCT = 65.0
PREPOSITION_KP = 0.35
TRAJECTORY_MS = [
    {sample_rows}
]
START_POSITIONS_DEG = {start_positions_literal}

replay_index = 0
replay_complete = False


def clamp(value, low, high):
    return max(low, min(high, value))


def apply_deadband(value):
    if abs(value) < DEADBAND_PERCENT:
        return 0
    return value


def controller_motion():
    forward_pct = apply_deadband({forward_expr})
    strafe_pct = apply_deadband({strafe_expr})
    turn_pct = apply_deadband({turn_expr})
    return {{
        "x.vel": (strafe_pct / 100.0) * MAX_LINEAR_SPEED_MPS,
        "y.vel": (forward_pct / 100.0) * MAX_LINEAR_SPEED_MPS,
        "theta.vel": (turn_pct / 100.0) * MAX_TURN_SPEED_DPS,
    }}


def motion_to_percent(motion):
    return (
        clamp((motion["y.vel"] / MAX_LINEAR_SPEED_MPS) * 100.0, -100.0, 100.0),
        clamp((motion["x.vel"] / MAX_LINEAR_SPEED_MPS) * 100.0, -100.0, 100.0),
        clamp((motion["theta.vel"] / MAX_TURN_SPEED_DPS) * 100.0, -100.0, 100.0),
    )


def spin_drive_motor(motor, percent):
    if abs(percent) <= 0.01:
        motor.stop(HOLD)
        return
    motor.set_velocity(abs(percent), PERCENT)
    motor.spin(FORWARD if percent >= 0 else REVERSE)


def apply_drive_motion(motion):
    forward_pct, strafe_pct, turn_pct = motion_to_percent(motion)
    front_left_pct = forward_pct + turn_pct + strafe_pct
    rear_left_pct = forward_pct + turn_pct - strafe_pct
    front_right_pct = forward_pct - turn_pct - strafe_pct
    rear_right_pct = forward_pct - turn_pct + strafe_pct

    spin_drive_motor(front_left, front_left_pct)
    spin_drive_motor(rear_left, rear_left_pct)
    spin_drive_motor(front_right, front_right_pct)
    spin_drive_motor(rear_right, rear_right_pct)


def read_motor_positions():
    return {{
        "vex_front_right.pos": front_right.position(DEGREES),
        "vex_front_left.pos": front_left.position(DEGREES),
        "vex_rear_right.pos": rear_right.position(DEGREES),
        "vex_rear_left.pos": rear_left.position(DEGREES),
    }}


def hold_drive():
    front_left.stop(HOLD)
    rear_left.stop(HOLD)
    front_right.stop(HOLD)
    rear_right.stop(HOLD)


def move_to_recorded_start():
    if len(START_POSITIONS_DEG) != 4:
        return False

    deadline_ms = brain.timer.system() + PREPOSITION_TIMEOUT_MS
    while True:
        settled = True
        positions = read_motor_positions()
        for key, motor in (
            ("vex_front_right.pos", front_right),
            ("vex_front_left.pos", front_left),
            ("vex_rear_right.pos", rear_right),
            ("vex_rear_left.pos", rear_left),
        ):
            target = START_POSITIONS_DEG.get(key)
            if target is None:
                continue
            error = target - positions.get(key, 0.0)
            if abs(error) <= PREPOSITION_TOLERANCE_DEG:
                motor.stop(HOLD)
                continue

            settled = False
            speed_pct = clamp(abs(error) * PREPOSITION_KP, PREPOSITION_MIN_SPEED_PCT, PREPOSITION_MAX_SPEED_PCT)
            motor.set_velocity(speed_pct, PERCENT)
            motor.spin(FORWARD if error >= 0 else REVERSE)

        if settled:
            hold_drive()
            return True
        if brain.timer.system() >= deadline_ms:
            hold_drive()
            return False

        wait(20, MSEC)


def replay_motion(now_ms):
    global replay_index
    global replay_complete

    if replay_complete:
        return None

    while replay_index + 1 < len(TRAJECTORY_MS) and now_ms >= TRAJECTORY_MS[replay_index + 1][0]:
        replay_index += 1

    if now_ms > TRAJECTORY_MS[-1][0]:
        replay_complete = True
        return None

    t_ms, x_vel, y_vel, theta_vel = TRAJECTORY_MS[replay_index]
    return {{"x.vel": x_vel, "y.vel": y_vel, "theta.vel": theta_vel}}


def print_motion(motion, source):
    positions = read_motor_positions()
    inertial_state = read_inertial_state()
    theta_vel = inertial_state["vex_inertial_rate_z.dps"] if inertial_available() else motion["theta.vel"]
    print(
        '{{"x.vel":%.4f,"y.vel":%.4f,"theta.vel":%.4f,"vex_front_right.pos":%.4f,"vex_front_left.pos":%.4f,"vex_rear_right.pos":%.4f,"vex_rear_left.pos":%.4f,"vex_inertial_rotation.deg":%.4f,"vex_inertial_heading.deg":%.4f,"vex_inertial_rate_z.dps":%.4f,"source":"%s"}}'
        % (
            motion["x.vel"],
            motion["y.vel"],
            theta_vel,
            positions["vex_front_right.pos"],
            positions["vex_front_left.pos"],
            positions["vex_rear_right.pos"],
            positions["vex_rear_left.pos"],
            inertial_state["vex_inertial_rotation.deg"],
            inertial_state["vex_inertial_heading.deg"],
            inertial_state["vex_inertial_rate_z.dps"],
            source,
        )
    )


def main():
    last_telemetry_ms = -TELEMETRY_INTERVAL_MS
    initialize_inertial()
    wait(30, MSEC)
    print("\\033[2J")
    move_to_recorded_start()
    replay_started_ms = brain.timer.system()

    while True:
        now_ms = brain.timer.system() - replay_started_ms
        active_motion = replay_motion(now_ms)
        if active_motion is None:
            active_motion = controller_motion()
            source = "controller"
        else:
            source = "replay"

        apply_drive_motion(active_motion)

        if now_ms - last_telemetry_ms >= TELEMETRY_INTERVAL_MS:
            print_motion(active_motion, source)
            last_telemetry_ms = now_ms

        wait(20, MSEC)


main()
"""


def build_vex_ecu_replay_program_source(
    samples: list[tuple[int, float, float, float, float, float, float, float]],
    control_config: dict[str, Any] | None,
    start_positions: dict[str, float] | None = None,
) -> str:
    config = normalize_vex_control_config(control_config)
    tuning = config["tuning"]
    controls = config["controls"]
    motor_lines = _motor_definition_lines(config)
    inertial_lines = _inertial_definition_lines(config)
    forward_expr = _axis_expression(controls["forwardAxis"], bool(controls["invertForward"]))
    strafe_expr = _axis_expression(controls["strafeAxis"], bool(controls["invertStrafe"]))
    turn_expr = _axis_expression(controls["turnAxis"], bool(controls["invertTurn"]))
    sample_rows = ",\n    ".join(
        f"({t_ms}, {front_right_pos:.2f}, {front_left_pos:.2f}, {rear_right_pos:.2f}, {rear_left_pos:.2f}, {x_vel:.4f}, {y_vel:.4f}, {theta_vel:.4f})"
        for (
            t_ms,
            front_right_pos,
            front_left_pos,
            rear_right_pos,
            rear_left_pos,
            x_vel,
            y_vel,
            theta_vel,
        ) in samples
    )
    if start_positions:
        start_position_rows = ",\n    ".join(
            f'"{key}": {float(start_positions[key]):.4f}'
            for key in VEX_MOTOR_STATE_KEYS
            if key in start_positions
        )
        start_positions_literal = "{\n    " + start_position_rows + "\n}"
    else:
        start_positions_literal = "{}"
    return f"""from vex import *

brain = Brain()
controller_1 = Controller(PRIMARY)

{motor_lines}
{inertial_lines}

MAX_LINEAR_SPEED_MPS = {float(tuning["maxLinearSpeedMps"]):.4f}
MAX_TURN_SPEED_DPS = {float(tuning["maxTurnSpeedDps"]):.4f}
DEADBAND_PERCENT = {int(tuning["deadbandPercent"])}
TELEMETRY_INTERVAL_MS = 50
PREPOSITION_TIMEOUT_MS = 6000
PREPOSITION_TOLERANCE_DEG = 6.0
PREPOSITION_MIN_SPEED_PCT = 15.0
PREPOSITION_MAX_SPEED_PCT = 65.0
PREPOSITION_KP = 0.35
ECU_IDLE_TOLERANCE_DEG = 0.75
ECU_COMMAND_TOLERANCE_DEG = 0.25
ECU_MIN_SPEED_DPS = 30.0
ECU_MAX_SPEED_DPS = 360.0
TRAJECTORY_STATES = [
    {sample_rows}
]
START_POSITIONS_DEG = {start_positions_literal}
MOTOR_ORDER = (
    ("vex_front_right.pos", front_right),
    ("vex_front_left.pos", front_left),
    ("vex_rear_right.pos", rear_right),
    ("vex_rear_left.pos", rear_left),
)

last_targets = {{}}
replay_index = 0
replay_complete = False


def clamp(value, low, high):
    return max(low, min(high, value))


def apply_deadband(value):
    if abs(value) < DEADBAND_PERCENT:
        return 0
    return value


def controller_motion():
    forward_pct = apply_deadband({forward_expr})
    strafe_pct = apply_deadband({strafe_expr})
    turn_pct = apply_deadband({turn_expr})
    return {{
        "x.vel": (strafe_pct / 100.0) * MAX_LINEAR_SPEED_MPS,
        "y.vel": (forward_pct / 100.0) * MAX_LINEAR_SPEED_MPS,
        "theta.vel": (turn_pct / 100.0) * MAX_TURN_SPEED_DPS,
    }}


def motion_to_percent(motion):
    return (
        clamp((motion["y.vel"] / MAX_LINEAR_SPEED_MPS) * 100.0, -100.0, 100.0),
        clamp((motion["x.vel"] / MAX_LINEAR_SPEED_MPS) * 100.0, -100.0, 100.0),
        clamp((motion["theta.vel"] / MAX_TURN_SPEED_DPS) * 100.0, -100.0, 100.0),
    )


def spin_drive_motor(motor, percent):
    if abs(percent) <= 0.01:
        motor.stop(HOLD)
        return
    motor.set_velocity(abs(percent), PERCENT)
    motor.spin(FORWARD if percent >= 0 else REVERSE)


def apply_drive_motion(motion):
    forward_pct, strafe_pct, turn_pct = motion_to_percent(motion)
    front_left_pct = forward_pct + turn_pct + strafe_pct
    rear_left_pct = forward_pct + turn_pct - strafe_pct
    front_right_pct = forward_pct - turn_pct - strafe_pct
    rear_right_pct = forward_pct - turn_pct + strafe_pct

    spin_drive_motor(front_left, front_left_pct)
    spin_drive_motor(rear_left, rear_left_pct)
    spin_drive_motor(front_right, front_right_pct)
    spin_drive_motor(rear_right, rear_right_pct)


def read_motor_positions():
    return {{
        "vex_front_right.pos": front_right.position(DEGREES),
        "vex_front_left.pos": front_left.position(DEGREES),
        "vex_rear_right.pos": rear_right.position(DEGREES),
        "vex_rear_left.pos": rear_left.position(DEGREES),
    }}


def hold_drive():
    front_left.stop(HOLD)
    rear_left.stop(HOLD)
    front_right.stop(HOLD)
    rear_right.stop(HOLD)


def move_to_recorded_start():
    if len(START_POSITIONS_DEG) != 4:
        return False

    deadline_ms = brain.timer.system() + PREPOSITION_TIMEOUT_MS
    while True:
        settled = True
        positions = read_motor_positions()
        for key, motor in MOTOR_ORDER:
            target = START_POSITIONS_DEG.get(key)
            if target is None:
                continue
            error = target - positions.get(key, 0.0)
            if abs(error) <= PREPOSITION_TOLERANCE_DEG:
                motor.stop(HOLD)
                continue

            settled = False
            speed_pct = clamp(abs(error) * PREPOSITION_KP, PREPOSITION_MIN_SPEED_PCT, PREPOSITION_MAX_SPEED_PCT)
            motor.set_velocity(speed_pct, PERCENT)
            motor.spin(FORWARD if error >= 0 else REVERSE)

        if settled:
            hold_drive()
            return True
        if brain.timer.system() >= deadline_ms:
            hold_drive()
            return False

        wait(20, MSEC)


def replay_state(now_ms):
    global replay_index
    global replay_complete

    if replay_complete:
        return None

    while replay_index + 1 < len(TRAJECTORY_STATES) and now_ms >= TRAJECTORY_STATES[replay_index + 1][0]:
        replay_index += 1

    if now_ms > TRAJECTORY_STATES[-1][0]:
        replay_complete = True
        return None

    row = TRAJECTORY_STATES[replay_index]
    previous_row = TRAJECTORY_STATES[replay_index - 1] if replay_index > 0 else row
    return {{
        "targets": {{
            "vex_front_right.pos": row[1],
            "vex_front_left.pos": row[2],
            "vex_rear_right.pos": row[3],
            "vex_rear_left.pos": row[4],
        }},
        "previous_targets": {{
            "vex_front_right.pos": previous_row[1],
            "vex_front_left.pos": previous_row[2],
            "vex_rear_right.pos": previous_row[3],
            "vex_rear_left.pos": previous_row[4],
        }},
        "motion": {{
            "x.vel": row[5],
            "y.vel": row[6],
            "theta.vel": row[7],
        }},
        "dt_ms": max(row[0] - previous_row[0], 20),
    }}


def apply_ecu_replay(sample):
    dt_s = max(sample["dt_ms"] / 1000.0, 0.02)
    for key, motor in MOTOR_ORDER:
        target = sample["targets"][key]
        previous_target = sample["previous_targets"][key]
        delta = target - previous_target

        if abs(delta) <= ECU_IDLE_TOLERANCE_DEG:
            if motor.is_done():
                motor.stop(HOLD)
            continue

        speed_dps = clamp(abs(delta) / dt_s, ECU_MIN_SPEED_DPS, ECU_MAX_SPEED_DPS)
        prior_target = last_targets.get(key)
        if prior_target is None or abs(prior_target - target) > ECU_COMMAND_TOLERANCE_DEG:
            motor.set_stopping(HOLD)
            motor.spin_to_position(target, DEGREES, speed_dps, VelocityUnits.DPS, wait=False)
            last_targets[key] = target
        elif motor.is_done():
            motor.stop(HOLD)


def print_motion(motion, source):
    positions = read_motor_positions()
    inertial_state = read_inertial_state()
    theta_vel = inertial_state["vex_inertial_rate_z.dps"] if inertial_available() else motion["theta.vel"]
    print(
        '{{"x.vel":%.4f,"y.vel":%.4f,"theta.vel":%.4f,"vex_front_right.pos":%.4f,"vex_front_left.pos":%.4f,"vex_rear_right.pos":%.4f,"vex_rear_left.pos":%.4f,"vex_inertial_rotation.deg":%.4f,"vex_inertial_heading.deg":%.4f,"vex_inertial_rate_z.dps":%.4f,"source":"%s"}}'
        % (
            motion["x.vel"],
            motion["y.vel"],
            theta_vel,
            positions["vex_front_right.pos"],
            positions["vex_front_left.pos"],
            positions["vex_rear_right.pos"],
            positions["vex_rear_left.pos"],
            inertial_state["vex_inertial_rotation.deg"],
            inertial_state["vex_inertial_heading.deg"],
            inertial_state["vex_inertial_rate_z.dps"],
            source,
        )
    )


def main():
    last_telemetry_ms = -TELEMETRY_INTERVAL_MS
    initialize_inertial()
    wait(30, MSEC)
    print("\\033[2J")
    move_to_recorded_start()
    for _, motor in MOTOR_ORDER:
        motor.set_stopping(HOLD)
    replay_started_ms = brain.timer.system()

    while True:
        now_ms = brain.timer.system() - replay_started_ms
        active_state = replay_state(now_ms)
        if active_state is None:
            active_motion = controller_motion()
            apply_drive_motion(active_motion)
            source = "controller"
        else:
            apply_ecu_replay(active_state)
            active_motion = active_state["motion"]
            source = "replay-ecu"

        if now_ms - last_telemetry_ms >= TELEMETRY_INTERVAL_MS:
            print_motion(active_motion, source)
            last_telemetry_ms = now_ms

        wait(20, MSEC)


main()
"""


class VexBaseTelemetryManager:
    def __init__(
        self,
        *,
        requested_vexcom_path: str,
        telemetry_slot: int,
        cache_dir: str,
        logger: logging.Logger,
    ) -> None:
        self.logger = logger
        self.requested_vexcom_path = requested_vexcom_path
        self.telemetry_slot = max(1, min(8, int(telemetry_slot)))
        self.cache_dir = Path(cache_dir).expanduser()
        self.vexcom_path: str | None = None
        self.comm_port: str | None = None
        self.status_message = ""
        self.refresh()

    def refresh(self) -> None:
        self.vexcom_path = detect_vexcom_path(self.requested_vexcom_path)
        self.comm_port = detect_vex_comm_port()
        if self.vexcom_path is None:
            self.status_message = "VEX base telemetry unavailable: vexcom was not found on the Pi."
        elif self.comm_port is None:
            self.status_message = "VEX base telemetry unavailable: no V5 Brain communication port detected."
        else:
            self.status_message = (
                f"VEX base telemetry ready on slot {self.telemetry_slot} via {self.comm_port} using {self.vexcom_path}."
            )

    @property
    def enabled(self) -> bool:
        return self.vexcom_path is not None and self.comm_port is not None

    def install_and_run(
        self,
        control_config: dict[str, Any] | None,
        *,
        program_name: str = "Base Telemetry",
        run_after_install: bool = True,
    ) -> bool:
        if not self.enabled or self.vexcom_path is None or self.comm_port is None:
            return False

        program_source = build_vex_telemetry_program_source(control_config)
        program_path = self._write_program(program_source)

        upload = self._run_vexcom(
            [
                self.vexcom_path,
                "--name",
                program_name,
                "--slot",
                str(self.telemetry_slot),
                "--write",
                str(program_path),
                "--json",
                self.comm_port,
            ]
        )
        if upload.returncode != 0:
            self.logger.warning("VEX telemetry upload failed: %s", upload.stderr.strip() or upload.stdout.strip())
            return False

        if not run_after_install:
            return True

        run = self._run_vexcom(
            [
                self.vexcom_path,
                "--slot",
                str(self.telemetry_slot),
                "--run",
                self.comm_port,
            ]
        )
        if run.returncode != 0:
            self.logger.warning("VEX telemetry start failed: %s", run.stderr.strip() or run.stdout.strip())
            return False

        return True

    def _write_program(self, source: str) -> Path:
        self.cache_dir.mkdir(parents=True, exist_ok=True)
        digest = hashlib.sha1(source.encode("utf-8")).hexdigest()[:12]
        path = self.cache_dir / f"base-telemetry-{digest}.py"
        path.write_text(source)
        return path

    def _run_vexcom(self, argv: list[str]) -> subprocess.CompletedProcess[str]:
        return subprocess.run(argv, capture_output=True, text=True, timeout=30, check=False)


def ensure_vex_command_stream(
    vex_base_bridge: VexBaseBridge,
    vex_base_telemetry: VexBaseTelemetryManager | None,
    control_config: dict[str, Any] | None,
    *,
    program_name: str,
    logger: logging.Logger,
    startup_delay_s: float = 1.0,
    telemetry_timeout_s: float = 5.0,
) -> bool:
    if vex_base_bridge.remote_command_stream_active(timeout_s=0.75):
        if vex_base_bridge.mixer_version_current():
            return True
        logger.warning(
            "VEX command stream is active, but the Brain program is missing mixer version %s; reinstalling.",
            REQUIRED_VEX_MIXER_VERSION,
        )
    elif vex_base_bridge.enabled:
        logger.warning("VEX telemetry is streaming, but it did not accept the remote command probe.")

    if vex_base_telemetry is None or not vex_base_telemetry.enabled:
        logger.warning("Cannot install the required VEX command/telemetry program on the Brain.")
        return False

    restored = vex_base_telemetry.install_and_run(
        control_config,
        program_name=program_name,
        run_after_install=True,
    )
    if not restored:
        logger.warning("Failed to install the VEX command/telemetry program on the Brain.")
        return False
    time.sleep(max(float(startup_delay_s), 0.0))
    vex_base_bridge.connect()

    if vex_base_bridge.remote_command_stream_active(timeout_s=telemetry_timeout_s):
        if vex_base_bridge.mixer_version_current():
            return True
        logger.warning(
            "VEX command/telemetry program started but still did not report mixer version %s.",
            REQUIRED_VEX_MIXER_VERSION,
        )
        return False

    logger.warning("Timed out waiting for the VEX command/telemetry program to accept remote commands.")
    return False


class VexBaseReplayManager:
    def __init__(
        self,
        *,
        requested_vexcom_path: str,
        replay_slot: int,
        cache_dir: str,
        control_config: dict[str, Any] | None,
        logger: logging.Logger,
    ) -> None:
        self.logger = logger
        self.replay_slot = max(1, min(8, int(replay_slot)))
        self.cache_dir = Path(cache_dir).expanduser()
        self.control_config = normalize_vex_control_config(control_config)
        self.vexcom_path = detect_vexcom_path(requested_vexcom_path)
        self.comm_port = detect_vex_comm_port()
        if self.vexcom_path is None:
            self.status_message = "VEX base replay unavailable: vexcom was not found on the Pi."
        elif self.comm_port is None:
            self.status_message = "VEX base replay unavailable: no V5 Brain communication port detected."
        else:
            self.status_message = (
                f"VEX base replay ready on slot {self.replay_slot} via {self.comm_port} using {self.vexcom_path}."
            )

    @property
    def enabled(self) -> bool:
        return self.vexcom_path is not None and self.comm_port is not None

    def launch_replay(self, samples: list[dict[str, Any]], replay_mode: str = "drive") -> bool:
        if not self.enabled or self.vexcom_path is None or self.comm_port is None:
            return False

        start_positions = _extract_vex_start_positions(samples)
        if replay_mode == "ecu":
            replay_samples = _extract_ecu_replay_samples(samples)
            program_source = build_vex_ecu_replay_program_source(
                replay_samples,
                self.control_config,
                start_positions=start_positions,
            )
        else:
            replay_samples = _extract_replay_samples(samples)
            program_source = build_vex_drive_replay_program_source(
                replay_samples,
                self.control_config,
                start_positions=start_positions,
            )
        program_path = self._write_program(program_source)

        upload = self._run_vexcom(
            [
                self.vexcom_path,
                "--name",
                "Base Replay",
                "--slot",
                str(self.replay_slot),
                "--write",
                str(program_path),
                "--json",
                self.comm_port,
            ]
        )
        if upload.returncode != 0:
            self.logger.warning("VEX replay upload failed: %s", upload.stderr.strip() or upload.stdout.strip())
            return False

        run = self._run_vexcom(
            [
                self.vexcom_path,
                "--slot",
                str(self.replay_slot),
                "--run",
                self.comm_port,
            ]
        )
        if run.returncode != 0:
            self.logger.warning("VEX replay start failed: %s", run.stderr.strip() or run.stdout.strip())
            return False

        return True

    def _write_program(self, source: str) -> Path:
        self.cache_dir.mkdir(parents=True, exist_ok=True)
        digest = hashlib.sha1(source.encode("utf-8")).hexdigest()[:12]
        path = self.cache_dir / f"base-replay-{digest}.py"
        path.write_text(source)
        return path

    def _run_vexcom(self, argv: list[str]) -> subprocess.CompletedProcess[str]:
        return subprocess.run(argv, capture_output=True, text=True, timeout=30, check=False)
