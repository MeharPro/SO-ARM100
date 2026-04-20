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

from lekiwi_runtime import BASE_STATE_KEYS

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
VALID_VEX_AXES = {"axis1", "axis2", "axis3", "axis4"}
DEFAULT_VEX_CONTROL_CONFIG = {
    "motors": {
        "frontRight": {"port": 1, "reversed": True},
        "frontLeft": {"port": 2, "reversed": False},
        "rearRight": {"port": 9, "reversed": True},
        "rearLeft": {"port": 10, "reversed": False},
    },
    "controls": {
        "forwardAxis": "axis2",
        "strafeAxis": "axis4",
        "turnAxis": "axis1",
        "invertForward": False,
        "invertStrafe": False,
        "invertTurn": False,
    },
    "tuning": {
        "deadbandPercent": 5,
        "maxLinearSpeedMps": 0.35,
        "maxTurnSpeedDps": 90.0,
    },
}


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


def normalize_vex_control_config(raw: dict[str, Any] | None) -> dict[str, Any]:
    payload = raw if isinstance(raw, dict) else {}
    motors = payload.get("motors") if isinstance(payload.get("motors"), dict) else {}
    controls = payload.get("controls") if isinstance(payload.get("controls"), dict) else {}
    tuning = payload.get("tuning") if isinstance(payload.get("tuning"), dict) else {}

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

    def float_value(group: dict[str, Any], key: str, fallback: float, low: float, high: float) -> float:
        candidate = group.get(key, fallback)
        try:
            numeric = float(candidate)
        except Exception:
            numeric = fallback
        return max(low, min(high, numeric))

    defaults = DEFAULT_VEX_CONTROL_CONFIG
    return {
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
        },
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


def build_vex_telemetry_program_source(control_config: dict[str, Any] | None) -> str:
    config = normalize_vex_control_config(control_config)
    tuning = config["tuning"]
    controls = config["controls"]
    motor_lines = _motor_definition_lines(config)
    forward_expr = _axis_expression(controls["forwardAxis"], bool(controls["invertForward"]))
    strafe_expr = _axis_expression(controls["strafeAxis"], bool(controls["invertStrafe"]))
    turn_expr = _axis_expression(controls["turnAxis"], bool(controls["invertTurn"]))
    return f"""from vex import *

brain = Brain()
controller_1 = Controller(PRIMARY)

{motor_lines}

MAX_LINEAR_SPEED_MPS = {float(tuning["maxLinearSpeedMps"]):.4f}
MAX_TURN_SPEED_DPS = {float(tuning["maxTurnSpeedDps"]):.4f}
DEADBAND_PERCENT = {int(tuning["deadbandPercent"])}
TELEMETRY_INTERVAL_MS = 50


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
        "x.vel": (forward_pct / 100.0) * MAX_LINEAR_SPEED_MPS,
        "y.vel": (strafe_pct / 100.0) * MAX_LINEAR_SPEED_MPS,
        "theta.vel": (turn_pct / 100.0) * MAX_TURN_SPEED_DPS,
    }}


def motion_to_percent(motion):
    return (
        clamp((motion["x.vel"] / MAX_LINEAR_SPEED_MPS) * 100.0, -100.0, 100.0),
        clamp((motion["y.vel"] / MAX_LINEAR_SPEED_MPS) * 100.0, -100.0, 100.0),
        clamp((motion["theta.vel"] / MAX_TURN_SPEED_DPS) * 100.0, -100.0, 100.0),
    )


def apply_drive_motion(motion):
    forward_pct, strafe_pct, turn_pct = motion_to_percent(motion)
    front_left_pct = forward_pct + turn_pct + strafe_pct
    rear_left_pct = forward_pct - turn_pct + strafe_pct
    front_right_pct = forward_pct - turn_pct - strafe_pct
    rear_right_pct = forward_pct + turn_pct - strafe_pct

    front_left.set_velocity(front_left_pct, PERCENT)
    rear_left.set_velocity(rear_left_pct, PERCENT)
    front_right.set_velocity(front_right_pct, PERCENT)
    rear_right.set_velocity(rear_right_pct, PERCENT)

    front_left.spin(FORWARD)
    rear_left.spin(FORWARD)
    front_right.spin(FORWARD)
    rear_right.spin(FORWARD)


def print_motion(motion, source):
    print(
        '{{"x.vel":%.4f,"y.vel":%.4f,"theta.vel":%.4f,"source":"%s"}}'
        % (motion["x.vel"], motion["y.vel"], motion["theta.vel"], source)
    )


def main():
    last_telemetry_ms = -TELEMETRY_INTERVAL_MS
    wait(30, MSEC)
    print("\\033[2J")

    while True:
        now_ms = brain.timer.system()
        active_motion = controller_motion()
        apply_drive_motion(active_motion)

        if now_ms - last_telemetry_ms >= TELEMETRY_INTERVAL_MS:
            print_motion(active_motion, "controller")
            last_telemetry_ms = now_ms

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
        self.latest_payload: dict[str, Any] | None = None
        self.latest_update_time = 0.0
        self.last_command_sent_at = 0.0
        self.status_message = "VEX base bridge disabled."

    @property
    def enabled(self) -> bool:
        return self.serial_handle is not None

    def connect(self) -> None:
        if serial is None:
            self.status_message = "VEX base bridge unavailable: pyserial is not installed."
            return

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
            self.logger.warning("VEX base bridge read failed on %s: %s", self.port, exc)
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

    def merge_observation(self, observation: dict[str, Any]) -> dict[str, Any]:
        merged = dict(observation)
        merged.update(self.poll())
        return merged

    def merge_action(self, action: dict[str, Any]) -> dict[str, float]:
        merged = dict(action)
        merged.update(self.current_motion())
        return {key: float(merged.get(key, 0.0)) if key in BASE_STATE_KEYS else merged[key] for key in merged}

    def send_motion(self, motion: dict[str, Any]) -> bool:
        payload = {
            "command": "velocity",
            "x.vel": float(motion.get("x.vel", 0.0) or 0.0),
            "y.vel": float(motion.get("y.vel", 0.0) or 0.0),
            "theta.vel": float(motion.get("theta.vel", 0.0) or 0.0),
            "ttl_ms": int(self.command_timeout_s * 1000),
        }
        return self._write_payload(payload)

    def send_stop(self) -> bool:
        sent_zero = self.send_motion(dict.fromkeys(BASE_STATE_KEYS, 0.0))
        sent_release = self.release_control()
        return sent_zero or sent_release

    def release_control(self) -> bool:
        return self._write_payload({"command": "release"})

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

        motion = {
            key: _extract_numeric(payload, aliases)
            for key, aliases in VEX_STATE_ALIASES.items()
        }
        if not any(abs(value) > 0 or key in payload for key, value in motion.items()):
            return

        self.latest_motion = motion
        self.latest_payload = payload
        self.latest_update_time = time.time()

    def _write_payload(self, payload: dict[str, Any]) -> bool:
        if self.serial_handle is None:
            return False

        try:
            serialized = json.dumps(payload, separators=(",", ":")) + "\n"
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


def build_vex_replay_program_source(
    samples: list[tuple[int, float, float, float]],
    control_config: dict[str, Any] | None,
) -> str:
    config = normalize_vex_control_config(control_config)
    tuning = config["tuning"]
    controls = config["controls"]
    motor_lines = _motor_definition_lines(config)
    forward_expr = _axis_expression(controls["forwardAxis"], bool(controls["invertForward"]))
    strafe_expr = _axis_expression(controls["strafeAxis"], bool(controls["invertStrafe"]))
    turn_expr = _axis_expression(controls["turnAxis"], bool(controls["invertTurn"]))
    sample_rows = ",\n    ".join(
        f"({t_ms}, {x_vel:.4f}, {y_vel:.4f}, {theta_vel:.4f})"
        for t_ms, x_vel, y_vel, theta_vel in samples
    )
    return f"""from vex import *

brain = Brain()
controller_1 = Controller(PRIMARY)

{motor_lines}

MAX_LINEAR_SPEED_MPS = {float(tuning["maxLinearSpeedMps"]):.4f}
MAX_TURN_SPEED_DPS = {float(tuning["maxTurnSpeedDps"]):.4f}
DEADBAND_PERCENT = {int(tuning["deadbandPercent"])}
TELEMETRY_INTERVAL_MS = 50
TRAJECTORY_MS = [
    {sample_rows}
]

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
        "x.vel": (forward_pct / 100.0) * MAX_LINEAR_SPEED_MPS,
        "y.vel": (strafe_pct / 100.0) * MAX_LINEAR_SPEED_MPS,
        "theta.vel": (turn_pct / 100.0) * MAX_TURN_SPEED_DPS,
    }}


def motion_to_percent(motion):
    return (
        clamp((motion["x.vel"] / MAX_LINEAR_SPEED_MPS) * 100.0, -100.0, 100.0),
        clamp((motion["y.vel"] / MAX_LINEAR_SPEED_MPS) * 100.0, -100.0, 100.0),
        clamp((motion["theta.vel"] / MAX_TURN_SPEED_DPS) * 100.0, -100.0, 100.0),
    )


def apply_drive_motion(motion):
    forward_pct, strafe_pct, turn_pct = motion_to_percent(motion)
    front_left_pct = forward_pct + turn_pct + strafe_pct
    rear_left_pct = forward_pct - turn_pct + strafe_pct
    front_right_pct = forward_pct - turn_pct - strafe_pct
    rear_right_pct = forward_pct + turn_pct - strafe_pct

    front_left.set_velocity(front_left_pct, PERCENT)
    rear_left.set_velocity(rear_left_pct, PERCENT)
    front_right.set_velocity(front_right_pct, PERCENT)
    rear_right.set_velocity(rear_right_pct, PERCENT)

    front_left.spin(FORWARD)
    rear_left.spin(FORWARD)
    front_right.spin(FORWARD)
    rear_right.spin(FORWARD)


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
    print(
        '{{"x.vel":%.4f,"y.vel":%.4f,"theta.vel":%.4f,"source":"%s"}}'
        % (motion["x.vel"], motion["y.vel"], motion["theta.vel"], source)
    )


def main():
    last_telemetry_ms = -TELEMETRY_INTERVAL_MS
    wait(30, MSEC)
    print("\\033[2J")

    while True:
        now_ms = brain.timer.system()
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
        self.telemetry_slot = max(1, min(8, int(telemetry_slot)))
        self.cache_dir = Path(cache_dir).expanduser()
        self.vexcom_path = detect_vexcom_path(requested_vexcom_path)
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

    def launch_replay(self, samples: list[dict[str, Any]]) -> bool:
        if not self.enabled or self.vexcom_path is None or self.comm_port is None:
            return False

        replay_samples = _extract_replay_samples(samples)
        program_source = build_vex_replay_program_source(replay_samples, self.control_config)
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
