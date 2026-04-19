#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import re
import subprocess
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import zmq

from lekiwi_runtime import ARM_STATE_KEYS, ArmSafetyFilter, WRIST_ROLL_KEY, align_degrees_near_reference

BASE_STATE_KEYS = ("x.vel", "y.vel", "theta.vel")
POWER_LINE_RE = re.compile(
    r"current≈(?P<current>[0-9.]+)A voltage≈(?P<voltage>[0-9.]+)V .* battery≈(?P<battery>[0-9.]+)%"
)

DEFAULT_CMD_PORT = 5565
DEFAULT_OBS_PORT = 5566
DEFAULT_RATE_HZ = 20.0
DEFAULT_CASE_HOLD_S = 0.6
DEFAULT_SETTLE_S = 1.0
DEFAULT_MIN_VOLTAGE_V = 11.0
DEFAULT_WATCHDOG_TIMEOUT_MS = 500
WRIST_SEAM_LEADER_VALUES = (
    170.0,
    175.0,
    179.0,
    -179.0,
    -175.0,
    -170.0,
    -165.0,
    -160.0,
    -165.0,
    -170.0,
    -175.0,
    -179.0,
    179.0,
    175.0,
    170.0,
)


@dataclass
class ProbeCase:
    name: str
    action: dict[str, float]
    expected: dict[str, float]
    final_observation: dict[str, float]
    max_abs_error: dict[str, float]
    final_error: dict[str, float]
    pass_thresholds: dict[str, float]
    passed: bool


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run a synthetic leader-motion probe against the Pi-side LeKiwi host and validate "
            "live follower tracking without the physical leader arm."
        )
    )
    parser.add_argument("--host-script", default=str(Path(__file__).with_name("lekiwi_host.py")))
    parser.add_argument("--robot-id", default="follow-mobile")
    parser.add_argument("--robot-port", default="/dev/ttyACM0")
    parser.add_argument("--torque-limits-path", default=str(Path.home() / ".lekiwi-ui" / "torque_limits.json"))
    parser.add_argument("--port-zmq-cmd", type=int, default=DEFAULT_CMD_PORT)
    parser.add_argument("--port-zmq-observations", type=int, default=DEFAULT_OBS_PORT)
    parser.add_argument("--connection-time-s", type=int, default=45)
    parser.add_argument("--rate-hz", type=float, default=DEFAULT_RATE_HZ)
    parser.add_argument("--case-hold-s", type=float, default=DEFAULT_CASE_HOLD_S)
    parser.add_argument("--settle-s", type=float, default=DEFAULT_SETTLE_S)
    parser.add_argument("--min-voltage-v", type=float, default=DEFAULT_MIN_VOLTAGE_V)
    parser.add_argument("--watchdog-timeout-ms", type=int, default=DEFAULT_WATCHDOG_TIMEOUT_MS)
    parser.add_argument("--output", default="", help="Optional path for a JSON summary.")
    return parser.parse_args()


def read_observation(sock: zmq.Socket, timeout_s: float) -> dict[str, Any]:
    poller = zmq.Poller()
    poller.register(sock, zmq.POLLIN)
    deadline = time.monotonic() + timeout_s
    latest: dict[str, Any] | None = None

    while time.monotonic() < deadline:
        remaining_ms = max(int((deadline - time.monotonic()) * 1000), 1)
        if not poller.poll(remaining_ms):
            break
        latest = json.loads(sock.recv_string())

    if latest is None:
        raise TimeoutError(f"No observation received within {timeout_s:.2f}s.")
    return latest


def extract_action_from_observation(observation: dict[str, Any]) -> dict[str, float]:
    action: dict[str, float] = {}
    for key in ARM_STATE_KEYS:
        value = observation.get(key)
        if not isinstance(value, (int, float)):
            raise ValueError(f"Observation is missing numeric {key}.")
        action[key] = float(value)
    for key in BASE_STATE_KEYS:
        action[key] = 0.0
    return action


def continuous_error(observed: float, expected: float) -> float:
    return align_degrees_near_reference(observed, expected) - expected


def parse_power_metrics(lines: list[str]) -> dict[str, float | None]:
    voltages: list[float] = []
    currents: list[float] = []
    batteries: list[float] = []

    for line in lines:
        match = POWER_LINE_RE.search(line)
        if not match:
            continue
        voltages.append(float(match.group("voltage")))
        currents.append(float(match.group("current")))
        batteries.append(float(match.group("battery")))

    return {
        "min_voltage_v": min(voltages) if voltages else None,
        "max_current_a": max(currents) if currents else None,
        "min_battery_percent": min(batteries) if batteries else None,
    }


def find_host_errors(lines: list[str]) -> list[str]:
    return [
        line
        for line in lines
        if "Message fetching failed" in line
        or "Traceback" in line
        or ("error" in line.lower() and "no command available" not in line.lower())
    ]


def build_joint_action(base_action: dict[str, float], **updates: float) -> dict[str, float]:
    action = dict(base_action)
    action.update(updates)
    return action


def build_joint_probe_plan(base_action: dict[str, float]) -> list[tuple[str, dict[str, float], dict[str, float]]]:
    return [
        (
            "hold-baseline",
            build_joint_action(base_action),
            {key: 4.0 for key in ARM_STATE_KEYS},
        ),
        (
            "shoulder-pan-plus",
            build_joint_action(base_action, **{"arm_shoulder_pan.pos": base_action["arm_shoulder_pan.pos"] + 6.0}),
            {"arm_shoulder_pan.pos": 8.0},
        ),
        (
            "shoulder-pan-home",
            build_joint_action(base_action),
            {"arm_shoulder_pan.pos": 8.0},
        ),
        (
            "shoulder-lift-plus",
            build_joint_action(base_action, **{"arm_shoulder_lift.pos": base_action["arm_shoulder_lift.pos"] + 6.0}),
            {"arm_shoulder_lift.pos": 8.0},
        ),
        (
            "elbow-flex-minus",
            build_joint_action(base_action, **{"arm_elbow_flex.pos": base_action["arm_elbow_flex.pos"] - 6.0}),
            {"arm_elbow_flex.pos": 8.0},
        ),
        (
            "wrist-flex-plus",
            build_joint_action(base_action, **{"arm_wrist_flex.pos": base_action["arm_wrist_flex.pos"] + 8.0}),
            {"arm_wrist_flex.pos": 10.0},
        ),
        (
            "gripper-nudge",
            build_joint_action(base_action, **{"arm_gripper.pos": base_action["arm_gripper.pos"] + 2.0}),
            {"arm_gripper.pos": 5.0},
        ),
        (
            "lift-proxy",
            build_joint_action(
                base_action,
                **{
                    "arm_shoulder_lift.pos": base_action["arm_shoulder_lift.pos"] + 6.0,
                    "arm_elbow_flex.pos": base_action["arm_elbow_flex.pos"] - 6.0,
                    "arm_wrist_flex.pos": base_action["arm_wrist_flex.pos"] + 4.0,
                },
            ),
            {
                "arm_shoulder_lift.pos": 10.0,
                "arm_elbow_flex.pos": 10.0,
                "arm_wrist_flex.pos": 10.0,
            },
        ),
        (
            "return-home",
            build_joint_action(base_action),
            {key: 5.0 for key in ARM_STATE_KEYS},
        ),
    ]


def launch_host(
    args: argparse.Namespace,
    *,
    cmd_port: int,
    obs_port: int,
) -> tuple[subprocess.Popen[str], list[str], threading.Thread, zmq.Context, zmq.Socket, zmq.Socket]:
    ctx = zmq.Context()
    cmd_socket = ctx.socket(zmq.PUSH)
    cmd_socket.setsockopt(zmq.CONFLATE, 1)
    cmd_socket.setsockopt(zmq.LINGER, 0)
    obs_socket = ctx.socket(zmq.PULL)
    obs_socket.setsockopt(zmq.CONFLATE, 1)
    obs_socket.setsockopt(zmq.LINGER, 0)

    cmd_socket.connect(f"tcp://127.0.0.1:{cmd_port}")
    obs_socket.connect(f"tcp://127.0.0.1:{obs_port}")

    host_cmd = [
        "python",
        args.host_script,
        "--connection-time-s",
        str(args.connection_time_s),
        "--robot-id",
        args.robot_id,
        "--robot-port",
        args.robot_port,
        "--robot-cameras-json",
        "{}",
        "--use-degrees",
        "true",
        "--watchdog-timeout-ms",
        str(args.watchdog_timeout_ms),
        "--port-zmq-cmd",
        str(cmd_port),
        "--port-zmq-observations",
        str(obs_port),
        "--torque-limits-path",
        args.torque_limits_path,
        "--base-max-raw-velocity",
        "6000",
        "--base-wheel-torque-limit",
        "700",
        "--enable-base",
        "false",
        "--safer-servo-mode",
    ]

    host_lines: list[str] = []
    process = subprocess.Popen(
        host_cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )

    def read_host_output() -> None:
        assert process.stdout is not None
        for line in process.stdout:
            host_lines.append(line.rstrip())

    host_thread = threading.Thread(target=read_host_output, daemon=True)
    host_thread.start()
    return process, host_lines, host_thread, ctx, cmd_socket, obs_socket


def shutdown_host(
    process: subprocess.Popen[str],
    host_thread: threading.Thread,
    ctx: zmq.Context,
    cmd_socket: zmq.Socket,
    obs_socket: zmq.Socket,
) -> None:
    try:
        cmd_socket.close()
    finally:
        obs_socket.close()
        ctx.term()
    process.terminate()
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=5)
    host_thread.join(timeout=2)


def execute_case(
    cmd_socket: zmq.Socket,
    obs_socket: zmq.Socket,
    safety_filter: ArmSafetyFilter,
    action: dict[str, float],
    hold_s: float,
    rate_hz: float,
    pass_thresholds: dict[str, float],
    name: str,
) -> ProbeCase:
    expected = safety_filter.normalize(dict(action))
    period_s = 1.0 / max(rate_hz, 1e-6)
    deadline = time.monotonic() + hold_s
    latest = read_observation(obs_socket, timeout_s=0.5)
    samples: list[dict[str, Any]] = []

    while time.monotonic() < deadline:
        cmd_socket.send_string(json.dumps(action))
        time.sleep(min(period_s * 0.5, 0.02))
        latest = read_observation(obs_socket, timeout_s=0.2)
        samples.append(latest)

    safety_filter.update(expected)

    max_abs_error: dict[str, float] = {}
    final_error: dict[str, float] = {}
    passed = True

    for key, threshold in pass_thresholds.items():
        errors: list[float] = []
        for sample in samples:
            observed = float(sample[key])
            desired = float(expected[key])
            error = continuous_error(observed, desired) if key == WRIST_ROLL_KEY else observed - desired
            errors.append(error)
        max_abs_error[key] = max(abs(error) for error in errors) if errors else float("inf")
        final_error[key] = errors[-1] if errors else float("inf")
        if max_abs_error[key] > threshold:
            passed = False

    return ProbeCase(
        name=name,
        action=action,
        expected=expected,
        final_observation={key: float(latest[key]) for key in ARM_STATE_KEYS},
        max_abs_error=max_abs_error,
        final_error=final_error,
        pass_thresholds=pass_thresholds,
        passed=passed,
    )


def run_joint_probe(args: argparse.Namespace) -> dict[str, Any]:
    process, host_lines, host_thread, ctx, cmd_socket, obs_socket = launch_host(
        args,
        cmd_port=args.port_zmq_cmd,
        obs_port=args.port_zmq_observations,
    )

    try:
        initial_observation = read_observation(obs_socket, timeout_s=args.settle_s + 4.0)
        base_action = extract_action_from_observation(initial_observation)
        safety_filter = ArmSafetyFilter(enabled=True, map_wrist_to_follower_start=True)
        safety_filter.seed_from_observation(initial_observation)

        cases = [
            execute_case(
                cmd_socket,
                obs_socket,
                safety_filter,
                action,
                args.case_hold_s,
                args.rate_hz,
                thresholds,
                name,
            )
            for name, action, thresholds in build_joint_probe_plan(base_action)
        ]

        power_metrics = parse_power_metrics(host_lines)
        host_error_lines = find_host_errors(host_lines)
        passed = all(case.passed for case in cases)
        min_voltage_v = power_metrics["min_voltage_v"]
        if isinstance(min_voltage_v, float) and min_voltage_v < args.min_voltage_v:
            passed = False
        if host_error_lines:
            passed = False

        return {
            "passed": passed,
            "initial_observation": initial_observation,
            "cases": [
                {
                    "name": case.name,
                    "passed": case.passed,
                    "action": case.action,
                    "expected": case.expected,
                    "final_observation": case.final_observation,
                    "max_abs_error": case.max_abs_error,
                    "final_error": case.final_error,
                    "pass_thresholds": case.pass_thresholds,
                }
                for case in cases
            ],
            "power_metrics": power_metrics,
            "host_error_lines": host_error_lines,
            "host_log_tail": host_lines[-40:],
        }
    finally:
        shutdown_host(process, host_thread, ctx, cmd_socket, obs_socket)


def run_wrist_offset_probe(args: argparse.Namespace) -> dict[str, Any]:
    process, host_lines, host_thread, ctx, cmd_socket, obs_socket = launch_host(
        args,
        cmd_port=args.port_zmq_cmd + 10,
        obs_port=args.port_zmq_observations + 10,
    )

    try:
        initial_observation = read_observation(obs_socket, timeout_s=args.settle_s + 4.0)
        base_action = extract_action_from_observation(initial_observation)
        safety_filter = ArmSafetyFilter(enabled=True, map_wrist_to_follower_start=True)
        safety_filter.seed_from_observation(initial_observation)

        period_s = 1.0 / max(args.rate_hz, 1e-6)
        hold_s = max(args.case_hold_s * 0.7, 0.35)
        steps: list[dict[str, float]] = []
        observed_wrist_history: list[float] = []
        previous_observed_wrist: float | None = None

        for leader_wrist in WRIST_SEAM_LEADER_VALUES:
            action = build_joint_action(base_action, **{WRIST_ROLL_KEY: leader_wrist})
            expected = safety_filter.normalize(dict(action))
            deadline = time.monotonic() + hold_s
            latest = initial_observation
            while time.monotonic() < deadline:
                cmd_socket.send_string(json.dumps(action))
                time.sleep(min(period_s * 0.5, 0.02))
                latest = read_observation(obs_socket, timeout_s=0.2)
                observed_wrist = float(latest[WRIST_ROLL_KEY])
                if previous_observed_wrist is not None:
                    observed_wrist = align_degrees_near_reference(observed_wrist, previous_observed_wrist)
                observed_wrist_history.append(observed_wrist)
                previous_observed_wrist = observed_wrist
            safety_filter.update(expected)
            expected_wrist = float(expected[WRIST_ROLL_KEY])
            observed_wrist = align_degrees_near_reference(float(latest[WRIST_ROLL_KEY]), expected_wrist)
            steps.append(
                {
                    "leader_wrist": leader_wrist,
                    "expected_wrist": expected_wrist,
                    "observed_wrist": observed_wrist,
                    "error_wrist": observed_wrist - expected_wrist,
                }
            )

        max_abs_error = max(abs(step["error_wrist"]) for step in steps)
        max_observed_wrist_jump = max(
            (abs(current - previous) for previous, current in zip(observed_wrist_history, observed_wrist_history[1:])),
            default=0.0,
        )
        power_metrics = parse_power_metrics(host_lines)
        host_error_lines = find_host_errors(host_lines)

        passed = max_abs_error <= 6.0 and max_observed_wrist_jump <= 12.0 and not host_error_lines
        min_voltage_v = power_metrics["min_voltage_v"]
        if isinstance(min_voltage_v, float) and min_voltage_v < args.min_voltage_v:
            passed = False

        return {
            "passed": passed,
            "initial_observation": initial_observation,
            "steps": steps,
            "max_abs_error_wrist": max_abs_error,
            "max_observed_wrist_jump_deg": max_observed_wrist_jump,
            "power_metrics": power_metrics,
            "host_error_lines": host_error_lines,
            "host_log_tail": host_lines[-40:],
        }
    finally:
        shutdown_host(process, host_thread, ctx, cmd_socket, obs_socket)


def main() -> None:
    args = parse_args()
    joint_probe = run_joint_probe(args)
    wrist_offset_probe = run_wrist_offset_probe(args)
    result = {
        "passed": bool(joint_probe["passed"] and wrist_offset_probe["passed"]),
        "joint_probe": joint_probe,
        "wrist_offset_probe": wrist_offset_probe,
    }
    if args.output:
        output_path = Path(args.output).expanduser()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        print(f"Wrote probe report to {output_path}", flush=True)
    print(json.dumps(result, indent=2), flush=True)
    if not result["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
