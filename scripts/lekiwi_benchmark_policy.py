#!/usr/bin/env python3

import argparse
import json
import time
from pathlib import Path

from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.policies.factory import make_pre_post_processors
from lerobot.processor import make_default_processors
from lerobot.robots.lekiwi import LeKiwi, LeKiwiConfig


def parse_bool(value: str) -> bool:
    normalized = value.strip().lower()
    if normalized in {"1", "true", "yes", "on"}:
        return True
    if normalized in {"0", "false", "no", "off"}:
        return False
    raise argparse.ArgumentTypeError("expected true or false")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Benchmark a deployed LeKiwi ACT policy on the Pi.")
    parser.add_argument("--robot-id", "--robot.id", dest="robot_id", default="follow-mobile")
    parser.add_argument("--robot-port", "--robot.port", dest="robot_port", default="/dev/ttyACM0")
    parser.add_argument("--robot-cameras-json", "--robot.cameras", dest="robot_cameras_json", default="default")
    parser.add_argument("--use-degrees", "--robot.use_degrees", dest="use_degrees", type=parse_bool, default=True)
    parser.add_argument("--enable-base", "--robot.enable_base", dest="enable_base", type=parse_bool, default=False)
    parser.add_argument("--policy-path", required=True)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--iterations", type=int, default=20)
    parser.add_argument("--warmup-iterations", type=int, default=2)
    return parser.parse_args()


def configure_cameras(robot_config: LeKiwiConfig, cameras_json: str) -> None:
    value = cameras_json.strip()
    if value in {"", "default", "__default__", "null"}:
        return
    parsed = json.loads(value)
    if parsed == {}:
        robot_config.cameras = {}
        return
    raise SystemExit("Only '{}' or 'default' are currently supported for --robot-cameras-json.")


def current_rss_mb() -> float | None:
    status = Path("/proc/self/status")
    if not status.exists():
        return None

    for line in status.read_text().splitlines():
        if line.startswith("VmRSS:"):
            parts = line.split()
            if len(parts) >= 2:
                return round(float(parts[1]) / 1024.0, 3)
    return None


def percentile(values: list[float], fraction: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = max(0, min(len(ordered) - 1, int(round((len(ordered) - 1) * fraction))))
    return ordered[index]


def main() -> None:
    args = parse_args()

    robot_config = LeKiwiConfig(
        id=args.robot_id,
        port=args.robot_port,
        use_degrees=args.use_degrees,
        enable_base=args.enable_base,
    )
    configure_cameras(robot_config, args.robot_cameras_json)

    robot = LeKiwi(robot_config)
    robot.connect()

    policy = ACTPolicy.from_pretrained(args.policy_path)
    preprocessor, postprocessor = make_pre_post_processors(
        policy_cfg=policy,
        pretrained_path=args.policy_path,
        preprocessor_overrides={"device_processor": {"device": str(policy.config.device)}},
    )
    _, _, robot_observation_processor = make_default_processors()

    latencies_ms: list[float] = []
    peak_rss_mb = current_rss_mb()
    total_start = time.perf_counter()

    try:
        for step in range(args.warmup_iterations + args.iterations):
            obs = robot.get_observation()
            processed_obs = robot_observation_processor(obs)

            step_start = time.perf_counter()
            policy_input = preprocessor(processed_obs)
            action = policy.select_action(policy_input)
            _ = postprocessor(action)
            step_latency_ms = (time.perf_counter() - step_start) * 1000.0

            if step >= args.warmup_iterations:
                latencies_ms.append(step_latency_ms)
                rss_mb = current_rss_mb()
                if rss_mb is not None:
                    peak_rss_mb = max(peak_rss_mb or rss_mb, rss_mb)
    finally:
        robot.disconnect()

    total_duration_s = max(time.perf_counter() - total_start, 1e-6)
    average_latency_ms = sum(latencies_ms) / len(latencies_ms) if latencies_ms else 0.0
    effective_fps = len(latencies_ms) / total_duration_s

    payload = {
        "targetFps": args.fps,
        "measuredAt": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "iterations": len(latencies_ms),
        "averageLatencyMs": round(average_latency_ms, 3),
        "p95LatencyMs": round(percentile(latencies_ms, 0.95), 3),
        "maxLatencyMs": round(max(latencies_ms) if latencies_ms else 0.0, 3),
        "effectiveFps": round(effective_fps, 3),
        "peakRssMb": peak_rss_mb,
        "passed": False,
    }
    print(json.dumps(payload))


if __name__ == "__main__":
    main()
