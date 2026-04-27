from __future__ import annotations

import sys
import unittest
import random
from pathlib import Path
from unittest import mock


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = ROOT_DIR / "scripts"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

import lekiwi_sensor_replay as sensor_replay
from lekiwi_sensor_replay import (
    GYRO_ROTATION_KEY,
    SENSOR_PREPOSITION_TIMEOUT_S,
    ULTRASONIC_X_KEY,
    ULTRASONIC_Y_KEY,
    SensorAwareReplayState,
    UltrasonicStabilityFilter,
    preposition_vex_base_to_recorded_state,
    recorded_state_has_sensor_reference,
)
from vex_base_bridge import VEX_MOTOR_STATE_KEYS
from vex_base_bridge import build_vex_telemetry_program_source
from vex_base_bridge import VexBaseBridge


def build_sample(
    *,
    t_s: float = 0.0,
    x_m: float = 1.0,
    y_m: float = 2.0,
    heading_deg: float = 30.0,
    pose_epoch: int = 1,
    x_vel: float = 0.2,
    y_vel: float = -0.1,
    theta_vel: float = 12.0,
    wheel_base: float = 100.0,
) -> dict[str, object]:
    state = {
        ULTRASONIC_X_KEY: x_m,
        ULTRASONIC_Y_KEY: y_m,
        GYRO_ROTATION_KEY: heading_deg,
        sensor_replay.VEX_POSE_EPOCH_KEY: pose_epoch,
        "x.vel": x_vel,
        "y.vel": y_vel,
        "theta.vel": theta_vel,
        "vex_front_right.pos": wheel_base + 1.0,
        "vex_front_left.pos": wheel_base + 2.0,
        "vex_rear_right.pos": wheel_base + 3.0,
        "vex_rear_left.pos": wheel_base + 4.0,
    }
    return {"t_s": t_s, "state": state}


class FakeObservationReader:
    def __init__(self, state: dict[str, float]) -> None:
        self.state = state

    def get_observation(self) -> dict[str, float]:
        return {}

    def get_sensor_status_snapshot(self) -> dict[str, dict[str, float | str]]:
        return {
            ULTRASONIC_X_KEY: {"state": "online", "value": self.state["x"]},
            ULTRASONIC_Y_KEY: {"state": "online", "value": self.state["y"]},
        }


class ScriptedUltrasonicObservationReader(FakeObservationReader):
    def __init__(
        self,
        state: dict[str, float],
        *,
        x_sequence: list[float] | None = None,
        y_sequence: list[float] | None = None,
    ) -> None:
        super().__init__(state)
        self.x_sequence = list(x_sequence or [])
        self.y_sequence = list(y_sequence or [])

    def get_sensor_status_snapshot(self) -> dict[str, dict[str, float | str]]:
        x_value = self.x_sequence.pop(0) if self.x_sequence else self.state["x"]
        y_value = self.y_sequence.pop(0) if self.y_sequence else self.state["y"]
        return {
            ULTRASONIC_X_KEY: {"state": "online", "value": x_value},
            ULTRASONIC_Y_KEY: {"state": "online", "value": y_value},
        }


class FakeVexBridge:
    def __init__(self, state: dict[str, float]) -> None:
        self.state = state
        self.commands: list[tuple[str, dict[str, float]]] = []
        self.motion_ttls_ms: list[int] = []

    def merge_observation(self, observation: dict[str, float]) -> dict[str, float]:
        return dict(observation)

    def gyro_status_snapshot(self) -> dict[str, float | str]:
        return {"state": "online", "value": self.state["heading"], "pose_epoch": self.state.get("pose_epoch", 1)}

    def send_motion(self, motion: dict[str, float], ttl_ms: int | None = None) -> bool:
        payload = {
            "x.vel": float(motion.get("x.vel", 0.0) or 0.0),
            "y.vel": float(motion.get("y.vel", 0.0) or 0.0),
            "theta.vel": float(motion.get("theta.vel", 0.0) or 0.0),
        }
        self.commands.append(("motion", payload))
        effective_ttl_ms = max(int(ttl_ms or sensor_replay.PREPOSITION_COMMAND_TTL_MS), 20)
        self.motion_ttls_ms.append(effective_ttl_ms)
        self._apply_motion(payload, effective_ttl_ms / 1000.0)
        return True

    def send_ecu_targets(
        self,
        targets: dict[str, float],
        *,
        motion: dict[str, float] | None = None,
        command_dt_ms: int = 20,
        ttl_ms: int | None = None,
    ) -> bool:
        payload = {
            "x.vel": float((motion or {}).get("x.vel", 0.0) or 0.0),
            "y.vel": float((motion or {}).get("y.vel", 0.0) or 0.0),
            "theta.vel": float((motion or {}).get("theta.vel", 0.0) or 0.0),
        }
        self.commands.append(("ecu", payload))
        self._apply_motion(payload, max(float(command_dt_ms) / 1000.0, 0.02))
        return True

    def send_hold(self, ttl_ms: int = 1200) -> bool:
        self.commands.append(("hold", {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}))
        return True

    def _apply_motion(self, motion: dict[str, float], dt_s: float) -> None:
        self.state["heading"] += motion["theta.vel"] * dt_s
        self.state["x"] -= motion["x.vel"] * dt_s
        self.state["y"] -= motion["y.vel"] * dt_s


class NoProgressVexBridge(FakeVexBridge):
    def _apply_motion(self, motion: dict[str, float], dt_s: float) -> None:
        return


class WrongYSignVexBridge(FakeVexBridge):
    def _apply_motion(self, motion: dict[str, float], dt_s: float) -> None:
        self.state["heading"] += motion["theta.vel"] * dt_s
        self.state["x"] -= motion["x.vel"] * dt_s
        self.state["y"] += motion["y.vel"] * dt_s


class WrongHeadingSignVexBridge(FakeVexBridge):
    def _apply_motion(self, motion: dict[str, float], dt_s: float) -> None:
        self.state["heading"] -= motion["theta.vel"] * dt_s
        self.state["x"] -= motion["x.vel"] * dt_s
        self.state["y"] -= motion["y.vel"] * dt_s


class HeadingCrossTargetVexBridge(FakeVexBridge):
    def __init__(self, state: dict[str, float]) -> None:
        super().__init__(state)
        self.heading_pulses = 0

    def _apply_motion(self, motion: dict[str, float], dt_s: float) -> None:
        if abs(motion["theta.vel"]) > 1e-6:
            self.heading_pulses += 1
            if self.heading_pulses == 1:
                self.state["heading"] = -2.4
                return
        super()._apply_motion(motion, dt_s)


class WideHeadingCrossTargetVexBridge(FakeVexBridge):
    def __init__(self, state: dict[str, float]) -> None:
        super().__init__(state)
        self.heading_pulses = 0

    def _apply_motion(self, motion: dict[str, float], dt_s: float) -> None:
        if abs(motion["theta.vel"]) > 1e-6:
            self.heading_pulses += 1
            if self.heading_pulses == 1:
                self.state["heading"] = 7.513
                return
        super()._apply_motion(motion, dt_s)


class NearTargetHeadingLagVexBridge(FakeVexBridge):
    def __init__(self, state: dict[str, float]) -> None:
        super().__init__(state)
        self.heading_pulses = 0

    def _apply_motion(self, motion: dict[str, float], dt_s: float) -> None:
        if abs(motion["theta.vel"]) > 1e-6:
            self.heading_pulses += 1
            if self.heading_pulses == 1:
                self.state["heading"] = 3.2
                return
        super()._apply_motion(motion, dt_s)


class LinearCrossTargetVexBridge(FakeVexBridge):
    def __init__(self, state: dict[str, float]) -> None:
        super().__init__(state)
        self.x_pulses = 0

    def _apply_motion(self, motion: dict[str, float], dt_s: float) -> None:
        if abs(motion["x.vel"]) > 1e-6:
            self.x_pulses += 1
            if self.x_pulses == 1:
                self.state["x"] = 0.8114
                return
        super()._apply_motion(motion, dt_s)


class RecoverableLinearCrossTargetVexBridge(FakeVexBridge):
    def __init__(self, state: dict[str, float]) -> None:
        super().__init__(state)
        self.x_pulses = 0

    def _apply_motion(self, motion: dict[str, float], dt_s: float) -> None:
        if abs(motion["x.vel"]) > 1e-6:
            self.x_pulses += 1
            if self.x_pulses == 1:
                self.state["x"] = 0.94
                return
            if self.x_pulses == 2:
                self.state["x"] = 1.0
                return
        super()._apply_motion(motion, dt_s)


class SlowUltrasonicProgressVexBridge(FakeVexBridge):
    def _apply_motion(self, motion: dict[str, float], dt_s: float) -> None:
        self.state["heading"] += motion["theta.vel"] * dt_s
        if abs(motion["x.vel"]) > 1e-6:
            self.state["x"] -= 0.001 if motion["x.vel"] > 0 else -0.001
        if abs(motion["y.vel"]) > 1e-6:
            self.state["y"] -= 0.001 if motion["y.vel"] > 0 else -0.001


class CommandWriteFailVexBridge(FakeVexBridge):
    def send_motion(self, motion: dict[str, float], ttl_ms: int | None = None) -> bool:
        payload = {
            "x.vel": float(motion.get("x.vel", 0.0) or 0.0),
            "y.vel": float(motion.get("y.vel", 0.0) or 0.0),
            "theta.vel": float(motion.get("theta.vel", 0.0) or 0.0),
        }
        self.commands.append(("failed-motion", payload))
        return False


class SensorReplayTests(unittest.TestCase):
    def _max_wheel_percent(self, motion: dict[str, float], replay_state: SensorAwareReplayState) -> float:
        wheel_pct = sensor_replay._drive_wheel_percentages(
            motion,
            max_linear_speed_mps=replay_state.max_linear_speed_mps,
            max_turn_speed_dps=replay_state.max_turn_speed_dps,
        )
        return max(abs(value) for value in wheel_pct.values())

    def _command_axis_order(self, commands: list[tuple[str, dict[str, float]]]) -> list[str]:
        axis_order: list[str] = []
        for command_type, motion in commands:
            if command_type == "hold":
                continue
            if abs(motion["theta.vel"]) > 1e-6:
                axis = "heading"
            elif abs(motion["x.vel"]) > 1e-6:
                axis = "x"
            elif abs(motion["y.vel"]) > 1e-6:
                axis = "y"
            else:
                continue
            if not axis_order or axis_order[-1] != axis:
                axis_order.append(axis)
        return axis_order

    def test_feedback_ignores_stale_or_held_ultrasonic_values(self) -> None:
        sample = build_sample(x_m=1.0, y_m=2.0, heading_deg=0.0, pose_epoch=1)
        replay_state = SensorAwareReplayState(
            [sample],
            replay_mode="drive",
            speed=1.0,
            control_config={"tuning": {"maxLinearSpeedMps": 0.35, "maxTurnSpeedDps": 90.0}},
        )

        feedback = replay_state.feedback_context(
            sample["state"],  # type: ignore[arg-type]
            live_sensor_status={
                ULTRASONIC_X_KEY: {"state": "online", "value": 1.4, "fresh": False},
                ULTRASONIC_Y_KEY: {"state": "stale", "value": 2.4},
            },
            live_gyro_status={"state": "online", "value": 0.0, "pose_epoch": 1},
        )

        self.assertNotIn("x", feedback["available_axes"])
        self.assertNotIn("y", feedback["available_axes"])
        self.assertIsNone(feedback["decision_x_error_m"])
        self.assertIsNone(feedback["decision_y_error_m"])

    def test_feedback_ignores_invalid_recorded_ultrasonic_zero_targets(self) -> None:
        sample = build_sample(x_m=0.0, y_m=0.0, heading_deg=0.0, pose_epoch=1)
        replay_state = SensorAwareReplayState(
            [sample],
            replay_mode="drive",
            speed=1.0,
            control_config={"tuning": {"maxLinearSpeedMps": 0.35, "maxTurnSpeedDps": 90.0}},
        )

        feedback = replay_state.feedback_context(
            sample["state"],  # type: ignore[arg-type]
            live_sensor_status={
                ULTRASONIC_X_KEY: {"state": "online", "value": 0.17},
                ULTRASONIC_Y_KEY: {"state": "online", "value": 1.75},
            },
            live_gyro_status={"state": "online", "value": 0.0, "pose_epoch": 1},
        )

        self.assertNotIn("x", feedback["available_axes"])
        self.assertNotIn("y", feedback["available_axes"])
        self.assertIsNone(feedback["recorded_x_m"])
        self.assertIsNone(feedback["recorded_y_m"])

    def _assert_motion_commands_are_axis_isolated(self, commands: list[tuple[str, dict[str, float]]]) -> None:
        for command_type, motion in commands:
            if command_type == "hold":
                continue
            nonzero_axes = [
                key
                for key in ("x.vel", "y.vel", "theta.vel")
                if abs(float(motion.get(key, 0.0) or 0.0)) > 1e-6
            ]
            self.assertLessEqual(len(nonzero_axes), 1, f"motion command leaked across axes: {motion}")

    def test_preposition_aligns_heading_then_x_then_y(self) -> None:
        samples = [build_sample()]
        replay_state = SensorAwareReplayState(
            samples,
            replay_mode="ecu",
            speed=1.0,
            control_config={"tuning": {"maxLinearSpeedMps": 0.35, "maxTurnSpeedDps": 90.0}},
        )
        live_state = {"x": 1.05, "y": 2.10, "heading": 35.0, "pose_epoch": 1}
        observation_reader = FakeObservationReader(live_state)
        vex_bridge = FakeVexBridge(live_state)

        with mock.patch.object(sensor_replay.time, "sleep", return_value=None):
            aligned = preposition_vex_base_to_recorded_state(
                vex_bridge,
                observation_reader,
                replay_state,
                samples[0]["state"],  # type: ignore[arg-type]
                timeout_s=SENSOR_PREPOSITION_TIMEOUT_S,
                settle_s=0.0,
            )

        self.assertTrue(aligned)

        self.assertEqual(self._command_axis_order(vex_bridge.commands), ["heading", "x", "y"])
        self._assert_motion_commands_are_axis_isolated(vex_bridge.commands)
        self.assertTrue(
            all(
                command_type == "motion"
                for command_type, _motion in vex_bridge.commands
                if command_type != "hold"
            )
        )
        self.assertAlmostEqual(live_state["heading"], 30.0, delta=1.5)
        self.assertAlmostEqual(live_state["x"], 1.0, delta=sensor_replay.XY_TRACK_TOLERANCE_M)
        self.assertAlmostEqual(live_state["y"], 2.0, delta=sensor_replay.XY_TRACK_TOLERANCE_M)

    def test_ultrasonic_filter_requires_confirmation_for_large_jump(self) -> None:
        axis_filter = UltrasonicStabilityFilter()

        self.assertAlmostEqual(axis_filter.observe(1.1048), 1.1048)
        self.assertAlmostEqual(axis_filter.observe(1.1046), 1.1047, places=4)
        self.assertAlmostEqual(axis_filter.observe(1.1041), 1.1046, places=4)
        self.assertGreater(axis_filter.observe(0.7343), 1.0)
        self.assertAlmostEqual(axis_filter.observe(1.1039), 1.1041, places=4)
        self.assertGreater(axis_filter.observe(0.7344), 1.0)
        self.assertGreater(axis_filter.observe(0.7342), 1.0)
        self.assertAlmostEqual(axis_filter.observe(0.7341), 1.0439, places=4)
        self.assertAlmostEqual(axis_filter.observe(0.7340), 1.0139, places=4)

    def test_preposition_ignores_brief_x_ultrasonic_outlier(self) -> None:
        samples = [build_sample(x_m=1.0, y_m=2.0, heading_deg=0.0, pose_epoch=1)]
        replay_state = SensorAwareReplayState(
            samples,
            replay_mode="drive",
            speed=1.0,
            control_config={"tuning": {"maxLinearSpeedMps": 0.35, "maxTurnSpeedDps": 90.0}},
        )
        live_state = {"x": 1.08, "y": 2.0, "heading": 0.0, "pose_epoch": 1}
        observation_reader = ScriptedUltrasonicObservationReader(
            live_state,
            x_sequence=[1.08, 1.079, 1.078, 0.7343, 1.076, 1.072, 1.068],
        )
        vex_bridge = FakeVexBridge(live_state)

        with mock.patch.object(sensor_replay.time, "sleep", return_value=None):
            aligned = preposition_vex_base_to_recorded_state(
                vex_bridge,
                observation_reader,
                replay_state,
                samples[0]["state"],  # type: ignore[arg-type]
                timeout_s=SENSOR_PREPOSITION_TIMEOUT_S,
                settle_s=0.0,
            )

        self.assertTrue(aligned)
        self.assertEqual(self._command_axis_order(vex_bridge.commands), ["x"])
        self.assertAlmostEqual(live_state["x"], 1.0, delta=sensor_replay.XY_TRACK_TOLERANCE_M)

    def test_preposition_uses_larger_pulses_for_large_linear_error(self) -> None:
        samples = [build_sample(x_m=1.0, y_m=2.0, heading_deg=0.0, pose_epoch=1)]
        replay_state = SensorAwareReplayState(
            samples,
            replay_mode="drive",
            speed=1.0,
            control_config={"tuning": {"maxLinearSpeedMps": 0.35, "maxTurnSpeedDps": 90.0}},
        )
        live_state = {"x": 1.14, "y": 2.0, "heading": 0.0, "pose_epoch": 1}
        observation_reader = FakeObservationReader(live_state)
        vex_bridge = FakeVexBridge(live_state)

        with mock.patch.object(sensor_replay.time, "sleep", return_value=None):
            aligned = preposition_vex_base_to_recorded_state(
                vex_bridge,
                observation_reader,
                replay_state,
                samples[0]["state"],  # type: ignore[arg-type]
                timeout_s=SENSOR_PREPOSITION_TIMEOUT_S,
                settle_s=0.0,
            )

        self.assertTrue(aligned)
        x_commands = [
            motion
            for command_type, motion in vex_bridge.commands
            if command_type != "hold" and abs(motion["x.vel"]) > 1e-6
        ]
        self.assertGreater(len(x_commands), 1)
        self.assertLessEqual(
            max(self._max_wheel_percent(motion, replay_state) for motion in x_commands),
            sensor_replay.PREPOSITION_MAX_WHEEL_PERCENT + 1e-6,
        )
        self.assertEqual(vex_bridge.motion_ttls_ms[0], sensor_replay.PREPOSITION_LARGE_PULSE_TTL_MS)
        self.assertTrue(all(ttl <= sensor_replay.PREPOSITION_LARGE_PULSE_TTL_MS for ttl in vex_bridge.motion_ttls_ms))
        self.assertAlmostEqual(live_state["x"], 1.0, delta=sensor_replay.XY_TRACK_TOLERANCE_M)

    def test_preposition_moves_away_when_ultrasonic_distance_is_too_close(self) -> None:
        samples = [build_sample(x_m=1.0, y_m=2.0, heading_deg=0.0, pose_epoch=1)]
        replay_state = SensorAwareReplayState(
            samples,
            replay_mode="drive",
            speed=1.0,
            control_config={"tuning": {"maxLinearSpeedMps": 0.35, "maxTurnSpeedDps": 90.0}},
        )
        live_state = {"x": 0.88, "y": 2.0, "heading": 0.0, "pose_epoch": 1}
        observation_reader = FakeObservationReader(live_state)
        vex_bridge = FakeVexBridge(live_state)

        with mock.patch.object(sensor_replay.time, "sleep", return_value=None):
            aligned = preposition_vex_base_to_recorded_state(
                vex_bridge,
                observation_reader,
                replay_state,
                samples[0]["state"],  # type: ignore[arg-type]
                timeout_s=SENSOR_PREPOSITION_TIMEOUT_S,
                settle_s=0.0,
            )

        self.assertTrue(aligned)
        x_commands = [
            motion
            for command_type, motion in vex_bridge.commands
            if command_type != "hold" and abs(motion["x.vel"]) > 1e-6
        ]
        self.assertGreater(len(x_commands), 1)
        self.assertLessEqual(
            max(self._max_wheel_percent(motion, replay_state) for motion in x_commands),
            sensor_replay.PREPOSITION_MAX_WHEEL_PERCENT + 1e-6,
        )
        self.assertLess(x_commands[0]["x.vel"], 0.0)
        self.assertAlmostEqual(live_state["x"], 1.0, delta=sensor_replay.XY_TRACK_TOLERANCE_M)

    def test_preposition_rejects_persistent_large_linear_cross_target(self) -> None:
        samples = [build_sample(x_m=1.0, y_m=2.0, heading_deg=0.0, pose_epoch=1)]
        replay_state = SensorAwareReplayState(
            samples,
            replay_mode="drive",
            speed=1.0,
            control_config={"tuning": {"maxLinearSpeedMps": 0.35, "maxTurnSpeedDps": 90.0}},
        )
        live_state = {"x": 1.08, "y": 2.08, "heading": 0.0, "pose_epoch": 1}
        observation_reader = FakeObservationReader(live_state)
        vex_bridge = LinearCrossTargetVexBridge(live_state)

        with mock.patch.object(sensor_replay.time, "sleep", return_value=None):
            aligned = preposition_vex_base_to_recorded_state(
                vex_bridge,
                observation_reader,
                replay_state,
                samples[0]["state"],  # type: ignore[arg-type]
                timeout_s=SENSOR_PREPOSITION_TIMEOUT_S,
                settle_s=0.0,
            )

        self.assertFalse(aligned)
        self.assertEqual(aligned.axis, "x")
        self.assertLess(live_state["x"], 0.9)

    def test_preposition_uses_custom_linear_trim_tolerance(self) -> None:
        samples = [build_sample(x_m=1.0, y_m=2.0, heading_deg=0.0, pose_epoch=1)]
        replay_state = SensorAwareReplayState(
            samples,
            replay_mode="drive",
            speed=1.0,
            control_config={"tuning": {"maxLinearSpeedMps": 0.35, "maxTurnSpeedDps": 90.0}},
        )
        live_state = {"x": 1.08, "y": 2.0, "heading": 0.0, "pose_epoch": 1}
        observation_reader = FakeObservationReader(live_state)
        vex_bridge = RecoverableLinearCrossTargetVexBridge(live_state)

        with mock.patch.object(sensor_replay.time, "sleep", return_value=None):
            aligned = preposition_vex_base_to_recorded_state(
                vex_bridge,
                observation_reader,
                replay_state,
                samples[0]["state"],  # type: ignore[arg-type]
                timeout_s=SENSOR_PREPOSITION_TIMEOUT_S,
                settle_s=0.0,
                xy_trim_tolerance_m=0.08,
            )

        self.assertTrue(aligned)
        self.assertAlmostEqual(live_state["x"], 1.0, delta=sensor_replay.XY_TRACK_TOLERANCE_M)

    def test_preposition_reports_command_write_failure_instead_of_no_progress(self) -> None:
        samples = [build_sample(x_m=1.0, y_m=2.0, heading_deg=0.0, pose_epoch=1)]
        replay_state = SensorAwareReplayState(
            samples,
            replay_mode="drive",
            speed=1.0,
            control_config={"tuning": {"maxLinearSpeedMps": 0.35, "maxTurnSpeedDps": 90.0}},
        )
        live_state = {"x": 1.08, "y": 2.0, "heading": 0.0, "pose_epoch": 1}
        observation_reader = FakeObservationReader(live_state)
        vex_bridge = CommandWriteFailVexBridge(live_state)

        with mock.patch.object(sensor_replay.time, "sleep", return_value=None):
            aligned = preposition_vex_base_to_recorded_state(
                vex_bridge,
                observation_reader,
                replay_state,
                samples[0]["state"],  # type: ignore[arg-type]
                timeout_s=SENSOR_PREPOSITION_TIMEOUT_S,
                settle_s=0.0,
            )

        self.assertFalse(aligned)
        self.assertEqual(aligned.reason, "command-write-failed")
        self.assertEqual(aligned.axis, "x")
        self.assertEqual(vex_bridge.commands[-1][0], "hold")

    def test_preposition_trims_small_heading_cross_target_then_continues_to_ultrasonic_axes(self) -> None:
        samples = [build_sample(x_m=1.0, y_m=2.0, heading_deg=0.0, pose_epoch=1)]
        replay_state = SensorAwareReplayState(
            samples,
            replay_mode="drive",
            speed=1.0,
            control_config={"tuning": {"maxLinearSpeedMps": 0.35, "maxTurnSpeedDps": 90.0}},
        )
        live_state = {"x": 1.07, "y": 2.09, "heading": 5.0, "pose_epoch": 1}
        observation_reader = FakeObservationReader(live_state)
        vex_bridge = HeadingCrossTargetVexBridge(live_state)

        with mock.patch.object(sensor_replay.time, "sleep", return_value=None):
            aligned = preposition_vex_base_to_recorded_state(
                vex_bridge,
                observation_reader,
                replay_state,
                samples[0]["state"],  # type: ignore[arg-type]
                timeout_s=SENSOR_PREPOSITION_TIMEOUT_S,
                settle_s=0.0,
            )

        self.assertTrue(aligned)
        self.assertEqual(self._command_axis_order(vex_bridge.commands), ["heading", "x", "y"])
        self.assertAlmostEqual(live_state["heading"], 0.0, delta=1.5)
        self.assertAlmostEqual(live_state["x"], 1.0, delta=sensor_replay.XY_TRACK_TOLERANCE_M)
        self.assertAlmostEqual(live_state["y"], 2.0, delta=sensor_replay.XY_TRACK_TOLERANCE_M)

    def test_preposition_flips_heading_direction_when_first_pulse_moves_wrong_way(self) -> None:
        samples = [build_sample(x_m=1.0, y_m=2.0, heading_deg=30.0, pose_epoch=1)]
        replay_state = SensorAwareReplayState(
            samples,
            replay_mode="drive",
            speed=1.0,
            control_config={"tuning": {"maxLinearSpeedMps": 0.35, "maxTurnSpeedDps": 90.0}},
        )
        live_state = {"x": 1.0, "y": 2.0, "heading": 50.0, "pose_epoch": 1}
        observation_reader = FakeObservationReader(live_state)
        vex_bridge = WrongHeadingSignVexBridge(live_state)

        with mock.patch.object(sensor_replay.time, "sleep", return_value=None):
            aligned = preposition_vex_base_to_recorded_state(
                vex_bridge,
                observation_reader,
                replay_state,
                samples[0]["state"],  # type: ignore[arg-type]
                timeout_s=SENSOR_PREPOSITION_TIMEOUT_S,
                settle_s=0.0,
            )

        self.assertTrue(aligned)
        heading_commands = [
            motion["theta.vel"]
            for command_type, motion in vex_bridge.commands
            if command_type != "hold" and abs(motion["theta.vel"]) > 1e-6
        ]
        self.assertGreaterEqual(len(heading_commands), 2)
        self.assertLess(heading_commands[0], 0.0)
        self.assertGreater(heading_commands[1], 0.0)
        self.assertAlmostEqual(live_state["heading"], 30.0, delta=1.5)

    def test_preposition_does_not_flip_direction_during_close_heading_trim(self) -> None:
        samples = [build_sample(x_m=1.0, y_m=2.0, heading_deg=0.0, pose_epoch=1)]
        replay_state = SensorAwareReplayState(
            samples,
            replay_mode="drive",
            speed=1.0,
            control_config={"tuning": {"maxLinearSpeedMps": 0.35, "maxTurnSpeedDps": 90.0}},
        )
        live_state = {"x": 1.0, "y": 2.0, "heading": 2.53, "pose_epoch": 1}
        observation_reader = FakeObservationReader(live_state)
        vex_bridge = NearTargetHeadingLagVexBridge(live_state)

        with mock.patch.object(sensor_replay.time, "sleep", return_value=None):
            aligned = preposition_vex_base_to_recorded_state(
                vex_bridge,
                observation_reader,
                replay_state,
                samples[0]["state"],  # type: ignore[arg-type]
                timeout_s=SENSOR_PREPOSITION_TIMEOUT_S,
                settle_s=0.0,
            )

        self.assertTrue(aligned)
        heading_commands = [
            motion["theta.vel"]
            for command_type, motion in vex_bridge.commands
            if command_type != "hold" and abs(motion["theta.vel"]) > 1e-6
        ]
        self.assertGreaterEqual(len(heading_commands), 2)
        self.assertTrue(all(command < 0.0 for command in heading_commands))
        self.assertAlmostEqual(live_state["heading"], 0.0, delta=1.5)

    def test_preposition_trims_recoverable_wide_heading_cross_target(self) -> None:
        samples = [build_sample(x_m=1.0, y_m=2.0, heading_deg=0.0, pose_epoch=1)]
        replay_state = SensorAwareReplayState(
            samples,
            replay_mode="drive",
            speed=1.0,
            control_config={"tuning": {"maxLinearSpeedMps": 0.35, "maxTurnSpeedDps": 90.0}},
        )
        live_state = {"x": 1.0, "y": 2.0, "heading": -7.83, "pose_epoch": 1}
        observation_reader = FakeObservationReader(live_state)
        vex_bridge = WideHeadingCrossTargetVexBridge(live_state)

        with mock.patch.object(sensor_replay.time, "sleep", return_value=None):
            aligned = preposition_vex_base_to_recorded_state(
                vex_bridge,
                observation_reader,
                replay_state,
                samples[0]["state"],  # type: ignore[arg-type]
                timeout_s=SENSOR_PREPOSITION_TIMEOUT_S,
                settle_s=0.0,
            )

        self.assertTrue(aligned)
        self.assertEqual(self._command_axis_order(vex_bridge.commands), ["heading"])
        self.assertGreater(vex_bridge.heading_pulses, 1)
        self.assertLessEqual(vex_bridge.motion_ttls_ms[0], sensor_replay.PREPOSITION_HEADING_MEDIUM_PULSE_TTL_MS)
        self.assertAlmostEqual(live_state["heading"], 0.0, delta=1.5)

    def test_simulated_randomized_start_pose_alignment(self) -> None:
        rng = random.Random(20260425)
        samples = [build_sample(x_m=0.78, y_m=1.33, heading_deg=0.0, pose_epoch=1)]

        for case_index in range(25):
            replay_state = SensorAwareReplayState(
                samples,
                replay_mode="drive",
                speed=1.0,
                control_config={"tuning": {"maxLinearSpeedMps": 0.35, "maxTurnSpeedDps": 90.0}},
            )
            live_state = {
                "x": 0.78 + rng.choice((-1.0, 1.0)) * rng.uniform(0.04, 0.12),
                "y": 1.33 + rng.choice((-1.0, 1.0)) * rng.uniform(0.04, 0.12),
                "heading": rng.choice((-1.0, 1.0)) * rng.uniform(4.0, 18.0),
                "pose_epoch": 1,
            }
            observation_reader = FakeObservationReader(live_state)
            vex_bridge = FakeVexBridge(live_state)

            with self.subTest(case=case_index):
                with mock.patch.object(sensor_replay.time, "sleep", return_value=None):
                    aligned = preposition_vex_base_to_recorded_state(
                        vex_bridge,
                        observation_reader,
                        replay_state,
                        samples[0]["state"],  # type: ignore[arg-type]
                        timeout_s=SENSOR_PREPOSITION_TIMEOUT_S,
                        settle_s=0.0,
                    )

                self.assertTrue(aligned)
                axis_order = self._command_axis_order(vex_bridge.commands)
                self.assertLessEqual(axis_order.index("heading"), axis_order.index("x"))
                self.assertLessEqual(axis_order.index("x"), axis_order.index("y"))
                self.assertAlmostEqual(live_state["heading"], 0.0, delta=1.5)
                self.assertAlmostEqual(live_state["x"], 0.78, delta=sensor_replay.XY_TRACK_TOLERANCE_M)
                self.assertAlmostEqual(live_state["y"], 1.33, delta=sensor_replay.XY_TRACK_TOLERANCE_M)

    def test_preposition_requires_live_marked_gyro_origin_before_ultrasonic_axes(self) -> None:
        samples = [build_sample(heading_deg=-47.6121, pose_epoch=1)]
        replay_state = SensorAwareReplayState(
            samples,
            replay_mode="drive",
            speed=1.0,
            control_config={"tuning": {"maxLinearSpeedMps": 0.35, "maxTurnSpeedDps": 90.0}},
        )
        live_state = {"x": 1.05, "y": 2.10, "heading": 0.0, "pose_epoch": 0}
        observation_reader = FakeObservationReader(live_state)
        vex_bridge = FakeVexBridge(live_state)

        with mock.patch.object(sensor_replay.time, "sleep", return_value=None):
            aligned = preposition_vex_base_to_recorded_state(
                vex_bridge,
                observation_reader,
                replay_state,
                samples[0]["state"],  # type: ignore[arg-type]
                timeout_s=SENSOR_PREPOSITION_TIMEOUT_S,
                settle_s=0.0,
            )

        self.assertFalse(aligned)
        self.assertEqual(aligned.reason, "live-pose-origin-unzeroed")
        self.assertEqual(aligned.axis, "heading")
        self.assertTrue(
            all(abs(motion["theta.vel"]) <= 1e-6 for command_type, motion in vex_bridge.commands if command_type != "hold")
        )
        self.assertAlmostEqual(live_state["heading"], 0.0, delta=1e-6)
        self.assertAlmostEqual(live_state["x"], 1.05, delta=1e-6)
        self.assertAlmostEqual(live_state["y"], 2.10, delta=1e-6)

    def test_preposition_aborts_recorded_heading_without_pose_epoch(self) -> None:
        sample = build_sample(heading_deg=12.0, pose_epoch=1)
        state = dict(sample["state"])  # type: ignore[arg-type]
        state.pop(sensor_replay.VEX_POSE_EPOCH_KEY, None)
        samples = [{"t_s": 0.0, "state": state}]
        replay_state = SensorAwareReplayState(
            samples,
            replay_mode="drive",
            speed=1.0,
            control_config={"tuning": {"maxLinearSpeedMps": 0.35, "maxTurnSpeedDps": 90.0}},
        )
        live_state = {"x": 1.05, "y": 2.10, "heading": 12.0, "pose_epoch": 1}
        observation_reader = FakeObservationReader(live_state)
        vex_bridge = FakeVexBridge(live_state)

        with mock.patch.object(sensor_replay.time, "sleep", return_value=None):
            aligned = preposition_vex_base_to_recorded_state(
                vex_bridge,
                observation_reader,
                replay_state,
                state,
                timeout_s=SENSOR_PREPOSITION_TIMEOUT_S,
                settle_s=0.0,
            )

        self.assertFalse(aligned)
        self.assertEqual(aligned.reason, "recorded-pose-epoch-missing")
        self.assertEqual(aligned.axis, "heading")
        self.assertEqual(self._command_axis_order(vex_bridge.commands), [])

    def test_preposition_allows_different_positive_pose_epochs_after_rezero(self) -> None:
        samples = [build_sample(heading_deg=0.0, pose_epoch=1)]
        replay_state = SensorAwareReplayState(
            samples,
            replay_mode="drive",
            speed=1.0,
            control_config={"tuning": {"maxLinearSpeedMps": 0.35, "maxTurnSpeedDps": 90.0}},
        )
        live_state = {"x": 1.0, "y": 2.0, "heading": 8.0, "pose_epoch": 4}
        observation_reader = FakeObservationReader(live_state)
        vex_bridge = FakeVexBridge(live_state)

        with mock.patch.object(sensor_replay.time, "sleep", return_value=None):
            aligned = preposition_vex_base_to_recorded_state(
                vex_bridge,
                observation_reader,
                replay_state,
                samples[0]["state"],  # type: ignore[arg-type]
                timeout_s=SENSOR_PREPOSITION_TIMEOUT_S,
                settle_s=0.0,
            )

        self.assertTrue(aligned)
        self.assertEqual(self._command_axis_order(vex_bridge.commands), ["heading"])
        self.assertAlmostEqual(live_state["heading"], 0.0, delta=1.5)

    def test_preposition_trusts_rezeroed_manual_origin(self) -> None:
        samples = [build_sample(heading_deg=0.0, pose_epoch=1)]
        replay_state = SensorAwareReplayState(
            samples,
            replay_mode="drive",
            speed=1.0,
            control_config={"tuning": {"maxLinearSpeedMps": 0.35, "maxTurnSpeedDps": 90.0}},
        )
        live_state = {"x": 1.0, "y": 2.0, "heading": 5.0, "pose_epoch": 1}
        observation_reader = FakeObservationReader(live_state)
        vex_bridge = FakeVexBridge(live_state)

        with mock.patch.object(sensor_replay.time, "sleep", return_value=None):
            aligned = preposition_vex_base_to_recorded_state(
                vex_bridge,
                observation_reader,
                replay_state,
                samples[0]["state"],  # type: ignore[arg-type]
                timeout_s=SENSOR_PREPOSITION_TIMEOUT_S,
                settle_s=0.0,
            )

        self.assertTrue(aligned)
        self.assertAlmostEqual(live_state["heading"], 0.0, delta=1.5)

    def test_preposition_uses_marked_gyro_zero_for_ultrasonic_only_recording(self) -> None:
        start_state = {
            ULTRASONIC_X_KEY: 1.0,
            ULTRASONIC_Y_KEY: 2.0,
            GYRO_ROTATION_KEY: 0.0,
            sensor_replay.VEX_POSE_EPOCH_KEY: 1,
            "x.vel": 0.0,
            "y.vel": 0.0,
            "theta.vel": 0.0,
        }
        samples = [{"t_s": 0.0, "state": start_state}]
        replay_state = SensorAwareReplayState(
            samples,
            replay_mode="drive",
            speed=1.0,
            control_config={"tuning": {"maxLinearSpeedMps": 0.35, "maxTurnSpeedDps": 90.0}},
        )
        live_state = {"x": 1.07, "y": 2.08, "heading": 14.0, "pose_epoch": 1}
        observation_reader = FakeObservationReader(live_state)
        vex_bridge = FakeVexBridge(live_state)

        with mock.patch.object(sensor_replay.time, "sleep", return_value=None):
            aligned = preposition_vex_base_to_recorded_state(
                vex_bridge,
                observation_reader,
                replay_state,
                start_state,
                timeout_s=SENSOR_PREPOSITION_TIMEOUT_S,
                settle_s=0.0,
            )

        self.assertTrue(aligned)
        self.assertEqual(self._command_axis_order(vex_bridge.commands), ["heading", "x", "y"])
        self.assertAlmostEqual(live_state["heading"], 0.0, delta=1.5)
        self.assertAlmostEqual(live_state["x"], 1.0, delta=sensor_replay.XY_TRACK_TOLERANCE_M)
        self.assertAlmostEqual(live_state["y"], 2.0, delta=sensor_replay.XY_TRACK_TOLERANCE_M)

    def test_preposition_stops_when_ultrasonic_reading_does_not_change(self) -> None:
        samples = [build_sample(y_m=2.0, heading_deg=0.0, pose_epoch=1)]
        replay_state = SensorAwareReplayState(
            samples,
            replay_mode="drive",
            speed=1.0,
            control_config={"tuning": {"maxLinearSpeedMps": 0.35, "maxTurnSpeedDps": 90.0}},
        )
        live_state = {"x": 1.0, "y": 2.12, "heading": 0.0, "pose_epoch": 1}
        observation_reader = FakeObservationReader(live_state)
        vex_bridge = NoProgressVexBridge(live_state)

        with mock.patch.object(sensor_replay.time, "sleep", return_value=None):
            aligned = preposition_vex_base_to_recorded_state(
                vex_bridge,
                observation_reader,
                replay_state,
                samples[0]["state"],  # type: ignore[arg-type]
                timeout_s=SENSOR_PREPOSITION_TIMEOUT_S,
                settle_s=0.0,
            )

        self.assertFalse(aligned)
        self.assertLessEqual(
            len([command for command in vex_bridge.commands if command[0] != "hold"]),
            sensor_replay.PREPOSITION_MAX_AXIS_COMMANDS_WITHOUT_PROGRESS + 1,
        )
        self.assertEqual(vex_bridge.commands[-1][0], "hold")

    def test_preposition_stops_when_y_axis_error_gets_worse(self) -> None:
        samples = [build_sample(y_m=2.0, heading_deg=0.0, pose_epoch=1)]
        replay_state = SensorAwareReplayState(
            samples,
            replay_mode="drive",
            speed=1.0,
            control_config={"tuning": {"maxLinearSpeedMps": 0.35, "maxTurnSpeedDps": 90.0}},
        )
        live_state = {"x": 1.0, "y": 2.12, "heading": 0.0, "pose_epoch": 1}
        observation_reader = FakeObservationReader(live_state)
        vex_bridge = WrongYSignVexBridge(live_state)

        with mock.patch.object(sensor_replay.time, "sleep", return_value=None):
            aligned = preposition_vex_base_to_recorded_state(
                vex_bridge,
                observation_reader,
                replay_state,
                samples[0]["state"],  # type: ignore[arg-type]
                timeout_s=SENSOR_PREPOSITION_TIMEOUT_S,
                settle_s=0.0,
        )

        self.assertFalse(aligned)
        self.assertLessEqual(len([command for command in vex_bridge.commands if command[0] != "hold"]), 6)
        self.assertEqual(vex_bridge.commands[-1][0], "hold")

    def test_generated_vex_program_supports_usb_origin_command(self) -> None:
        source = build_vex_telemetry_program_source({})
        compile(source, "<generated-vex-telemetry>", "exec")

        self.assertIn("#vex:disable=repl", source)
        self.assertIn("serial_reader_thread = Thread(serial_command_reader)", source)
        self.assertIn("chunk = sys.stdin.read(1)", source)
        self.assertIn("start_serial_command_reader()", source)
        self.assertNotIn("import uselect", source)
        self.assertNotIn("stdin_poll.register(sys.stdin, uselect.POLLIN)", source)
        self.assertNotIn('open("/dev/serial1", "rb")', source)
        self.assertIn('if command == "!origin"', source)
        self.assertIn("inertial_1.reset_rotation()", source)
        self.assertIn("inertial_1.reset_heading()", source)
        self.assertIn("pose_epoch += 1", source)
        self.assertIn('"vex_pose_epoch":%d', source)
        self.assertIn("VEX_MIXER_VERSION = 6", source)
        self.assertIn('"vex_mixer_version":%d', source)
        self.assertIn("remote_takeover = True", source)
        self.assertIn('remote_mode = "hold"', source)

    def test_command_stream_probe_requires_runtime_zero_velocity_command(self) -> None:
        bridge = VexBaseBridge(
            requested_port="auto",
            baudrate=115200,
            stale_after_s=0.35,
            command_timeout_s=0.35,
            logger=mock.Mock(),
        )
        bridge.serial_handle = object()
        bridge.wait_for_update = mock.Mock(return_value=True)  # type: ignore[method-assign]
        bridge.send_motion = mock.Mock(return_value=True)  # type: ignore[method-assign]
        bridge.send_hold = mock.Mock(return_value=True)  # type: ignore[method-assign]
        bridge.wait_for_source = mock.Mock(return_value=True)  # type: ignore[method-assign]

        self.assertTrue(bridge.remote_command_stream_active(timeout_s=1.25))

        bridge.send_motion.assert_called_once_with(
            {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0},
            ttl_ms=350,
        )
        bridge.wait_for_source.assert_called_once()
        self.assertEqual(bridge.wait_for_source.call_args.args[0], {"pi-drive"})
        self.assertEqual(bridge.wait_for_source.call_args.kwargs["timeout_s"], 1.25)
        bridge.send_hold.assert_called_once_with(ttl_ms=350)

    def test_generated_vex_program_uses_normal_vex_axis_defaults(self) -> None:
        source = build_vex_telemetry_program_source({})

        self.assertIn("forward_pct = apply_deadband(controller_1.axis3.position())", source)
        self.assertIn("strafe_pct = apply_deadband(controller_1.axis4.position())", source)
        self.assertIn("turn_pct = apply_deadband(controller_1.axis1.position())", source)
        self.assertIn('"x.vel": (strafe_pct / 100.0) * MAX_LINEAR_SPEED_MPS', source)
        self.assertIn('"y.vel": (forward_pct / 100.0) * MAX_LINEAR_SPEED_MPS', source)
        self.assertIn(
            'return (\n'
            '        clamp((motion["y.vel"] / MAX_LINEAR_SPEED_MPS) * 100.0, -100.0, 100.0),\n'
            '        clamp((motion["x.vel"] / MAX_LINEAR_SPEED_MPS) * 100.0, -100.0, 100.0),',
            source,
        )
        self.assertNotIn("forward_pct = apply_deadband(controller_1.axis2.position())", source)

    def test_generated_vex_program_uses_default_x_drive_mixer(self) -> None:
        source = build_vex_telemetry_program_source({})

        self.assertIn("front_left_pct = forward_pct + turn_pct + strafe_pct", source)
        self.assertIn("rear_left_pct = forward_pct + turn_pct - strafe_pct", source)
        self.assertIn("front_right_pct = forward_pct - turn_pct - strafe_pct", source)
        self.assertIn("rear_right_pct = forward_pct - turn_pct + strafe_pct", source)
        self.assertNotIn("rear_left_pct = forward_pct - turn_pct + strafe_pct", source)
        self.assertNotIn("rear_right_pct = forward_pct + turn_pct - strafe_pct", source)
        self.assertIn("def spin_drive_motor(motor, percent):", source)
        self.assertIn("motor.set_velocity(abs(percent), PERCENT)", source)
        self.assertIn("motor.spin(FORWARD if percent >= 0 else REVERSE)", source)
        self.assertIn("spin_drive_motor(front_right, front_right_pct)", source)
        self.assertNotIn("front_right.set_velocity(front_right_pct, PERCENT)", source)

    def test_wheel_correction_signs_match_default_x_drive_mixer(self) -> None:
        self.assertEqual(sensor_replay.WHEEL_CORRECTION_SIGNS["vex_front_left.pos"], (1.0, 1.0))
        self.assertEqual(sensor_replay.WHEEL_CORRECTION_SIGNS["vex_rear_left.pos"], (-1.0, 1.0))
        self.assertEqual(sensor_replay.WHEEL_CORRECTION_SIGNS["vex_front_right.pos"], (-1.0, -1.0))
        self.assertEqual(sensor_replay.WHEEL_CORRECTION_SIGNS["vex_rear_right.pos"], (1.0, -1.0))

    def test_drive_wheel_percentages_treat_x_as_strafe_and_y_as_forward(self) -> None:
        forward_pct = sensor_replay._drive_wheel_percentages(
            {"x.vel": 0.0, "y.vel": 0.035, "theta.vel": 0.0},
            max_linear_speed_mps=0.35,
            max_turn_speed_dps=90.0,
        )
        strafe_pct = sensor_replay._drive_wheel_percentages(
            {"x.vel": 0.035, "y.vel": 0.0, "theta.vel": 0.0},
            max_linear_speed_mps=0.35,
            max_turn_speed_dps=90.0,
        )

        self.assertEqual(
            forward_pct,
            {
                "front_right": 10.0,
                "front_left": 10.0,
                "rear_right": 10.0,
                "rear_left": 10.0,
            },
        )
        self.assertEqual(
            strafe_pct,
            {
                "front_right": -10.0,
                "front_left": 10.0,
                "rear_right": 10.0,
                "rear_left": -10.0,
            },
        )

    def test_turn_only_command_uses_left_right_rotation_pattern(self) -> None:
        wheel_pct = sensor_replay._drive_wheel_percentages(
            {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 9.0},
            max_linear_speed_mps=0.35,
            max_turn_speed_dps=90.0,
        )

        self.assertLess(wheel_pct["front_right"], 0.0)
        self.assertGreater(wheel_pct["front_left"], 0.0)
        self.assertLess(wheel_pct["rear_right"], 0.0)
        self.assertGreater(wheel_pct["rear_left"], 0.0)

    def test_recorded_state_has_sensor_reference(self) -> None:
        self.assertTrue(
            recorded_state_has_sensor_reference(build_sample()["state"])  # type: ignore[arg-type]
        )
        self.assertFalse(recorded_state_has_sensor_reference({"x.vel": 0.0, "y.vel": 0.0}))

    def test_build_replay_command_uses_recorded_targets_without_live_trim(self) -> None:
        sample = build_sample(x_vel=0.15, y_vel=0.05, theta_vel=8.0, wheel_base=250.0)
        replay_state = SensorAwareReplayState(
            [sample],
            replay_mode="ecu",
            speed=1.5,
            control_config={"tuning": {"maxLinearSpeedMps": 0.35, "maxTurnSpeedDps": 90.0}},
        )
        replay_state.ecu_offsets_deg = {key: 99.0 for key in VEX_MOTOR_STATE_KEYS}

        command = replay_state.build_replay_command(
            sample["state"],  # type: ignore[arg-type]
            command_dt_s=0.05,
        )

        self.assertEqual(command.mode, "ecu")
        self.assertIsNotNone(command.ecu_targets)
        for key in VEX_MOTOR_STATE_KEYS:
            self.assertEqual(command.ecu_targets[key], sample["state"][key])  # type: ignore[index]
        self.assertAlmostEqual(command.motion["x.vel"], 0.225)
        self.assertAlmostEqual(command.motion["y.vel"], 0.075)
        self.assertAlmostEqual(command.motion["theta.vel"], 12.0)

    def test_build_replay_command_drive_mode_replays_recorded_motion(self) -> None:
        sample = build_sample(x_vel=0.1, y_vel=-0.2, theta_vel=6.0)
        replay_state = SensorAwareReplayState(
            [sample],
            replay_mode="drive",
            speed=2.0,
            control_config={"tuning": {"maxLinearSpeedMps": 0.5, "maxTurnSpeedDps": 120.0}},
        )

        command = replay_state.build_replay_command(
            sample["state"],  # type: ignore[arg-type]
            command_dt_s=0.05,
        )

        self.assertEqual(command.mode, "drive")
        self.assertIsNone(command.ecu_targets)
        self.assertAlmostEqual(command.motion["x.vel"], 0.2)
        self.assertAlmostEqual(command.motion["y.vel"], -0.4)
        self.assertAlmostEqual(command.motion["theta.vel"], 12.0)


if __name__ == "__main__":
    unittest.main()
