from __future__ import annotations

import logging
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = ROOT_DIR / "scripts"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

from lekiwi_runtime import (
    ACTION_COMMAND_SOURCE_KEY,
    ARM_MOTORS,
    ArmSafetyFilter,
    COMMAND_SOURCE_KEYBOARD,
    ServoProtectionSupervisor,
    build_normalized_arm_position_limits,
    clamp_safe_arm_torque_limits,
)


ALL_MOTORS = list(ARM_MOTORS) + [
    "base_left_wheel",
    "base_back_wheel",
    "base_right_wheel",
]


class FakeBus:
    def __init__(self) -> None:
        self.disabled_calls: list[list[str]] = []
        self.goal_writes: list[dict[str, float]] = []
        self.motors = {motor: object() for motor in ALL_MOTORS}

    def disable_torque(self, motors, num_retry: int = 10) -> None:  # noqa: ANN001
        if isinstance(motors, str):
            payload = [motors]
        else:
            payload = list(motors)
        self.disabled_calls.append(payload)

    def sync_write(self, item: str, values: dict[str, float]) -> None:
        if item == "Goal_Position":
            self.goal_writes.append(dict(values))


class FakeRobot:
    def __init__(self) -> None:
        self.arm_motors = list(ARM_MOTORS)
        self.bus = FakeBus()


class FakeMotorConfig:
    def __init__(self, model: str, norm_mode: str) -> None:
        self.model = model
        self.norm_mode = SimpleNamespace(value=norm_mode)


class FakeLimitRobot:
    def __init__(self) -> None:
        self.arm_motors = [
            "arm_shoulder_pan",
            "arm_wrist_roll",
            "arm_gripper",
        ]
        self.bus = SimpleNamespace(
            calibration={
                "arm_shoulder_pan": SimpleNamespace(range_min=1024, range_max=3071),
                "arm_wrist_roll": SimpleNamespace(range_min=0, range_max=4095),
                "arm_gripper": SimpleNamespace(range_min=0, range_max=4095),
            },
            motors={
                "arm_shoulder_pan": FakeMotorConfig("sts3215", "degrees"),
                "arm_wrist_roll": FakeMotorConfig("sts3215", "degrees"),
                "arm_gripper": FakeMotorConfig("sts3215", "range_0_100"),
            },
            model_resolution_table={"sts3215": 4096},
        )


def build_observation(elbow: float = -95.0) -> dict[str, float]:
    return {
        "arm_shoulder_pan.pos": -85.0,
        "arm_shoulder_lift.pos": -95.0,
        "arm_elbow_flex.pos": elbow,
        "arm_wrist_flex.pos": 105.0,
        "arm_wrist_roll.pos": 25.0,
        "arm_gripper.pos": 40.0,
    }


class RuntimeSafetyTests(unittest.TestCase):
    def test_build_normalized_arm_position_limits_uses_robot_calibration(self) -> None:
        limits = build_normalized_arm_position_limits(
            FakeLimitRobot(),
            preserve_continuous_wrist_roll=True,
        )

        midpoint = (1024 + 3071) / 2.0
        expected_pan_limit = ((1024 - midpoint) * 360.0) / 4095.0
        self.assertAlmostEqual(limits["arm_shoulder_pan.pos"][0], expected_pan_limit, places=3)
        self.assertAlmostEqual(limits["arm_shoulder_pan.pos"][1], -expected_pan_limit, places=3)
        self.assertEqual(limits["arm_gripper.pos"], (0.0, 85.0))
        self.assertNotIn("arm_wrist_roll.pos", limits)

    def test_arm_safety_filter_keyboard_source_skips_step_limit_but_keeps_hard_stop(self) -> None:
        safety_filter = ArmSafetyFilter(
            enabled=True,
            absolute_position_limits={"arm_shoulder_pan.pos": (-20.0, 20.0)},
            skip_step_limit_sources={COMMAND_SOURCE_KEYBOARD},
        )
        observation = build_observation()
        observation["arm_shoulder_pan.pos"] = 0.0
        safety_filter.seed_from_observation(observation)

        limited = safety_filter.normalize({"arm_shoulder_pan.pos": 18.0})
        keyboard = safety_filter.normalize(
            {
                "arm_shoulder_pan.pos": 18.0,
                ACTION_COMMAND_SOURCE_KEY: COMMAND_SOURCE_KEYBOARD,
            }
        )
        hard_stop = safety_filter.normalize(
            {
                "arm_shoulder_pan.pos": 25.0,
                ACTION_COMMAND_SOURCE_KEY: COMMAND_SOURCE_KEYBOARD,
            }
        )

        self.assertEqual(limited["arm_shoulder_pan.pos"], 8.0)
        self.assertEqual(keyboard["arm_shoulder_pan.pos"], 18.0)
        self.assertEqual(hard_stop["arm_shoulder_pan.pos"], 20.0)

    def test_clamp_safe_arm_torque_limits_caps_unsafe_values(self) -> None:
        clamped, capped = clamp_safe_arm_torque_limits(
            {
                "arm_elbow_flex": 790,
                "arm_shoulder_lift": 870,
                "arm_gripper": 840,
            }
        )

        self.assertEqual(clamped["arm_elbow_flex"], 600)
        self.assertEqual(clamped["arm_shoulder_lift"], 650)
        self.assertEqual(clamped["arm_gripper"], 840)
        self.assertIn("arm_elbow_flex=790->600", capped)
        self.assertIn("arm_shoulder_lift=870->650", capped)

    def test_servo_protection_latches_on_persistent_stall(self) -> None:
        robot = FakeRobot()
        supervisor = ServoProtectionSupervisor(robot, logging.getLogger("runtime-safety-test"))
        observation = build_observation()
        supervisor.seed_from_observation(observation)

        supervisor.record_command({"arm_elbow_flex.pos": -70.0})
        with mock.patch("lekiwi_runtime.time.monotonic", return_value=0.0):
            self.assertFalse(supervisor.observe(observation))
        with mock.patch("lekiwi_runtime.time.monotonic", return_value=0.6):
            tripped = supervisor.observe(observation)

        self.assertTrue(tripped)
        self.assertTrue(supervisor.latched)
        self.assertIsNotNone(supervisor.fault)
        self.assertEqual(supervisor.fault["motor"], "arm_elbow_flex")
        self.assertEqual(robot.bus.disabled_calls[-1], ALL_MOTORS)

    def test_shoulder_lift_low_current_position_lag_does_not_latch(self) -> None:
        robot = FakeRobot()
        supervisor = ServoProtectionSupervisor(robot, logging.getLogger("runtime-safety-test"))
        observation = build_observation()
        observation["arm_shoulder_lift.pos"] = -47.56
        supervisor.seed_from_observation(observation)

        supervisor.record_command({"arm_shoulder_lift.pos": -41.23})
        with mock.patch("lekiwi_runtime.time.monotonic", return_value=0.0):
            self.assertFalse(
                supervisor.observe(
                    observation,
                    {
                        "monotonic_s": 0.0,
                        "arm_shoulder_lift.current_ma_est": 20.0,
                    },
                )
            )
        with mock.patch("lekiwi_runtime.time.monotonic", return_value=0.6):
            tripped = supervisor.observe(
                observation,
                {
                    "monotonic_s": 0.6,
                    "arm_shoulder_lift.current_ma_est": 20.0,
                },
            )

        self.assertFalse(tripped)
        self.assertFalse(supervisor.latched)
        self.assertEqual(robot.bus.disabled_calls, [])

    def test_wrist_flex_low_current_position_lag_does_not_latch(self) -> None:
        robot = FakeRobot()
        supervisor = ServoProtectionSupervisor(robot, logging.getLogger("runtime-safety-test"))
        observation = build_observation()
        observation["arm_wrist_flex.pos"] = -95.34
        supervisor.seed_from_observation(observation)

        supervisor.record_command({"arm_wrist_flex.pos": -89.89})
        with mock.patch("lekiwi_runtime.time.monotonic", return_value=0.0):
            self.assertFalse(
                supervisor.observe(
                    observation,
                    {
                        "monotonic_s": 0.0,
                        "arm_wrist_flex.current_ma_est": 290.0,
                    },
                )
            )
        with mock.patch("lekiwi_runtime.time.monotonic", return_value=0.5):
            tripped = supervisor.observe(
                observation,
                {
                    "monotonic_s": 0.5,
                    "arm_wrist_flex.current_ma_est": 290.0,
                },
            )

        self.assertFalse(tripped)
        self.assertFalse(supervisor.latched)
        self.assertEqual(robot.bus.disabled_calls, [])

    def test_wrist_flex_high_current_stall_still_latches(self) -> None:
        robot = FakeRobot()
        supervisor = ServoProtectionSupervisor(robot, logging.getLogger("runtime-safety-test"))
        observation = build_observation()
        observation["arm_wrist_flex.pos"] = -95.34
        supervisor.seed_from_observation(observation)

        supervisor.record_command({"arm_wrist_flex.pos": -89.89})
        with mock.patch("lekiwi_runtime.time.monotonic", return_value=0.0):
            self.assertFalse(
                supervisor.observe(
                    observation,
                    {
                        "monotonic_s": 0.0,
                        "arm_wrist_flex.current_ma_est": 700.0,
                    },
                )
            )
        with mock.patch("lekiwi_runtime.time.monotonic", return_value=0.5):
            tripped = supervisor.observe(
                observation,
                {
                    "monotonic_s": 0.5,
                    "arm_wrist_flex.current_ma_est": 700.0,
                },
            )

        self.assertTrue(tripped)
        self.assertTrue(supervisor.latched)
        self.assertEqual(supervisor.fault["motor"], "arm_wrist_flex")
        self.assertIn("current 700mA >= 450mA", supervisor.fault["reason"])

    def test_gripper_closing_stall_enters_force_limited_hold(self) -> None:
        robot = FakeRobot()
        supervisor = ServoProtectionSupervisor(robot, logging.getLogger("runtime-safety-test"))
        observation = build_observation()
        supervisor.seed_from_observation(observation)

        supervisor.record_command({"arm_gripper.pos": 2.0})
        with mock.patch("lekiwi_runtime.time.monotonic", return_value=0.0):
            self.assertFalse(
                supervisor.observe(
                    observation,
                    {
                        "monotonic_s": 0.0,
                        "arm_gripper.current_ma_est": 950.0,
                    },
                )
            )
        with mock.patch("lekiwi_runtime.time.monotonic", return_value=0.25):
            self.assertFalse(
                supervisor.observe(
                    observation,
                    {
                        "monotonic_s": 0.25,
                        "arm_gripper.current_ma_est": 950.0,
                    },
                )
            )

        self.assertFalse(supervisor.latched)
        self.assertEqual(robot.bus.disabled_calls, [])
        self.assertTrue(supervisor.gripper_force_limit.active)
        self.assertAlmostEqual(supervisor.gripper_force_limit.hold_position, 40.0)
        self.assertAlmostEqual(supervisor.gripper_force_limit.hold_target, 39.2)
        self.assertEqual(robot.bus.goal_writes[-1], {"arm_gripper": 39.2})

        limited = supervisor.limit_action({"arm_gripper.pos": 2.0})
        self.assertEqual(limited["arm_gripper.pos"], 39.2)

        released = supervisor.limit_action({"arm_gripper.pos": 45.0})
        self.assertEqual(released["arm_gripper.pos"], 45.0)
        self.assertFalse(supervisor.gripper_force_limit.active)

    def test_gripper_closing_low_current_stall_does_not_force_hold_or_latch(self) -> None:
        robot = FakeRobot()
        supervisor = ServoProtectionSupervisor(robot, logging.getLogger("runtime-safety-test"))
        observation = build_observation()
        observation["arm_gripper.pos"] = 13.36
        supervisor.seed_from_observation(observation)

        supervisor.record_command({"arm_gripper.pos": 2.38})
        with mock.patch("lekiwi_runtime.time.monotonic", return_value=0.0):
            self.assertFalse(
                supervisor.observe(
                    observation,
                    {
                        "monotonic_s": 0.0,
                        "arm_gripper.current_ma_est": 650.0,
                    },
                )
            )
        with mock.patch("lekiwi_runtime.time.monotonic", return_value=0.25):
            tripped = supervisor.observe(
                observation,
                {
                    "monotonic_s": 0.25,
                    "arm_gripper.current_ma_est": 650.0,
                },
            )

        self.assertFalse(tripped)
        self.assertFalse(supervisor.latched)
        self.assertFalse(supervisor.gripper_force_limit.active)
        self.assertEqual(robot.bus.disabled_calls, [])
        self.assertEqual(robot.bus.goal_writes, [])

    def test_gripper_logged_current_reaches_default_force_threshold(self) -> None:
        robot = FakeRobot()
        supervisor = ServoProtectionSupervisor(robot, logging.getLogger("runtime-safety-test"))
        observation = build_observation()
        observation["arm_gripper.pos"] = 13.36
        supervisor.seed_from_observation(observation)
        supervisor.record_command({"arm_gripper.pos": 2.38})

        with mock.patch("lekiwi_runtime.time.monotonic", return_value=0.0):
            tripped = supervisor.observe(
                observation,
                {
                    "monotonic_s": 0.0,
                    "arm_gripper.current_ma_est": 734.0,
                },
            )

        self.assertFalse(tripped)
        self.assertFalse(supervisor.latched)
        self.assertTrue(supervisor.gripper_force_limit.active)
        self.assertIn("soft limit 700mA", supervisor.gripper_force_limit.reason)

    def test_gripper_current_threshold_enters_force_limited_hold(self) -> None:
        robot = FakeRobot()
        supervisor = ServoProtectionSupervisor(
            robot,
            logging.getLogger("runtime-safety-test"),
            gripper_current_limit_ma=500.0,
        )
        observation = build_observation()
        supervisor.seed_from_observation(observation)
        supervisor.record_command({"arm_gripper.pos": 5.0})

        with mock.patch("lekiwi_runtime.time.monotonic", return_value=0.0):
            tripped = supervisor.observe(
                observation,
                {
                    "monotonic_s": 0.0,
                    "arm_gripper.current_ma_est": 650.0,
                },
            )

        self.assertFalse(tripped)
        self.assertFalse(supervisor.latched)
        self.assertEqual(robot.bus.disabled_calls, [])
        self.assertTrue(supervisor.gripper_force_limit.active)
        self.assertIn("current reached 650mA", supervisor.gripper_force_limit.reason)

    def test_gripper_opening_high_current_stall_still_latches(self) -> None:
        robot = FakeRobot()
        supervisor = ServoProtectionSupervisor(robot, logging.getLogger("runtime-safety-test"))
        observation = build_observation()
        supervisor.seed_from_observation(observation)

        supervisor.record_command({"arm_gripper.pos": 75.0})
        with mock.patch("lekiwi_runtime.time.monotonic", return_value=0.0):
            self.assertFalse(
                supervisor.observe(
                    observation,
                    {
                        "monotonic_s": 0.0,
                        "arm_gripper.current_ma_est": 950.0,
                    },
                )
            )
        with mock.patch("lekiwi_runtime.time.monotonic", return_value=0.25):
            tripped = supervisor.observe(
                observation,
                {
                    "monotonic_s": 0.25,
                    "arm_gripper.current_ma_est": 950.0,
                },
            )

        self.assertTrue(tripped)
        self.assertTrue(supervisor.latched)
        self.assertEqual(supervisor.fault["motor"], "arm_gripper")
        self.assertEqual(robot.bus.disabled_calls[-1], ALL_MOTORS)

    def test_servo_protection_can_disable_stall_detection_for_free_teach(self) -> None:
        robot = FakeRobot()
        supervisor = ServoProtectionSupervisor(
            robot,
            logging.getLogger("runtime-safety-test"),
            stall_detection_enabled=False,
        )
        observation = build_observation()
        observation["arm_shoulder_lift.pos"] = -104.35
        supervisor.seed_from_observation(observation)
        supervisor.record_command({"arm_shoulder_lift.pos": -104.35})

        hand_moved = dict(observation)
        hand_moved["arm_shoulder_lift.pos"] = -83.69
        with mock.patch("lekiwi_runtime.time.monotonic", return_value=0.0):
            self.assertFalse(supervisor.observe(hand_moved))
        with mock.patch("lekiwi_runtime.time.monotonic", return_value=0.6):
            self.assertFalse(supervisor.observe(hand_moved))

        self.assertFalse(supervisor.latched)
        self.assertEqual(robot.bus.disabled_calls, [])

    def test_servo_protection_stall_disabled_keeps_overtemp_latch(self) -> None:
        robot = FakeRobot()
        supervisor = ServoProtectionSupervisor(
            robot,
            logging.getLogger("runtime-safety-test"),
            stall_detection_enabled=False,
        )
        observation = build_observation()
        supervisor.seed_from_observation(observation)

        with mock.patch("lekiwi_runtime.time.monotonic", return_value=0.0):
            self.assertFalse(
                supervisor.observe(
                    observation,
                    {
                        "monotonic_s": 0.0,
                        "base_left_wheel.temperature_c": 50.0,
                    },
                )
            )
        with mock.patch("lekiwi_runtime.time.monotonic", return_value=0.5):
            self.assertTrue(
                supervisor.observe(
                    observation,
                    {
                        "monotonic_s": 0.5,
                        "base_left_wheel.temperature_c": 50.0,
                    },
                )
            )

        self.assertTrue(supervisor.latched)
        self.assertEqual(supervisor.fault["motor"], "base_left_wheel")
        self.assertEqual(robot.bus.disabled_calls[-1], ALL_MOTORS)

    def test_servo_protection_latches_on_any_motor_overtemp(self) -> None:
        robot = FakeRobot()
        supervisor = ServoProtectionSupervisor(robot, logging.getLogger("runtime-safety-test"))
        observation = build_observation()
        supervisor.seed_from_observation(observation)

        with mock.patch("lekiwi_runtime.time.monotonic", return_value=0.0):
            self.assertFalse(
                supervisor.observe(
                    observation,
                    {
                        "monotonic_s": 0.0,
                        "base_left_wheel.temperature_c": 50.0,
                    },
                )
            )
        with mock.patch("lekiwi_runtime.time.monotonic", return_value=0.5):
            tripped = supervisor.observe(
                observation,
                {
                    "monotonic_s": 0.5,
                    "base_left_wheel.temperature_c": 50.0,
                },
            )

        self.assertTrue(tripped)
        self.assertTrue(supervisor.latched)
        self.assertEqual(supervisor.fault["motor"], "base_left_wheel")
        self.assertEqual(robot.bus.disabled_calls[-1], ALL_MOTORS)

    def test_servo_protection_ignores_single_overtemp_spike(self) -> None:
        robot = FakeRobot()
        supervisor = ServoProtectionSupervisor(robot, logging.getLogger("runtime-safety-test"))
        observation = build_observation()
        supervisor.seed_from_observation(observation)

        with mock.patch("lekiwi_runtime.time.monotonic", return_value=0.0):
            self.assertFalse(
                supervisor.observe(
                    observation,
                    {
                        "monotonic_s": 0.0,
                        "arm_shoulder_lift.temperature_c": 100.0,
                    },
                )
            )
        with mock.patch("lekiwi_runtime.time.monotonic", return_value=0.5):
            self.assertFalse(
                supervisor.observe(
                    observation,
                    {
                        "monotonic_s": 0.5,
                        "arm_shoulder_lift.temperature_c": 28.0,
                    },
                )
            )

        self.assertFalse(supervisor.latched)
        self.assertEqual(robot.bus.disabled_calls, [])


if __name__ == "__main__":
    unittest.main()
