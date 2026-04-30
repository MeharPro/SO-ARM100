from __future__ import annotations

import sys
import types
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = ROOT_DIR / "scripts"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))


def install_lerobot_stubs() -> None:
    if "lerobot.robots.lekiwi" in sys.modules:
        return

    lerobot = types.ModuleType("lerobot")
    robots = types.ModuleType("lerobot.robots")
    lekiwi = types.ModuleType("lerobot.robots.lekiwi")
    utils = types.ModuleType("lerobot.utils")
    robot_utils = types.ModuleType("lerobot.utils.robot_utils")

    class DummyLeKiwiClient:  # pragma: no cover - import stub only
        pass

    class DummyLeKiwiClientConfig:  # pragma: no cover - import stub only
        def __init__(self, *args, **kwargs) -> None:
            self.args = args
            self.kwargs = kwargs

    def precise_sleep(_duration: float) -> None:
        return None

    lekiwi.LeKiwiClient = DummyLeKiwiClient
    lekiwi.LeKiwiClientConfig = DummyLeKiwiClientConfig
    robot_utils.precise_sleep = precise_sleep

    sys.modules["lerobot"] = lerobot
    sys.modules["lerobot.robots"] = robots
    sys.modules["lerobot.robots.lekiwi"] = lekiwi
    sys.modules["lerobot.utils"] = utils
    sys.modules["lerobot.utils.robot_utils"] = robot_utils


install_lerobot_stubs()

import lekiwi_keyboard_teleop as keyboard_teleop


def build_targets(**overrides: float) -> dict[str, float]:
    targets = {
        joint: 0.0
        for joint, *_rest in keyboard_teleop.ARM_BINDINGS
    }
    targets.update(overrides)
    return targets


class KeyboardTeleopTests(unittest.TestCase):
    def test_base_keyboard_mapping_matches_documented_vex_controls(self) -> None:
        forward_left_turn_left = keyboard_teleop.build_base_action(
            {"i", "j", "u"},
            linear_speed=0.04,
            turn_speed=12.0,
        )

        self.assertEqual(forward_left_turn_left["x.vel"], -0.04)
        self.assertEqual(forward_left_turn_left["y.vel"], 0.04)
        self.assertEqual(forward_left_turn_left["theta.vel"], -12.0)

        back_right_turn_right = keyboard_teleop.build_base_action(
            {"k", "l", "o"},
            linear_speed=0.04,
            turn_speed=12.0,
        )

        self.assertEqual(back_right_turn_right["x.vel"], 0.04)
        self.assertEqual(back_right_turn_right["y.vel"], -0.04)
        self.assertEqual(back_right_turn_right["theta.vel"], 12.0)

    def test_base_keyboard_single_keys_are_axis_isolated_and_speed_limited(self) -> None:
        requested_linear_speed = 0.35
        requested_turn_speed = 90.0
        linear_limit = keyboard_teleop.KEYBOARD_BASE_LINEAR_SPEED_LIMIT_MPS
        turn_limit = keyboard_teleop.KEYBOARD_BASE_TURN_SPEED_LIMIT_DPS

        forward = keyboard_teleop.build_base_action(
            {"i"},
            linear_speed=requested_linear_speed,
            turn_speed=requested_turn_speed,
        )
        self.assertEqual(forward, {"x.vel": 0.0, "y.vel": linear_limit, "theta.vel": 0.0})

        back = keyboard_teleop.build_base_action(
            {"k"},
            linear_speed=requested_linear_speed,
            turn_speed=requested_turn_speed,
        )
        self.assertEqual(back, {"x.vel": 0.0, "y.vel": -linear_limit, "theta.vel": 0.0})

        turn_left = keyboard_teleop.build_base_action(
            {"u"},
            linear_speed=requested_linear_speed,
            turn_speed=requested_turn_speed,
        )
        self.assertEqual(turn_left, {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": -turn_limit})

        turn_right = keyboard_teleop.build_base_action(
            {"o"},
            linear_speed=requested_linear_speed,
            turn_speed=requested_turn_speed,
        )
        self.assertEqual(turn_right, {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": turn_limit})

    def test_base_keyboard_ramp_accelerates_held_key_slowly(self) -> None:
        ramp = keyboard_teleop.KeyboardBaseRamp()
        linear_limit = keyboard_teleop.KEYBOARD_BASE_LINEAR_SPEED_LIMIT_MPS
        initial_ratio = keyboard_teleop.KEYBOARD_BASE_INITIAL_SPEED_RATIO

        first = ramp.update(
            {"i"},
            linear_speed=0.35,
            turn_speed=90.0,
            now_s=10.0,
        )
        halfway = ramp.update(
            {"i"},
            linear_speed=0.35,
            turn_speed=90.0,
            now_s=10.0 + keyboard_teleop.KEYBOARD_BASE_ACCELERATION_S / 2.0,
        )
        fully_ramped = ramp.update(
            {"i"},
            linear_speed=0.35,
            turn_speed=90.0,
            now_s=10.0 + keyboard_teleop.KEYBOARD_BASE_ACCELERATION_S,
        )

        self.assertAlmostEqual(first["y.vel"], linear_limit * initial_ratio)
        self.assertGreater(halfway["y.vel"], first["y.vel"])
        self.assertLess(halfway["y.vel"], fully_ramped["y.vel"])
        self.assertAlmostEqual(fully_ramped["y.vel"], linear_limit)
        self.assertEqual(first["x.vel"], 0.0)
        self.assertEqual(first["theta.vel"], 0.0)

        ramp.update(set(), linear_speed=0.35, turn_speed=90.0, now_s=13.0)
        reset = ramp.update(
            {"i"},
            linear_speed=0.35,
            turn_speed=90.0,
            now_s=14.0,
        )
        self.assertAlmostEqual(reset["y.vel"], linear_limit * initial_ratio)

    def test_vex_mode_toggle_switches_keyboard_base_speed(self) -> None:
        mode = keyboard_teleop.VexControlModeState("drive")
        self.assertEqual(
            mode.speed_pair(
                drive_linear_speed=0.35,
                drive_turn_speed=90.0,
                ecu_linear_speed=0.06,
                ecu_turn_speed=18.0,
            ),
            (0.35, 90.0),
        )

        self.assertTrue(mode.update({"0"}))
        self.assertEqual(
            mode.speed_pair(
                drive_linear_speed=0.35,
                drive_turn_speed=90.0,
                ecu_linear_speed=0.06,
                ecu_turn_speed=18.0,
            ),
            (0.06, 18.0),
        )

        self.assertTrue(mode.update({"0"}))
        self.assertEqual(
            mode.speed_pair(
                drive_linear_speed=0.35,
                drive_turn_speed=90.0,
                ecu_linear_speed=0.06,
                ecu_turn_speed=18.0,
            ),
            (0.35, 90.0),
        )

        drive_action = keyboard_teleop.build_base_action(
            {"i", "o"},
            linear_speed=0.35,
            turn_speed=90.0,
            linear_speed_limit=0.35,
            turn_speed_limit=90.0,
        )
        self.assertEqual(drive_action, {"x.vel": 0.0, "y.vel": 0.35, "theta.vel": 90.0})

    def test_direction_lock_blocks_outward_motion_but_allows_reverse(self) -> None:
        joint = "arm_shoulder_lift.pos"
        limiter = keyboard_teleop.JointDirectionLimiter({})
        observed = build_targets(**{joint: -63.0})
        limiter.seed(observed)

        stalled_targets = build_targets(**{joint: -58.0})
        limiter.update(observed, stalled_targets, {"w"}, 0.0)
        events = limiter.update(observed, stalled_targets, {"w"}, 0.35)

        self.assertTrue(limiter.is_locked(joint, "positive"))
        self.assertIn("positive input blocked", events[-1])

        held_forward = keyboard_teleop.update_arm_targets(
            stalled_targets,
            observed,
            {"w"},
            {"w"},
            0.1,
            {},
            limiter,
        )
        self.assertEqual(held_forward[joint], -63.0)

        reverse = keyboard_teleop.update_arm_targets(
            stalled_targets,
            observed,
            {"s"},
            {"s"},
            0.1,
            {},
            limiter,
        )
        self.assertLess(reverse[joint], -63.0)

        moved_away = build_targets(**{joint: -65.0})
        limiter.update(moved_away, reverse, {"s"}, 0.4)
        self.assertFalse(limiter.is_locked(joint, "positive"))


if __name__ == "__main__":
    unittest.main()
