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
            linear_speed=0.25,
            turn_speed=30.0,
        )

        self.assertEqual(forward_left_turn_left["x.vel"], -0.25)
        self.assertEqual(forward_left_turn_left["y.vel"], 0.25)
        self.assertEqual(forward_left_turn_left["theta.vel"], -30.0)

        back_right_turn_right = keyboard_teleop.build_base_action(
            {"k", "l", "o"},
            linear_speed=0.25,
            turn_speed=30.0,
        )

        self.assertEqual(back_right_turn_right["x.vel"], 0.25)
        self.assertEqual(back_right_turn_right["y.vel"], -0.25)
        self.assertEqual(back_right_turn_right["theta.vel"], 30.0)

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
