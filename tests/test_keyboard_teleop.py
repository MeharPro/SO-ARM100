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
            {"arrow_up", "arrow_left", "o"},
            linear_speed=0.04,
            turn_speed=12.0,
        )

        self.assertEqual(forward_left_turn_left["x.vel"], 0.04)
        self.assertEqual(forward_left_turn_left["y.vel"], 0.04)
        self.assertEqual(forward_left_turn_left["theta.vel"], -12.0)

        back_right_turn_right = keyboard_teleop.build_base_action(
            {"arrow_down", "arrow_right", "p"},
            linear_speed=0.04,
            turn_speed=12.0,
        )

        self.assertEqual(back_right_turn_right["x.vel"], -0.04)
        self.assertEqual(back_right_turn_right["y.vel"], -0.04)
        self.assertEqual(back_right_turn_right["theta.vel"], 12.0)

    def test_legacy_letter_base_keys_remain_fallback_not_primary(self) -> None:
        legacy = keyboard_teleop.build_base_action(
            {"i", "l", "u"},
            linear_speed=0.04,
            turn_speed=12.0,
        )

        self.assertEqual(legacy, {"x.vel": 0.04, "y.vel": 0.04, "theta.vel": -12.0})

    def test_base_keyboard_single_keys_are_axis_isolated_and_speed_limited(self) -> None:
        requested_linear_speed = 0.35
        requested_turn_speed = 90.0
        linear_limit = keyboard_teleop.KEYBOARD_BASE_LINEAR_SPEED_LIMIT_MPS
        turn_limit = keyboard_teleop.KEYBOARD_BASE_TURN_SPEED_LIMIT_DPS

        forward = keyboard_teleop.build_base_action(
            {"arrow_up"},
            linear_speed=requested_linear_speed,
            turn_speed=requested_turn_speed,
        )
        self.assertEqual(forward, {"x.vel": 0.0, "y.vel": linear_limit, "theta.vel": 0.0})

        back = keyboard_teleop.build_base_action(
            {"arrow_down"},
            linear_speed=requested_linear_speed,
            turn_speed=requested_turn_speed,
        )
        self.assertEqual(back, {"x.vel": 0.0, "y.vel": -linear_limit, "theta.vel": 0.0})

        turn_left = keyboard_teleop.build_base_action(
            {"o"},
            linear_speed=requested_linear_speed,
            turn_speed=requested_turn_speed,
        )
        self.assertEqual(turn_left, {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": -turn_limit})

        turn_right = keyboard_teleop.build_base_action(
            {"p"},
            linear_speed=requested_linear_speed,
            turn_speed=requested_turn_speed,
        )
        self.assertEqual(turn_right, {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": turn_limit})

    def test_enter_hold_rotates_clockwise_after_short_hold_delay(self) -> None:
        enter = keyboard_teleop.EnterRotationClickState()

        initial = enter.update({"enter"}, 10.0)
        held = enter.update(
            {"enter"},
            10.0 + keyboard_teleop.ENTER_SINGLE_HOLD_DELAY_S + 0.01,
        )
        action = keyboard_teleop.build_base_action(
            held,
            linear_speed=0.04,
            turn_speed=12.0,
        )

        self.assertNotIn(keyboard_teleop.ENTER_CLOCKWISE_KEY, initial)
        self.assertEqual(action, {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 12.0})
        released = enter.update(set(), 10.5)
        self.assertEqual(released, set())

    def test_enter_double_click_and_hold_rotates_counter_clockwise(self) -> None:
        enter = keyboard_teleop.EnterRotationClickState()

        self.assertEqual(enter.update({"enter"}, 10.0), set())
        self.assertEqual(enter.update(set(), 10.05), set())
        held = enter.update({"enter"}, 10.20)
        action = keyboard_teleop.build_base_action(
            held,
            linear_speed=0.04,
            turn_speed=12.0,
        )

        self.assertIn(keyboard_teleop.ENTER_COUNTER_CLOCKWISE_KEY, held)
        self.assertEqual(action, {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": -12.0})

    def test_enter_double_click_window_expires_before_clockwise_hold(self) -> None:
        enter = keyboard_teleop.EnterRotationClickState()

        self.assertEqual(enter.update({"enter"}, 10.0), set())
        self.assertEqual(enter.update(set(), 10.05), set())
        second_press_at = 10.05 + keyboard_teleop.ENTER_DOUBLE_CLICK_MAX_GAP_S + 0.01
        self.assertEqual(enter.update({"enter"}, second_press_at), set())
        held = enter.update(
            {"enter"},
            second_press_at + keyboard_teleop.ENTER_SINGLE_HOLD_DELAY_S + 0.01,
        )
        action = keyboard_teleop.build_base_action(
            held,
            linear_speed=0.04,
            turn_speed=12.0,
        )

        self.assertEqual(action, {"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 12.0})

    def test_base_keyboard_ramp_accelerates_held_key_slowly(self) -> None:
        ramp = keyboard_teleop.KeyboardBaseRamp()
        linear_limit = keyboard_teleop.KEYBOARD_BASE_LINEAR_SPEED_LIMIT_MPS
        initial_ratio = keyboard_teleop.KEYBOARD_BASE_INITIAL_SPEED_RATIO

        first = ramp.update(
            {"arrow_up"},
            linear_speed=0.35,
            turn_speed=90.0,
            now_s=10.0,
        )
        halfway = ramp.update(
            {"arrow_up"},
            linear_speed=0.35,
            turn_speed=90.0,
            now_s=10.0 + keyboard_teleop.KEYBOARD_BASE_ACCELERATION_S / 2.0,
        )
        fully_ramped = ramp.update(
            {"arrow_up"},
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
            {"arrow_up"},
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
            {"arrow_up", "p"},
            linear_speed=0.35,
            turn_speed=90.0,
            linear_speed_limit=0.35,
            turn_speed_limit=90.0,
        )
        self.assertEqual(drive_action, {"x.vel": 0.0, "y.vel": 0.35, "theta.vel": 90.0})

    def test_direction_calibration_signs_transform_keyboard_base_only(self) -> None:
        signs = keyboard_teleop.normalize_keyboard_direction_calibration(
            '{"xSign": -1, "ySign": -1, "thetaSign": -1}'
        )

        calibrated = keyboard_teleop.build_base_action(
            {"arrow_left", "arrow_up", "p"},
            linear_speed=0.04,
            turn_speed=12.0,
            direction_signs=signs,
        )

        self.assertEqual(calibrated, {"x.vel": -0.04, "y.vel": -0.04, "theta.vel": -12.0})

    def test_default_direction_calibration_is_neutral(self) -> None:
        signs = keyboard_teleop.normalize_keyboard_direction_calibration("{}")
        neutral = keyboard_teleop.build_base_action(
            {"arrow_left", "arrow_up", "p"},
            linear_speed=0.04,
            turn_speed=12.0,
            direction_signs=signs,
        )

        self.assertEqual(neutral, {"x.vel": 0.04, "y.vel": 0.04, "theta.vel": 12.0})

    def test_leader_mode_ignores_keyboard_arm_input(self) -> None:
        joint = "arm_shoulder_pan.pos"
        limiter = keyboard_teleop.JointDirectionLimiter({})
        targets = build_targets(**{joint: 0.0})
        observed = build_targets(**{joint: 0.0})
        limiter.seed(observed)

        next_targets = keyboard_teleop.apply_arm_authority(
            targets,
            observed,
            {"q"},
            {"q"},
            0.5,
            {},
            limiter,
            keyboard_arm_input_enabled=False,
            leader_arm_action={joint: 22.0},
        )

        self.assertEqual(next_targets[joint], 22.0)

    def test_keyboard_mode_ignores_leader_when_leader_authority_absent(self) -> None:
        joint = "arm_shoulder_pan.pos"
        limiter = keyboard_teleop.JointDirectionLimiter({})
        targets = build_targets(**{joint: 0.0})
        observed = build_targets(**{joint: 0.0})
        limiter.seed(observed)

        next_targets = keyboard_teleop.apply_arm_authority(
            targets,
            observed,
            {"q"},
            {"q"},
            0.5,
            {},
            limiter,
            keyboard_arm_input_enabled=True,
            leader_arm_action=None,
        )

        self.assertGreater(next_targets[joint], targets[joint])

    def test_hold_or_base_only_mode_blocks_keyboard_arm_input(self) -> None:
        joint = "arm_shoulder_pan.pos"
        limiter = keyboard_teleop.JointDirectionLimiter({})
        targets = build_targets(**{joint: 7.0})
        observed = build_targets(**{joint: 7.0})
        limiter.seed(observed)

        next_targets = keyboard_teleop.apply_arm_authority(
            targets,
            observed,
            {"q"},
            {"q"},
            0.5,
            {},
            limiter,
            keyboard_arm_input_enabled=False,
            leader_arm_action=None,
        )

        self.assertEqual(next_targets[joint], 7.0)

    def test_base_only_action_does_not_command_arm_targets(self) -> None:
        action = keyboard_teleop.build_teleop_action(
            build_targets(**{"arm_shoulder_pan.pos": 42.0}),
            {"x.vel": 0.0, "y.vel": 0.05, "theta.vel": 0.0},
            mode="drive",
            include_arm_targets=False,
        )

        self.assertNotIn("arm_shoulder_pan.pos", action)
        self.assertEqual(action["y.vel"], 0.05)
        self.assertEqual(action[keyboard_teleop.VEX_CONTROL_MODE_KEY], "drive")

    def test_disabled_base_input_filters_vex_base_keys(self) -> None:
        filtered = keyboard_teleop.filter_base_control_keys(
            {"arrow_up", "p", "0", "enter", "q", "n"}
        )

        self.assertEqual(filtered, {"q", "n"})

    def test_controller_base_action_does_not_command_vex_base_or_mode(self) -> None:
        action = keyboard_teleop.build_teleop_action(
            build_targets(**{"arm_shoulder_pan.pos": 42.0}),
            {"x.vel": 0.0, "y.vel": 0.05, "theta.vel": 0.0},
            mode="drive",
            include_arm_targets=True,
            include_base_action=False,
        )

        self.assertEqual(action["arm_shoulder_pan.pos"], 42.0)
        self.assertNotIn("x.vel", action)
        self.assertNotIn(keyboard_teleop.VEX_CONTROL_MODE_KEY, action)

    def test_send_latest_action_uses_nonblocking_socket(self) -> None:
        class FakeAgain(Exception):
            pass

        class FakeZmq:
            NOBLOCK = 7
            Again = FakeAgain

        class FakeSocket:
            def __init__(self) -> None:
                self.sent: list[tuple[str, int]] = []

            def send_string(self, payload: str, flags: int = 0) -> None:
                self.sent.append((payload, flags))

        class FakeRobot:
            _zmq = FakeZmq

            def __init__(self) -> None:
                self.zmq_cmd_socket = FakeSocket()

        robot = FakeRobot()

        self.assertTrue(keyboard_teleop.send_latest_action(robot, {"x.vel": 1.0}))
        self.assertEqual(robot.zmq_cmd_socket.sent[0][1], FakeZmq.NOBLOCK)
        self.assertIn('"x.vel"', robot.zmq_cmd_socket.sent[0][0])

    def test_send_latest_action_drops_when_socket_is_full(self) -> None:
        class FakeAgain(Exception):
            pass

        class FakeZmq:
            NOBLOCK = 7
            Again = FakeAgain

        class FakeSocket:
            def send_string(self, _payload: str, flags: int = 0) -> None:
                raise FakeAgain()

        class FakeRobot:
            _zmq = FakeZmq
            zmq_cmd_socket = FakeSocket()

        self.assertFalse(keyboard_teleop.send_latest_action(FakeRobot(), {"x.vel": 1.0}))

    def test_vex_pin5_servo_keys_map_to_discrete_positions(self) -> None:
        self.assertEqual(keyboard_teleop.vex_pin5_servo_position_from_keys({"n"}), "start")
        self.assertEqual(keyboard_teleop.vex_pin5_servo_position_from_keys({"b"}), "up")
        self.assertEqual(keyboard_teleop.vex_pin5_servo_position_from_keys({"v"}), "down")
        self.assertIsNone(keyboard_teleop.vex_pin5_servo_position_from_keys({"z"}))

    def test_vex_pin5_servo_keys_are_not_gripper_aliases(self) -> None:
        gripper_binding = next(
            binding
            for binding in keyboard_teleop.ARM_BINDINGS
            if binding[0] == "arm_gripper.pos"
        )

        self.assertEqual(gripper_binding[1], ("z", "c"))
        self.assertEqual(gripper_binding[2], ("x",))

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
