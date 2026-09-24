#!/usr/bin/env python3

from pathlib import Path
from types import SimpleNamespace
import sys
import unittest


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from fr3_keyboard_joint_jog import (  # noqa: E402
    JOINTS,
    STEP_SIZES,
    build_jog_target,
    interpret_key,
    maximum_absolute_velocity,
    measured_gripper_width,
    requires_per_jog_confirmation,
    validate_gripper_configuration,
    validate_session_budget,
    validate_trajectory_start,
)
from sensor_msgs.msg import JointState


class KeyboardJointJogTests(unittest.TestCase):
    def setUp(self) -> None:
        self.current = {joint: 0.0 for joint in JOINTS}

    def test_joint_selection_and_direction_keys(self) -> None:
        self.assertEqual(interpret_key("3").kind, "select_joint")
        self.assertEqual(interpret_key("3").value, 2)
        self.assertEqual(interpret_key("LEFT").value, -1.0)
        self.assertEqual(interpret_key("RIGHT").value, 1.0)

    def test_increment_modes_and_stop_keys(self) -> None:
        self.assertEqual(interpret_key("f").value, "fine")
        self.assertEqual(interpret_key("N").value, "normal")
        self.assertEqual(interpret_key("c").value, "coarse")
        self.assertEqual(interpret_key("SPACE").kind, "stop")
        self.assertEqual(interpret_key("q").kind, "stop")

    def test_configured_increment_values(self) -> None:
        self.assertEqual(
            STEP_SIZES,
            {"fine": 0.010, "normal": 0.030, "coarse": 0.060},
        )

    def test_gripper_keys(self) -> None:
        self.assertEqual(interpret_key("o").kind, "gripper_open")
        self.assertEqual(interpret_key("G").kind, "gripper_grasp")

    def test_auto_execute_skips_only_the_per_jog_confirmation(self) -> None:
        self.assertTrue(
            requires_per_jog_confirmation(execute=True, auto_execute=False)
        )
        self.assertFalse(
            requires_per_jog_confirmation(execute=True, auto_execute=True)
        )
        self.assertFalse(
            requires_per_jog_confirmation(execute=False, auto_execute=False)
        )

    def test_builds_only_selected_joint_delta(self) -> None:
        target = build_jog_target(self.current, "fr3_joint4", -1.0, 0.01)
        self.assertAlmostEqual(target["fr3_joint4"], -0.01)
        for joint in JOINTS:
            if joint != "fr3_joint4":
                self.assertAlmostEqual(target[joint], 0.0)

    def test_rejects_unconfigured_step(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "configured bounded increments"):
            build_jog_target(self.current, "fr3_joint1", 1.0, 0.1)

    def test_session_budget_rejects_command_count_and_cumulative_limit(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "command limit"):
            validate_session_budget(
                20,
                0.1,
                0.01,
                max_commands=20,
                max_cumulative_delta=0.2,
            )
        with self.assertRaisesRegex(RuntimeError, "cumulative-delta"):
            validate_session_budget(
                1,
                0.195,
                0.01,
                max_commands=20,
                max_cumulative_delta=0.2,
            )

    def test_trajectory_start_validation(self) -> None:
        trajectory = SimpleNamespace(
            joint_trajectory=SimpleNamespace(
                joint_names=list(JOINTS),
                points=[SimpleNamespace(positions=[0.0] * 7)],
            )
        )
        self.assertAlmostEqual(
            validate_trajectory_start(trajectory, self.current, 0.01),
            0.0,
        )
        trajectory.joint_trajectory.points[0].positions[2] = 0.02
        with self.assertRaisesRegex(RuntimeError, "start error"):
            validate_trajectory_start(trajectory, self.current, 0.01)

    def test_velocity_check_requires_all_joints(self) -> None:
        velocities = {joint: 0.001 for joint in JOINTS}
        velocities["fr3_joint5"] = -0.003
        joint, maximum = maximum_absolute_velocity(velocities)
        self.assertEqual(joint, "fr3_joint5")
        self.assertAlmostEqual(maximum, 0.003)
        del velocities["fr3_joint7"]
        with self.assertRaisesRegex(RuntimeError, "incomplete"):
            maximum_absolute_velocity(velocities)

    def test_measured_gripper_width_uses_both_fingers(self) -> None:
        message = JointState()
        message.name = ["fr3_finger_joint2", "fr3_finger_joint1"]
        message.position = [0.034, 0.035]
        self.assertAlmostEqual(measured_gripper_width(message), 0.069)

    def test_measured_gripper_width_requires_both_fingers(self) -> None:
        message = JointState()
        message.name = ["fr3_finger_joint1"]
        message.position = [0.035]
        with self.assertRaisesRegex(RuntimeError, "missing positions"):
            measured_gripper_width(message)

    def test_enabled_gripper_requires_explicit_grasp_width_and_force(self) -> None:
        args = SimpleNamespace(
            enable_gripper=True,
            gripper_open_width=0.07,
            gripper_grasp_width=None,
            gripper_speed=0.02,
            gripper_force=None,
            gripper_epsilon_inner=0.01,
            gripper_epsilon_outer=0.01,
            gripper_max_width=0.08,
            gripper_min_force=30.0,
            gripper_max_force=70.0,
            gripper_state_timeout=3.0,
            gripper_open_tolerance=0.005,
        )
        with self.assertRaisesRegex(RuntimeError, "gripper-force"):
            validate_gripper_configuration(args)
        args.gripper_force = 30.0
        with self.assertRaisesRegex(RuntimeError, "gripper-grasp-width"):
            validate_gripper_configuration(args)
        args.gripper_grasp_width = 0.04
        validate_gripper_configuration(args)

    def test_gripper_configuration_rejects_guard_limit_violations(self) -> None:
        args = SimpleNamespace(
            enable_gripper=True,
            gripper_open_width=0.09,
            gripper_grasp_width=0.04,
            gripper_speed=0.02,
            gripper_force=10.0,
            gripper_epsilon_inner=0.01,
            gripper_epsilon_outer=0.01,
            gripper_max_width=0.08,
            gripper_min_force=30.0,
            gripper_max_force=70.0,
            gripper_state_timeout=3.0,
            gripper_open_tolerance=0.005,
        )
        with self.assertRaisesRegex(RuntimeError, "open-width exceeds"):
            validate_gripper_configuration(args)

    def test_gripper_configuration_rejects_force_outside_documented_range(self) -> None:
        args = SimpleNamespace(
            enable_gripper=True,
            gripper_open_width=0.07,
            gripper_grasp_width=0.04,
            gripper_speed=0.02,
            gripper_force=20.0,
            gripper_epsilon_inner=0.01,
            gripper_epsilon_outer=0.01,
            gripper_max_width=0.08,
            gripper_min_force=30.0,
            gripper_max_force=70.0,
            gripper_state_timeout=3.0,
            gripper_open_tolerance=0.005,
        )
        with self.assertRaisesRegex(RuntimeError, "below --gripper-min-force"):
            validate_gripper_configuration(args)


if __name__ == "__main__":
    unittest.main()
