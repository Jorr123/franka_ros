#!/usr/bin/env python3

from pathlib import Path
import subprocess
import sys
import unittest


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from fr3_keyboard_cartesian_jog import (  # noqa: E402
    STEP_SIZES,
    build_gripper_command,
    build_cartesian_command,
    interpret_key,
    parse_args,
    requires_per_jog_confirmation,
    run_guarded_cartesian_jog,
    run_gripper_command,
    validate_args,
    validate_gripper_configuration,
    validate_session_budget,
)


class KeyboardCartesianJogTests(unittest.TestCase):
    def test_axis_direction_mode_and_stop_keys(self) -> None:
        self.assertEqual(interpret_key("x").value, "x")
        self.assertEqual(interpret_key("Y").value, "y")
        self.assertEqual(interpret_key("z").value, "z")
        self.assertEqual(interpret_key("LEFT").value, -1.0)
        self.assertEqual(interpret_key("RIGHT").value, 1.0)
        self.assertEqual(interpret_key("f").value, "fine")
        self.assertEqual(interpret_key("N").value, "normal")
        self.assertEqual(interpret_key("c").value, "coarse")
        self.assertEqual(interpret_key("SPACE").kind, "stop")

    def test_configured_increment_values(self) -> None:
        self.assertEqual(
            STEP_SIZES,
            {"fine": 0.010, "normal": 0.025, "coarse": 0.050},
        )

    def test_gripper_keys(self) -> None:
        self.assertEqual(interpret_key("o").kind, "gripper_open")
        self.assertEqual(interpret_key("G").kind, "gripper_grasp")

    def test_plan_only_command_uses_pilz_lin_and_one_axis(self) -> None:
        args = parse_args([])
        command = build_cartesian_command(args, "y", -1.0, 0.010)
        self.assertEqual(command[command.index("--path-mode") + 1], "pilz-lin")
        self.assertEqual(float(command[command.index("--dy") + 1]), -0.010)
        self.assertNotIn("--dx", command)
        self.assertNotIn("--dz", command)
        self.assertNotIn("--execute", command)

    def test_execute_command_forwards_guards_without_a_shell(self) -> None:
        args = parse_args(
            [
                "--execute",
                "--max-wrist-joint-delta",
                "0.06",
                "--max-total-joint-delta",
                "0.4",
            ]
        )
        calls = []

        def runner(command, *, check):
            calls.append((command, check))
            return subprocess.CompletedProcess(command, 0)

        run_guarded_cartesian_jog(args, "z", 1.0, 0.025, runner=runner)
        command, check = calls[0]
        self.assertFalse(check)
        self.assertIn("--execute", command)
        self.assertEqual(
            command[command.index("--max-wrist-joint-delta") + 1], "0.06"
        )
        self.assertEqual(
            command[command.index("--max-total-joint-delta") + 1], "0.4"
        )

    def test_child_failure_terminates_the_jog(self) -> None:
        args = parse_args([])

        def runner(command, *, check):
            return subprocess.CompletedProcess(command, 2)

        with self.assertRaisesRegex(RuntimeError, "exit code 2"):
            run_guarded_cartesian_jog(
                args, "x", 1.0, 0.010, runner=runner
            )

    def test_gripper_requires_explicit_width_and_force(self) -> None:
        args = parse_args(["--enable-gripper"])
        with self.assertRaisesRegex(RuntimeError, "gripper-grasp-width"):
            validate_gripper_configuration(args)
        args.gripper_grasp_width = 0.035
        with self.assertRaisesRegex(RuntimeError, "gripper-force"):
            validate_gripper_configuration(args)

    def test_gripper_command_forwards_configuration_and_execution(self) -> None:
        args = parse_args(
            [
                "--enable-gripper",
                "--gripper-grasp-width",
                "0.035",
                "--gripper-force",
                "30",
                "--execute",
            ]
        )
        command = build_gripper_command(args, "grasp")
        self.assertIn("--execute", command)
        self.assertEqual(
            command[command.index("--gripper-grasp-width") + 1], "0.035"
        )
        self.assertEqual(command[command.index("--gripper-force") + 1], "30.0")

        calls = []

        def runner(child_command, *, check):
            calls.append((child_command, check))
            return subprocess.CompletedProcess(child_command, 0)

        run_gripper_command(args, "open", runner=runner)
        self.assertFalse(calls[0][1])

    def test_gripper_child_failure_terminates_session(self) -> None:
        args = parse_args(
            [
                "--enable-gripper",
                "--gripper-grasp-width",
                "0.035",
                "--gripper-force",
                "30",
            ]
        )

        def runner(command, *, check):
            return subprocess.CompletedProcess(command, 1)

        with self.assertRaisesRegex(RuntimeError, "Gripper grasp command failed"):
            run_gripper_command(args, "grasp", runner=runner)

    def test_session_budget_rejects_count_and_cumulative_distance(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "command limit"):
            validate_session_budget(
                20,
                0.05,
                0.005,
                max_commands=20,
                max_cumulative_distance=0.20,
            )

    def test_cumulative_distance_limit_can_be_explicitly_disabled(self) -> None:
        validate_session_budget(
            19,
            10.0,
            0.050,
            max_commands=20,
            max_cumulative_distance=None,
        )
        args = parse_args(["--no-cumulative-distance-limit"])
        validate_args(args)
        self.assertTrue(args.no_cumulative_distance_limit)
        with self.assertRaisesRegex(RuntimeError, "cumulative-distance"):
            validate_session_budget(
                1,
                0.195,
                0.010,
                max_commands=20,
                max_cumulative_distance=0.20,
            )

    def test_auto_execute_requires_execution_and_skips_confirmation(self) -> None:
        args = parse_args(["--auto-execute"])
        with self.assertRaisesRegex(RuntimeError, "requires --execute"):
            validate_args(args)
        self.assertTrue(
            requires_per_jog_confirmation(execute=True, auto_execute=False)
        )
        self.assertFalse(
            requires_per_jog_confirmation(execute=True, auto_execute=True)
        )

    def test_max_distance_must_cover_the_largest_increment(self) -> None:
        args = parse_args(["--max-distance", "0.01"])
        with self.assertRaisesRegex(RuntimeError, "coarse step"):
            validate_args(args)


if __name__ == "__main__":
    unittest.main()
