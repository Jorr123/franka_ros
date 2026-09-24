#!/usr/bin/env python3

import json
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "scripts"
CONFIG = ROOT / "src" / "franka_fr3_moveit_config" / "config"
LAUNCH = ROOT / "src" / "franka_fr3_moveit_config" / "launch"
PACKAGE_XML = ROOT / "src" / "franka_fr3_moveit_config" / "package.xml"
sys.path.insert(0, str(SCRIPTS))

from fr3_keyboard_cartesian_servo import (  # noqa: E402
    CartesianProgressGuard,
    MotionState,
    PilotRecorder,
    apply_decision,
    delta_from_velocity,
    interpret_key,
    parse_args,
    speed_presets,
    validate_args,
)


class KeyboardCartesianServoTests(unittest.TestCase):
    def test_keys_cover_axis_motion_speed_halt_and_quit(self) -> None:
        self.assertEqual(interpret_key("X").value, "x")
        self.assertEqual(interpret_key("LEFT").value, -1.0)
        self.assertEqual(interpret_key("RIGHT").value, 1.0)
        self.assertEqual(interpret_key("f").value, "fine")
        self.assertEqual(interpret_key("N").value, "normal")
        self.assertEqual(interpret_key("c").value, "coarse")
        self.assertEqual(interpret_key("O").kind, "gripper_open")
        self.assertEqual(interpret_key("g").kind, "gripper_grasp")
        self.assertEqual(interpret_key("SPACE").kind, "halt")
        self.assertEqual(interpret_key("q").kind, "quit")

    def test_watchdog_expires_to_zero_velocity(self) -> None:
        args = parse_args([])
        state = MotionState(axis="y", mode="normal")
        apply_decision(
            state,
            interpret_key("RIGHT"),
            now=10.0,
            key_timeout=0.25,
        )
        self.assertEqual(
            state.linear_velocity(10.1, speed_presets(args)),
            (0.0, 0.01, 0.0),
        )
        self.assertEqual(
            state.linear_velocity(10.25, speed_presets(args)),
            (0.0, 0.0, 0.0),
        )

    def test_axis_or_mode_change_halts_existing_motion(self) -> None:
        state = MotionState(axis="y", mode="normal")
        state.refresh(1.0, 10.0, 0.25)
        apply_decision(
            state,
            interpret_key("z"),
            now=10.1,
            key_timeout=0.25,
        )
        self.assertEqual(state.axis, "z")
        self.assertEqual(state.direction, 0.0)

        state.refresh(-1.0, 11.0, 0.25)
        apply_decision(
            state,
            interpret_key("c"),
            now=11.1,
            key_timeout=0.25,
        )
        self.assertEqual(state.mode, "coarse")
        self.assertEqual(state.direction, 0.0)

    def test_velocity_converts_to_relative_delta_at_record_rate(self) -> None:
        self.assertEqual(
            delta_from_velocity((0.01, -0.02, 0.0), 0.1),
            (0.001, -0.002, 0.0),
        )

    def test_defaults_are_bounded_and_dry_run(self) -> None:
        args = parse_args([])
        validate_args(args)
        self.assertFalse(args.execute)
        self.assertEqual(args.command_rate, 20.0)
        self.assertEqual(args.record_rate, 10.0)
        self.assertEqual(args.key_timeout, 0.25)
        self.assertEqual(args.max_linear_speed, 0.02)
        self.assertEqual(args.startup_timeout, 30.0)
        self.assertEqual(args.controller_state_timeout, 0.5)
        self.assertEqual(args.servo_output_timeout, 0.25)
        self.assertEqual(args.tracking_grace_time, 1.0)
        self.assertEqual(args.tracking_min_command_distance, 0.003)
        self.assertEqual(args.minimum_progress_ratio, 0.10)

    def test_invalid_rate_speed_and_recording_configuration_is_rejected(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "record-rate"):
            validate_args(parse_args(["--record-rate", "30"]))
        with self.assertRaisesRegex(RuntimeError, "coarse-speed"):
            validate_args(parse_args(["--max-linear-speed", "0.01"]))
        with self.assertRaisesRegex(RuntimeError, "--task"):
            validate_args(parse_args(["--record-jsonl", "/tmp/test.jsonl"]))
        with self.assertRaisesRegex(RuntimeError, "minimum-progress-ratio"):
            validate_args(parse_args(["--minimum-progress-ratio", "0"]))

    def test_tracking_guard_accepts_measured_cartesian_progress(self) -> None:
        guard = CartesianProgressGuard(
            grace_time=1.0,
            minimum_command_distance=0.003,
            minimum_progress_ratio=0.10,
            maximum_tracking_error=0.010,
        )
        guard.update(
            now=0.0,
            velocity=(0.01, 0.0, 0.0),
            duration=0.0,
            tcp_position=(0.0, 0.0, 0.0),
            servo_status=0,
        )
        assessment = guard.update(
            now=1.0,
            velocity=(0.01, 0.0, 0.0),
            duration=1.0,
            tcp_position=(0.009, 0.0, 0.0),
            servo_status=0,
        )
        self.assertIsNotNone(assessment)
        self.assertIsNone(assessment.failure_reason)
        self.assertAlmostEqual(assessment.progress_ratio, 0.9)

    def test_tracking_guard_rejects_sustained_command_without_motion(self) -> None:
        guard = CartesianProgressGuard(
            grace_time=1.0,
            minimum_command_distance=0.003,
            minimum_progress_ratio=0.10,
            maximum_tracking_error=0.010,
        )
        guard.update(
            now=0.0,
            velocity=(0.01, 0.0, 0.0),
            duration=0.0,
            tcp_position=(0.0, 0.0, 0.0),
            servo_status=0,
        )
        assessment = guard.update(
            now=1.0,
            velocity=(0.01, 0.0, 0.0),
            duration=1.0,
            tcp_position=(0.0001, 0.0, 0.0),
            servo_status=0,
        )
        self.assertIsNotNone(assessment)
        self.assertIn("progress is too small", assessment.failure_reason)

    def test_tracking_guard_defers_to_servo_deceleration_status(self) -> None:
        guard = CartesianProgressGuard(
            grace_time=1.0,
            minimum_command_distance=0.003,
            minimum_progress_ratio=0.10,
            maximum_tracking_error=0.010,
        )
        guard.update(
            now=0.0,
            velocity=(0.0, -0.01, 0.0),
            duration=0.0,
            tcp_position=(0.0, 0.0, 0.0),
            servo_status=3,
        )
        assessment = guard.update(
            now=1.0,
            velocity=(0.0, -0.01, 0.0),
            duration=1.0,
            tcp_position=(0.0, 0.0, 0.0),
            servo_status=3,
        )
        self.assertIsNotNone(assessment)
        self.assertIsNone(assessment.failure_reason)

    def test_tracking_guard_resets_when_command_direction_changes(self) -> None:
        guard = CartesianProgressGuard(
            grace_time=1.0,
            minimum_command_distance=0.003,
            minimum_progress_ratio=0.10,
            maximum_tracking_error=0.010,
        )
        guard.update(
            now=0.0,
            velocity=(0.01, 0.0, 0.0),
            duration=0.5,
            tcp_position=(0.004, 0.0, 0.0),
            servo_status=0,
        )
        assessment = guard.update(
            now=0.6,
            velocity=(-0.01, 0.0, 0.0),
            duration=0.1,
            tcp_position=(0.004, 0.0, 0.0),
            servo_status=0,
        )
        self.assertIsNotNone(assessment)
        self.assertAlmostEqual(assessment.commanded_distance, 0.001)
        self.assertIsNone(assessment.failure_reason)

    def test_enabled_gripper_requires_explicit_grasp_configuration(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "gripper-force"):
            validate_args(parse_args(["--enable-gripper"]))
        with self.assertRaisesRegex(RuntimeError, "gripper-grasp-width"):
            validate_args(
                parse_args(
                    [
                        "--enable-gripper",
                        "--gripper-force",
                        "30",
                    ]
                )
            )
        validate_args(
            parse_args(
                [
                    "--enable-gripper",
                    "--gripper-grasp-width",
                    "0.05",
                    "--gripper-force",
                    "30",
                ]
            )
        )

    def test_pilot_recorder_writes_metadata_frame_and_end(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "episode.jsonl"
            args = parse_args(
                ["--record-jsonl", str(path), "--task", "Pick up the box"]
            )
            recorder = PilotRecorder(path, args)
            recorder.add_frame(
                observation={"joint_positions": [0.0] * 7},
                translation_delta=(0.001, 0.0, 0.0),
                servo_status=0,
            )
            recorder.close("COMPLETED")

            records = [
                json.loads(line)
                for line in path.read_text(encoding="ascii").splitlines()
            ]
            self.assertEqual(
                [record["record_type"] for record in records],
                ["session", "frame", "session_end"],
            )
            self.assertEqual(records[1]["action"]["translation_delta"][0], 0.001)
            self.assertEqual(records[1]["servo_status_name"], "NO_WARNING")
            self.assertEqual(records[1]["action"]["frame"], "base")

            with self.assertRaises(FileExistsError):
                PilotRecorder(path, args)

    def test_pilot_recorder_captures_gripper_command(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "episode.jsonl"
            args = parse_args(
                [
                    "--record-jsonl",
                    str(path),
                    "--task",
                    "Pick up the box",
                    "--enable-gripper",
                    "--gripper-grasp-width",
                    "0.05",
                    "--gripper-force",
                    "30",
                ]
            )
            recorder = PilotRecorder(path, args)
            recorder.add_frame(
                observation=None,
                translation_delta=(0.0, 0.0, 0.0),
                servo_status=0,
                gripper_command="grasp",
            )
            recorder.close("COMPLETED")
            records = [
                json.loads(line)
                for line in path.read_text(encoding="ascii").splitlines()
            ]
            self.assertEqual(
                records[0]["action_space"],
                "base_relative_cartesian_translation_with_gripper",
            )
            self.assertEqual(records[1]["action"]["gripper_command"], "grasp")

    def test_pilot_recorder_preserves_configured_action_frame(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "episode.jsonl"
            args = parse_args(
                [
                    "--planning-frame",
                    "custom_base",
                    "--record-jsonl",
                    str(path),
                    "--task",
                    "Move the TCP",
                ]
            )
            recorder = PilotRecorder(path, args)
            recorder.add_frame(
                observation=None,
                translation_delta=(0.0, 0.0, 0.0),
                servo_status=None,
            )
            recorder.close("COMPLETED")
            records = [
                json.loads(line)
                for line in path.read_text(encoding="ascii").splitlines()
            ]
            self.assertEqual(records[0]["action_frame"], "custom_base")
            self.assertEqual(records[1]["action"]["frame"], "custom_base")

    def test_servo_config_targets_existing_fr3_trajectory_controller(self) -> None:
        config = yaml.safe_load(
            (CONFIG / "fr3_servo.yaml").read_text(encoding="ascii")
        )
        self.assertEqual(config["command_in_type"], "speed_units")
        self.assertEqual(
            config["command_out_type"], "trajectory_msgs/JointTrajectory"
        )
        self.assertEqual(
            config["command_out_topic"],
            "/fr3_arm_controller/joint_trajectory",
        )
        self.assertEqual(config["move_group_name"], "fr3_arm")
        self.assertEqual(config["planning_frame"], "base")
        self.assertEqual(config["ee_frame_name"], "fr3_hand_tcp")
        self.assertEqual(config["joint_topic"], "/joint_states")
        self.assertTrue(config["check_collisions"])
        self.assertLessEqual(config["incoming_command_timeout"], 0.25)
        self.assertFalse(config["is_primary_planning_scene_monitor"])

    def test_standalone_launch_does_not_start_another_controller(self) -> None:
        source = (LAUNCH / "servo.launch.py").read_text(encoding="ascii")
        self.assertIn('package="moveit_servo"', source)
        self.assertIn('executable="servo_node_main"', source)
        self.assertNotIn('package="controller_manager"', source)
        self.assertNotIn("ros2_control_node", source)

    def test_package_declares_moveit_servo_dependency(self) -> None:
        root = ET.parse(PACKAGE_XML).getroot()
        dependencies = {
            element.text for element in root.findall("exec_depend")
        }
        self.assertIn("moveit_servo", dependencies)


if __name__ == "__main__":
    unittest.main()
