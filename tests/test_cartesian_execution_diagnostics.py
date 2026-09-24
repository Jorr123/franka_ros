#!/usr/bin/env python3

import pathlib
from types import SimpleNamespace
import sys
import tempfile
import unittest


SCRIPTS_DIR = pathlib.Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from cartesian_execution_diagnostics import (  # noqa: E402
    JOINTS,
    CartesianTrackingSample,
    ControllerRuntimeConfiguration,
    ControllerStateSample,
    PIDGains,
    analyze_execution_tracking,
    build_cartesian_tracking_samples,
    controller_state_sample_from_message,
    format_controller_configuration,
    franka_inertia_configuration_from_robot_state,
    maximum_joint_velocity,
    write_execution_trace_csv,
)


def point(*, positions=(), velocities=(), effort=()):
    return SimpleNamespace(
        positions=list(positions),
        velocities=list(velocities),
        accelerations=[],
        effort=list(effort),
    )


def controller_message(
    *,
    desired_positions,
    actual_positions,
    errors,
):
    zeros = [0.0] * len(JOINTS)
    reference = point(
        positions=desired_positions,
        velocities=zeros,
    )
    feedback = point(
        positions=actual_positions,
        velocities=zeros,
    )
    return SimpleNamespace(
        joint_names=list(JOINTS),
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=12, nanosec=500_000_000)
        ),
        reference=reference,
        feedback=feedback,
        desired=point(),
        actual=point(),
        error=point(positions=errors, velocities=zeros),
        output=point(effort=zeros),
    )


def sample(
    receipt_time,
    desired,
    actual,
    *,
    desired_velocities=None,
    actual_velocities=None,
):
    errors = {
        joint: desired[joint] - actual[joint] for joint in JOINTS
    }
    zeros = {joint: 0.0 for joint in JOINTS}
    desired_velocities = desired_velocities or zeros
    actual_velocities = actual_velocities or zeros
    return ControllerStateSample(
        receipt_time=receipt_time,
        message_time=receipt_time,
        desired_positions=dict(desired),
        actual_positions=dict(actual),
        position_errors=errors,
        desired_velocities=dict(desired_velocities),
        actual_velocities=dict(actual_velocities),
        velocity_errors={
            joint: desired_velocities[joint] - actual_velocities[joint]
            for joint in JOINTS
        },
        output_efforts=zeros,
    )


class FakeFK:
    def position(self, positions):
        return (
            positions["fr3_joint1"],
            positions["fr3_joint2"],
            0.3 + positions["fr3_joint3"],
        )


class CartesianExecutionDiagnosticsTests(unittest.TestCase):
    def test_maximum_measured_velocity_is_reported(self):
        velocities = {joint: 0.001 for joint in JOINTS}
        velocities["fr3_joint4"] = -0.008
        joint, value = maximum_joint_velocity(velocities)
        self.assertEqual(joint, "fr3_joint4")
        self.assertAlmostEqual(value, 0.008)

    def test_controller_state_records_desired_actual_and_error(self):
        desired = [0.01 * index for index in range(len(JOINTS))]
        actual = [value - 0.002 for value in desired]
        errors = [0.002] * len(JOINTS)
        result = controller_state_sample_from_message(
            controller_message(
                desired_positions=desired,
                actual_positions=actual,
                errors=errors,
            ),
            receipt_time=20.0,
        )
        self.assertAlmostEqual(result.message_time, 12.5)
        self.assertAlmostEqual(
            result.desired_positions["fr3_joint7"],
            desired[-1],
        )
        self.assertAlmostEqual(
            result.actual_positions["fr3_joint7"],
            actual[-1],
        )
        self.assertAlmostEqual(
            result.position_errors["fr3_joint7"],
            0.002,
        )

    def test_all_three_franka_inertias_are_recorded(self):
        def stamped(mass, frame):
            inertia = SimpleNamespace(
            m=mass,
            com=SimpleNamespace(x=0.01, y=-0.02, z=0.03),
            ixx=0.1,
            ixy=0.01,
            ixz=0.02,
            iyy=0.2,
            iyz=0.03,
            izz=0.3,
            )
            return SimpleNamespace(
                header=SimpleNamespace(frame_id=frame),
                inertia=inertia,
            )

        message = SimpleNamespace(
            inertia_ee=stamped(0.7, "fr3_ee"),
            inertia_load=stamped(1.2, "fr3_load"),
            inertia_total=stamped(1.9, "fr3_total"),
        )
        configuration = franka_inertia_configuration_from_robot_state(
            message
        )
        self.assertAlmostEqual(configuration.end_effector.mass, 0.7)
        self.assertAlmostEqual(configuration.load.mass, 1.2)
        self.assertAlmostEqual(configuration.total.mass, 1.9)
        self.assertEqual(
            configuration.load.center_of_mass,
            (0.01, -0.02, 0.03),
        )
        self.assertEqual(
            configuration.total.inertia[1],
            (0.01, 0.2, 0.03),
        )

    def test_tracking_report_finds_dip_and_first_growing_joint_error(self):
        desired_states = []
        actual_states = []
        for index in range(4):
            desired = {joint: 0.0 for joint in JOINTS}
            actual = dict(desired)
            desired["fr3_joint2"] = -0.001 * index
            actual["fr3_joint2"] = -0.0005 * index
            desired["fr3_joint3"] = 0.0
            actual["fr3_joint3"] = -0.001 * index
            desired_states.append(desired)
            actual_states.append(actual)

        controller_samples = tuple(
            sample(10.0 + index * 0.02, desired, actual)
            for index, (desired, actual) in enumerate(
                zip(desired_states, actual_states)
            )
        )
        tracking = build_cartesian_tracking_samples(
            controller_samples,
            execution_start_time=10.0,
            fk=FakeFK(),
        )
        report = analyze_execution_tracking(
            tracking,
            start_tcp=(0.0, 0.0, 0.3),
            command_duration=0.06,
            initial_window_duration=0.06,
            joint7_goal_tolerance=0.01,
            error_growth_epsilon=1e-6,
            consecutive_growth_samples=2,
        )
        self.assertAlmostEqual(report.minimum_actual_z_offset, -0.003)
        self.assertAlmostEqual(report.minimum_actual_z_time, 0.06)
        self.assertAlmostEqual(report.desired_z_offset_at_minimum, 0.0)
        self.assertEqual(report.first_increasing_error_joint, "fr3_joint2")
        self.assertAlmostEqual(
            report.maximum_joint_errors["fr3_joint3"],
            0.003,
        )
        self.assertGreater(report.maximum_cartesian_error, 0.003)

    def test_joint7_behavior_reports_stiction_and_continuous_lag(self):
        controller_samples = []
        for index in range(7):
            desired = {joint: 0.0 for joint in JOINTS}
            actual = dict(desired)
            desired["fr3_joint7"] = -0.001 * index
            actual["fr3_joint7"] = (
                0.0 if index < 4 else -0.0002 * (index - 3)
            )
            desired_velocity = {joint: 0.0 for joint in JOINTS}
            desired_velocity["fr3_joint7"] = -0.01
            controller_samples.append(
                sample(
                    20.0 + 0.1 * index,
                    desired,
                    actual,
                    desired_velocities=desired_velocity,
                )
            )
        tracking = build_cartesian_tracking_samples(
            controller_samples,
            execution_start_time=20.0,
            fk=FakeFK(),
        )
        report = analyze_execution_tracking(
            tracking,
            start_tcp=(0.0, 0.0, 0.3),
            command_duration=0.6,
            initial_window_duration=0.5,
            joint7_goal_tolerance=0.002,
        )
        self.assertTrue(report.joint7.initially_sticks)
        self.assertTrue(report.joint7.continuously_lags)
        self.assertFalse(
            report.joint7.desired_trajectory_discontinuous
        )
        self.assertFalse(report.joint7.reached_then_rebounded)

    def test_official_controller_comparison_separates_tolerances(self):
        gains = {
            joint: PIDGains(
                p=(600.0 if index < 4 else (250.0, 150.0, 50.0)[index - 4]),
                i=0.0,
                d=(30.0 if index < 4 else (10.0, 10.0, 5.0)[index - 4]),
                i_clamp=1.0,
                ff_velocity_scale=0.0,
            )
            for index, joint in enumerate(JOINTS)
        }
        configuration = ControllerRuntimeConfiguration(
            name="fr3_arm_controller",
            state="active",
            controller_type=(
                "joint_trajectory_controller/JointTrajectoryController"
            ),
            configured_command_interfaces=("effort",),
            claimed_interfaces=tuple(
                f"{joint}/effort" for joint in JOINTS
            ),
            effort_interface_confirmed=True,
            gains=gains,
            goal_time_tolerance=2.0,
            stopped_velocity_tolerance=0.01,
            goal_tolerances={joint: 0.01 for joint in JOINTS},
            trajectory_tolerances={joint: 0.05 for joint in JOINTS},
        )
        output = format_controller_configuration(configuration)
        self.assertIn("PID and velocity feedforward: MATCH", output)
        self.assertIn("Effective tolerance defaults: DIFFERS", output)
        self.assertIn("fr3_joint7", output)

    def test_csv_contains_tcp_and_joint_tracking_columns(self):
        desired = {joint: 0.0 for joint in JOINTS}
        actual = dict(desired)
        actual["fr3_joint3"] = -0.001
        controller_samples = (
            sample(1.0, desired, desired),
            sample(1.1, desired, actual),
        )
        tracking = build_cartesian_tracking_samples(
            controller_samples,
            execution_start_time=1.0,
            fk=FakeFK(),
        )
        with tempfile.TemporaryDirectory() as directory:
            output = pathlib.Path(directory) / "trace.csv"
            write_execution_trace_csv(output, tracking)
            contents = output.read_text(encoding="ascii")
        self.assertIn("desired_tcp_z_m", contents)
        self.assertIn("actual_tcp_z_m", contents)
        self.assertIn("fr3_joint7_position_error_rad", contents)
        self.assertEqual(len(contents.splitlines()), 3)


if __name__ == "__main__":
    unittest.main()
