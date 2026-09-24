#!/usr/bin/env python3

import math
import pathlib
from types import SimpleNamespace
import sys
import unittest


SCRIPTS_DIR = pathlib.Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from cartesian_motion_safety import (  # noqa: E402
    format_cartesian_verification_report,
    scale_timed_trajectory,
    trajectory_final_positions,
    validate_trajectory,
    verify_cartesian_motion,
)


JOINTS = [f"fr3_joint{index}" for index in range(1, 8)]


def duration(seconds):
    whole = int(seconds)
    return SimpleNamespace(
        sec=whole,
        nanosec=int(round((seconds - whole) * 1e9)),
    )


def point(time_value, positions, velocities=None, accelerations=None):
    return SimpleNamespace(
        time_from_start=duration(time_value),
        positions=list(positions),
        velocities=list(
            velocities if velocities is not None else [0.0] * len(JOINTS)
        ),
        accelerations=list(
            accelerations if accelerations is not None else [0.0] * len(JOINTS)
        ),
    )


def trajectory(
    *,
    times=(0.0, 0.5, 1.0),
    excursions=(0.0, 0.01, 0.02),
):
    points = []
    for time_value, excursion in zip(times, excursions):
        positions = [0.0] * len(JOINTS)
        positions[2] = excursion
        velocities = [0.0] * len(JOINTS)
        velocities[2] = (
            0.08 if time_value not in (times[0], times[-1]) else 0.0
        )
        accelerations = [0.0] * len(JOINTS)
        accelerations[2] = 0.2 if time_value == times[0] else 0.0
        points.append(point(time_value, positions, velocities, accelerations))
    return SimpleNamespace(
        joint_trajectory=SimpleNamespace(
            joint_names=list(JOINTS),
            points=points,
        )
    )


class CartesianTrajectorySafetyTests(unittest.TestCase):
    def test_scales_existing_timing_and_dynamics(self):
        planned = trajectory()

        factor = scale_timed_trajectory(
            planned,
            max_velocity=0.04,
            max_acceleration=0.05,
        )

        self.assertEqual(factor, 2.0)
        self.assertEqual(
            planned.joint_trajectory.points[-1].time_from_start.sec, 2
        )
        self.assertAlmostEqual(
            planned.joint_trajectory.points[1].velocities[2], 0.04
        )
        self.assertAlmostEqual(
            planned.joint_trajectory.points[0].accelerations[2], 0.05
        )

    def test_rejects_untimed_trajectory(self):
        planned = trajectory(times=(0.0, 0.0, 0.0))
        with self.assertRaisesRegex(ValueError, "untimed|strictly increasing"):
            scale_timed_trajectory(
                planned,
                max_velocity=0.02,
                max_acceleration=0.05,
            )

    def test_accepts_small_complete_fr3_trajectory(self):
        planned = trajectory()
        factor = scale_timed_trajectory(
            planned,
            max_velocity=0.04,
            max_acceleration=0.05,
        )

        report = validate_trajectory(
            planned,
            {joint: 0.0 for joint in JOINTS},
            JOINTS,
            timing_scale=factor,
            max_start_error=0.01,
            max_joint_excursion=0.10,
            max_joint_step=0.03,
            max_planned_velocity=0.15,
            max_planned_acceleration=0.50,
        )

        self.assertEqual(report.point_count, 3)
        self.assertAlmostEqual(report.duration, 2.0)
        self.assertAlmostEqual(report.max_joint_excursion, 0.02)
        self.assertEqual(report.max_joint_excursion_joint, "fr3_joint3")

    def test_rejects_large_joint_excursion(self):
        planned = trajectory(excursions=(0.0, 0.04, 0.12))
        factor = scale_timed_trajectory(
            planned,
            max_velocity=0.04,
            max_acceleration=0.05,
        )

        with self.assertRaisesRegex(ValueError, "moves 0.120000 rad"):
            validate_trajectory(
                planned,
                {joint: 0.0 for joint in JOINTS},
                JOINTS,
                timing_scale=factor,
                max_start_error=0.01,
                max_joint_excursion=0.10,
                max_joint_step=0.10,
                max_planned_velocity=0.15,
                max_planned_acceleration=0.50,
            )

    def test_rejects_start_state_mismatch(self):
        planned = trajectory()
        factor = scale_timed_trajectory(
            planned,
            max_velocity=0.04,
            max_acceleration=0.05,
        )
        current = {joint: 0.0 for joint in JOINTS}
        current["fr3_joint1"] = 0.02

        with self.assertRaisesRegex(ValueError, "planned start differs"):
            validate_trajectory(
                planned,
                current,
                JOINTS,
                timing_scale=factor,
                max_start_error=0.01,
                max_joint_excursion=0.10,
                max_joint_step=0.03,
                max_planned_velocity=0.15,
                max_planned_acceleration=0.50,
            )

    def test_accepts_machine_roundoff_above_acceleration_limit(self):
        planned = trajectory()
        planned.joint_trajectory.points[0].accelerations[2] = math.nextafter(
            0.5, math.inf
        )

        report = validate_trajectory(
            planned,
            {joint: 0.0 for joint in JOINTS},
            JOINTS,
            timing_scale=1.0,
            max_start_error=0.01,
            max_joint_excursion=0.10,
            max_joint_step=0.03,
            max_planned_velocity=0.15,
            max_planned_acceleration=0.50,
        )

        self.assertAlmostEqual(report.max_planned_acceleration, 0.5)

    def test_rejects_material_acceleration_limit_violation(self):
        planned = trajectory()
        planned.joint_trajectory.points[0].accelerations[2] = 0.500001

        with self.assertRaisesRegex(ValueError, "reaches 0.500001"):
            validate_trajectory(
                planned,
                {joint: 0.0 for joint in JOINTS},
                JOINTS,
                timing_scale=1.0,
                max_start_error=0.01,
                max_joint_excursion=0.10,
                max_joint_step=0.03,
                max_planned_velocity=0.15,
                max_planned_acceleration=0.50,
            )

    def test_returns_final_joint_positions_by_name(self):
        final = trajectory_final_positions(trajectory())
        self.assertAlmostEqual(final["fr3_joint3"], 0.02)
        self.assertAlmostEqual(final["fr3_joint7"], 0.0)


class CartesianVerificationTests(unittest.TestCase):
    def test_accepts_measured_motion_near_target(self):
        expected = {joint: 0.0 for joint in JOINTS}
        expected["fr3_joint3"] = 0.02
        measured = dict(expected)
        measured["fr3_joint3"] = 0.019

        report = verify_cartesian_motion(
            start_position=(0.40, 0.00, 0.30),
            target_position=(0.405, 0.00, 0.30),
            final_position=(0.404, 0.0002, 0.30),
            target_orientation=(0.0, 0.0, 0.0, 1.0),
            final_orientation=(0.0, 0.0, 0.0, 1.0),
            expected_joint_positions=expected,
            measured_joint_positions=measured,
            measured_joint_velocities={joint: 0.001 for joint in JOINTS},
            target_error_limit=0.002,
            orientation_error_limit=0.03,
            joint_target_error_limit=0.005,
            max_settled_velocity=0.01,
            minimum_measured_motion=0.001,
        )

        self.assertTrue(report.passed)
        self.assertTrue(report.direction_ok)
        self.assertTrue(report.position_ok)

    def test_rejects_wrong_direction(self):
        expected = {joint: 0.0 for joint in JOINTS}
        report = verify_cartesian_motion(
            start_position=(0.40, 0.00, 0.30),
            target_position=(0.405, 0.00, 0.30),
            final_position=(0.398, 0.00, 0.30),
            target_orientation=(0.0, 0.0, 0.0, 1.0),
            final_orientation=(0.0, 0.0, 0.0, 1.0),
            expected_joint_positions=expected,
            measured_joint_positions=expected,
            measured_joint_velocities={joint: 0.0 for joint in JOINTS},
            target_error_limit=0.01,
            orientation_error_limit=0.03,
            joint_target_error_limit=0.005,
            max_settled_velocity=0.01,
            minimum_measured_motion=0.001,
        )

        self.assertFalse(report.passed)
        self.assertFalse(report.direction_ok)

    def test_controller_failure_cannot_be_reported_as_verified_success(self):
        expected = {joint: 0.0 for joint in JOINTS}
        report = verify_cartesian_motion(
            start_position=(0.40, 0.00, 0.30),
            target_position=(0.405, 0.00, 0.30),
            final_position=(0.405, 0.00, 0.30),
            target_orientation=(0.0, 0.0, 0.0, 1.0),
            final_orientation=(0.0, 0.0, 0.0, 1.0),
            expected_joint_positions=expected,
            measured_joint_positions=expected,
            measured_joint_velocities={joint: 0.0 for joint in JOINTS},
            target_error_limit=0.002,
            orientation_error_limit=0.03,
            joint_target_error_limit=0.005,
            max_settled_velocity=0.01,
            minimum_measured_motion=0.001,
        )

        text = format_cartesian_verification_report(
            report,
            target_error_limit=0.002,
            orientation_error_limit=0.03,
            joint_target_error_limit=0.005,
            max_settled_velocity=0.01,
            execution_succeeded=False,
        )

        self.assertTrue(report.passed)
        self.assertIn("CONTROLLER EXECUTION FAILED", text)
        self.assertNotIn("VERIFIED CARTESIAN SUCCESS", text)


if __name__ == "__main__":
    unittest.main()
