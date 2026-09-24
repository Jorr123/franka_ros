#!/usr/bin/env python3

import pathlib
from types import SimpleNamespace
import sys
import unittest


SCRIPTS_DIR = pathlib.Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from moveit_joint_goal_test import JOINTS  # noqa: E402
from moveit_joint7_plan_diagnostic import (  # noqa: E402
    analyze_joint7_plan,
)


def duration(seconds):
    whole = int(seconds)
    return SimpleNamespace(
        sec=whole,
        nanosec=int(round((seconds - whole) * 1e9)),
    )


def trajectory(q7_values, *, joint3_excursion=0.0):
    points = []
    for index, q7 in enumerate(q7_values):
        positions = [0.0] * len(JOINTS)
        positions[2] = joint3_excursion * index
        positions[6] = q7
        points.append(
            SimpleNamespace(
                positions=positions,
                velocities=[0.0] * len(JOINTS),
                accelerations=[0.0] * len(JOINTS),
                time_from_start=duration(0.1 * (index + 1)),
            )
        )
    return SimpleNamespace(
        joint_trajectory=SimpleNamespace(
            joint_names=list(JOINTS),
            points=points,
        )
    )


class Joint7PlanDiagnosticTests(unittest.TestCase):
    def setUp(self):
        self.current = {joint: 0.0 for joint in JOINTS}
        self.target = dict(self.current)
        self.target["fr3_joint7"] = 0.005

    def test_accepts_monotonic_joint7_only_plan(self):
        report = analyze_joint7_plan(
            trajectory((0.0, 0.0025, 0.005)),
            self.current,
            self.target,
            holding_joint_tolerance=0.0001,
            goal_tolerance=0.002,
        )
        self.assertTrue(report.passed)
        self.assertTrue(report.joint7_monotonic)
        self.assertAlmostEqual(report.joint7_planned_delta, 0.005)
        self.assertAlmostEqual(report.maximum_holding_joint_excursion, 0.0)

    def test_reports_unexpected_motion_of_holding_joint(self):
        report = analyze_joint7_plan(
            trajectory(
                (0.0, 0.0025, 0.005),
                joint3_excursion=0.0002,
            ),
            self.current,
            self.target,
            holding_joint_tolerance=0.0001,
            goal_tolerance=0.002,
        )
        self.assertFalse(report.passed)
        self.assertFalse(report.holding_joints_within_tolerance)
        self.assertEqual(report.maximum_holding_joint, "fr3_joint3")

    def test_reports_nonmonotonic_joint7_plan(self):
        report = analyze_joint7_plan(
            trajectory((0.0, 0.003, 0.002, 0.005)),
            self.current,
            self.target,
            holding_joint_tolerance=0.0001,
            goal_tolerance=0.002,
        )
        self.assertFalse(report.passed)
        self.assertFalse(report.joint7_monotonic)


if __name__ == "__main__":
    unittest.main()
