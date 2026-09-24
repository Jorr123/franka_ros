#!/usr/bin/env python3

import sys
import unittest
from pathlib import Path


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from joint_motion_verification import (  # noqa: E402
    DEFAULT_JOINT7_TARGET_ERROR_LIMIT,
    DEFAULT_TARGET_ERROR_LIMIT,
    verification_report_payload,
    verify_joint_motion,
)


JOINTS = [f"fr3_joint{index}" for index in range(1, 8)]


class JointMotionVerificationTest(unittest.TestCase):
    def setUp(self) -> None:
        self.before = {joint: 0.0 for joint in JOINTS}
        self.target = dict(self.before)
        self.target["fr3_joint3"] = 0.03
        self.after = dict(self.before)
        self.after["fr3_joint3"] = 0.028
        self.velocity = {joint: 0.001 for joint in JOINTS}

    def verify(self):
        return verify_joint_motion(
            JOINTS,
            self.before,
            self.target,
            self.after,
            self.velocity,
            target_error_limit=0.005,
            non_commanded_drift_limit=0.01,
            max_settled_velocity=0.01,
            minimum_measured_delta=0.001,
        )

    def test_prototype_default_target_error_limits_are_point_one_rad(self) -> None:
        self.assertAlmostEqual(DEFAULT_TARGET_ERROR_LIMIT, 0.01)
        self.assertAlmostEqual(DEFAULT_JOINT7_TARGET_ERROR_LIMIT, 0.01)

    def test_expected_motion_passes(self) -> None:
        self.assertTrue(self.verify().passed)

    def test_report_payload_preserves_measured_motion(self) -> None:
        payload = verification_report_payload(self.verify())
        self.assertEqual(payload["status"], "PASSED")
        self.assertEqual(payload["commanded_joints"][0]["joint"], "fr3_joint3")
        self.assertAlmostEqual(
            payload["commanded_joints"][0]["measured_delta"],
            0.028,
        )
        self.assertTrue(payload["settled_velocity_ok"])

    def test_wrong_direction_fails(self) -> None:
        self.after["fr3_joint3"] = -0.01
        report = self.verify()
        self.assertFalse(report.passed)
        self.assertFalse(report.commanded_joints[0].direction_ok)

    def test_large_target_error_fails(self) -> None:
        self.after["fr3_joint3"] = 0.02
        report = self.verify()
        self.assertFalse(report.passed)
        self.assertFalse(report.target_state_passed)
        self.assertFalse(report.commanded_joints[0].target_ok)

    def test_joint7_can_use_a_separate_target_error_limit(self) -> None:
        self.target["fr3_joint3"] = 0.0
        self.after["fr3_joint3"] = 0.0
        self.target["fr3_joint7"] = 0.05
        self.after["fr3_joint7"] = 0.043
        report = verify_joint_motion(
            JOINTS,
            self.before,
            self.target,
            self.after,
            self.velocity,
            target_error_limit=0.005,
            joint_target_error_limits={"fr3_joint7": 0.0075},
            non_commanded_drift_limit=0.01,
            max_settled_velocity=0.01,
            minimum_measured_delta=0.001,
        )

        self.assertTrue(report.passed)
        self.assertAlmostEqual(
            report.commanded_joints[0].target_error_limit,
            0.0075,
        )

    def test_joint7_separate_limit_does_not_loosen_other_joints(self) -> None:
        self.after["fr3_joint3"] = 0.023
        report = verify_joint_motion(
            JOINTS,
            self.before,
            self.target,
            self.after,
            self.velocity,
            target_error_limit=0.005,
            joint_target_error_limits={"fr3_joint7": 0.0075},
            non_commanded_drift_limit=0.01,
            max_settled_velocity=0.01,
            minimum_measured_delta=0.001,
        )

        self.assertFalse(report.passed)
        self.assertAlmostEqual(
            report.commanded_joints[0].target_error_limit,
            0.005,
        )

    def test_non_commanded_joint_drift_fails(self) -> None:
        self.after["fr3_joint2"] = 0.011
        report = self.verify()
        self.assertFalse(report.passed)
        self.assertFalse(report.non_commanded_drift_ok)

    def test_unsettled_velocity_fails(self) -> None:
        self.velocity["fr3_joint6"] = 0.011
        report = self.verify()
        self.assertFalse(report.passed)
        self.assertFalse(report.settled_velocity_ok)

    def test_missing_velocity_is_rejected(self) -> None:
        del self.velocity["fr3_joint7"]
        with self.assertRaisesRegex(ValueError, "missing joints"):
            self.verify()

    def test_tiny_ik_component_is_checked_as_non_commanded_drift(self) -> None:
        self.target["fr3_joint7"] = 0.00005
        self.after["fr3_joint7"] = 0.0
        report = verify_joint_motion(
            JOINTS,
            self.before,
            self.target,
            self.after,
            self.velocity,
            target_error_limit=0.007,
            non_commanded_drift_limit=0.01,
            max_settled_velocity=0.01,
            minimum_measured_delta=0.001,
            command_delta_threshold=0.001,
        )

        self.assertTrue(report.passed)
        self.assertEqual(
            [result.joint for result in report.commanded_joints],
            ["fr3_joint3"],
        )

    def test_command_threshold_does_not_hide_joint_drift(self) -> None:
        self.target["fr3_joint7"] = 0.00005
        self.after["fr3_joint7"] = 0.011
        report = verify_joint_motion(
            JOINTS,
            self.before,
            self.target,
            self.after,
            self.velocity,
            target_error_limit=0.007,
            non_commanded_drift_limit=0.01,
            max_settled_velocity=0.01,
            minimum_measured_delta=0.001,
            command_delta_threshold=0.001,
        )

        self.assertFalse(report.passed)
        self.assertFalse(report.target_state_passed)
        self.assertFalse(report.non_commanded_drift_ok)

    def test_redundant_ik_target_state_can_pass_without_component_motion(self) -> None:
        self.target["fr3_joint6"] = 0.003
        self.after["fr3_joint6"] = -0.0001
        report = self.verify()

        joint6 = next(
            result
            for result in report.commanded_joints
            if result.joint == "fr3_joint6"
        )
        self.assertFalse(joint6.direction_ok)
        self.assertFalse(joint6.movement_ok)
        self.assertTrue(joint6.target_ok)
        self.assertFalse(report.passed)
        self.assertTrue(report.target_state_passed)


if __name__ == "__main__":
    unittest.main()
