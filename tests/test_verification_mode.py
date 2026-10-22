"""Exercise real execution result handling with fake ROS/MoveIt measurements."""

from types import SimpleNamespace
import unittest
from unittest import mock

from tests import test_cosmos_action_planning as fixtures
from fr3_ai_supervisor import RosMoveItPlanner, ExecutionRejected
from fr3_cosmos_action_server import parse_args
from joint_motion_verification import verify_joint_motion, verification_report_payload


class VerificationModeTest(unittest.TestCase):
    def run_execution(self, mode="enforce", *, error=0.003, velocity=0.0, control_code=1,
                      measurement_error=False, drift=0.0):
        planner = object.__new__(RosMoveItPlanner)
        planner.args = parse_args(["--verification-mode", mode])
        planner.args.enable_local_execution = True
        planner.JOINTS = ["fr3_joint1", "fr3_joint2"]
        before = dict.fromkeys(planner.JOINTS, 0.0)
        target = {"fr3_joint1": 0.02, "fr3_joint2": 0.0}
        after = {"fr3_joint1": 0.02 - error, "fr3_joint2": drift}
        velocities = dict.fromkeys(planner.JOINTS, velocity)
        start = SimpleNamespace(positions=before, velocities=dict.fromkeys(planner.JOINTS, 0.0))
        final = SimpleNamespace(positions=after, velocities=velocities)
        report = verify_joint_motion(
            planner.JOINTS, before, target, after, velocities,
            target_error_limit=0.002, non_commanded_drift_limit=0.002,
            max_settled_velocity=0.01, minimum_measured_delta=0.001,
        )
        planner.node = mock.Mock(joint_state_sequence=1)
        planner.node.wait_for_joint_state_snapshot.return_value = start
        planner.node.execute.return_value = SimpleNamespace(error_code=SimpleNamespace(val=control_code))
        planner.MoveItErrorCodes = SimpleNamespace(SUCCESS=1)
        planner.error_name = lambda code: "SUCCESS" if code == 1 else "CONTROL_FAILED"
        planner.before_execute = mock.Mock()
        planner.collect_post_execution_verification = mock.Mock(return_value=(report, final))
        if measurement_error:
            planner.collect_post_execution_verification.side_effect = RuntimeError("No fresh state")
        planner.verification_report_payload = verification_report_payload
        context = fixtures.ExecutionContext("fake trajectory", before, target)
        return planner.execute(context)

    def test_default_mode_enforces_accuracy_and_preserves_details(self):
        self.assertEqual(parse_args([]).verification_mode, "enforce")
        with self.assertRaises(ExecutionRejected) as caught:
            self.run_execution()
        self.assertEqual(caught.exception.code, "VERIFICATION_FAILED")
        self.assertEqual(caught.exception.details["verification"]["status"], "FAILED")

    def test_report_only_records_failed_accuracy(self):
        result = self.run_execution("report-only")
        self.assertEqual(result["status"], "EXECUTED_WITH_WARNINGS")
        self.assertEqual(result["verification"]["status"], "FAILED")
        self.assertAlmostEqual(result["verification"]["commanded_joints"][0]["target_error"], 0.003)

    def test_passing_motion_remains_verified_in_both_modes(self):
        for mode in ("enforce", "report-only"):
            with self.subTest(mode=mode):
                self.assertEqual(self.run_execution(mode, error=0.0)["status"], "VERIFIED_SUCCESS")

    def test_report_only_does_not_bypass_hard_failures(self):
        for changes, code in [
            ({"velocity": 0.02}, "VERIFICATION_FAILED"),
            ({"drift": 0.003}, "VERIFICATION_FAILED"),
            ({"error": 0.11}, "VERIFICATION_FAILED"),
            ({"control_code": -4}, "MOVEIT_EXECUTION_FAILED"),
            ({"measurement_error": True}, "VERIFICATION_ERROR"),
        ]:
            with self.subTest(changes=changes), self.assertRaises(ExecutionRejected) as caught:
                self.run_execution("report-only", **changes)
            self.assertEqual(caught.exception.code, code)


if __name__ == "__main__":
    unittest.main()
