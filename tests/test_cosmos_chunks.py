"""Chunk timing, path validation and Ubuntu adapter tests without ROS hardware."""

from copy import deepcopy
from types import SimpleNamespace, ModuleType
import sys
import unittest
from unittest import mock

from tests import test_cosmos_action_planning as fixtures
from robot_bridge.chunks import timed_chunk
from robot_bridge.chunk_planning import prepare_chunk
from robot_bridge.protocol import JOINTS, Limits, ProposalError, evaluate_proposal, validate_shape, PLAN_MODE
from fr3_ai_supervisor import PlanInputRejected
from fr3_cosmos_action_server import parse_args


class ChunkTest(unittest.TestCase):
    def test_preserves_every_target_and_continuous_interior_velocity(self):
        targets = [[0.002 * i] * 7 for i in (1, 2, 3, 4)]
        samples, metadata = timed_chunk([0.0] * 7, targets, 1 / 15, [0.04] * 7, [0.1] * 7)
        self.assertGreater(metadata["time_scale"], 1)
        for index, target in zip(metadata["waypoint_sample_indices"], targets):
            self.assertEqual(samples[index][1], target)
        self.assertTrue(all(v > 0 for v in samples[metadata["waypoint_sample_indices"][0]][2]))
        self.assertTrue(all(abs(v) < 1e-10 for v in samples[0][2] + samples[-1][2]))
        for previous, current in zip(samples, samples[1:]):
            self.assertGreater(current[0], previous[0])
            self.assertLessEqual(max(abs(b - a) for a, b in zip(previous[1], current[1])), 0.001 + 1e-12)
            self.assertLessEqual(current[0] - previous[0], 0.05 + 1e-12)
        for _, q, v, a in samples:
            self.assertTrue(all(0 <= x <= 0.008 for x in q))
            self.assertLessEqual(max(map(abs, v)), 0.04)
            self.assertLessEqual(max(map(abs, a)), 0.1)

    def test_reversal_is_bounded_and_has_zero_turning_velocity(self):
        samples, meta = timed_chunk([0.0] * 7, [[0.01] * 7, [-0.01] * 7], 0.1, [0.04] * 7, [0.1] * 7)
        self.assertTrue(all(abs(v) < 1e-12 for v in samples[meta["waypoint_sample_indices"][0]][2]))
        self.assertTrue(all(-0.01 <= q <= 0.01 for sample in samples for q in sample[1]))

    def proposal(self):
        core = fixtures.CoreTest()
        core.setUp()
        message = core.proposal(0.002)
        q = message["basis_state"]["joint_positions_rad"]
        middle = list(q)
        middle[0] += 0.001
        message["action_chunk"] = {"joint_positions_rad": [middle, message["target_joint_positions_rad"]],
                                   "action_period_seconds": 1 / 15}
        return message, core

    def test_invalid_interior_is_rejected_even_when_final_target_is_small(self):
        message, core = self.proposal()
        message["action_chunk"]["joint_positions_rad"][0][0] += 0.2
        with self.assertRaisesRegex(ProposalError, "chunk waypoint"):
            evaluate_proposal(message, core.snapshot()[1], Limits(), message["created_at_unix_ms"], mode=PLAN_MODE)

    def test_protocol_rejects_bad_count_timing_and_endpoint(self):
        original, _ = self.proposal()
        for field, value in (("joint_positions_rad", []), ("action_period_seconds", float("nan")),
                             ("action_period_seconds", 0.0)):
            message = deepcopy(original)
            message["action_chunk"][field] = value
            with self.subTest(field=field, value=value), self.assertRaises(ProposalError):
                validate_shape(message, expected_mode=PLAN_MODE)
        original["action_chunk"]["joint_positions_rad"][-1] = [0.0] * 7
        with self.assertRaises(ProposalError):
            validate_shape(original, expected_mode=PLAN_MODE)

    def prepare(self, *, collision=False, middle_delta=0.001, recheck=False):
        message, core = self.proposal()
        message["action_chunk"]["joint_positions_rad"][0][0] = message["basis_state"]["joint_positions_rad"][0] + middle_delta
        current = dict(zip(JOINTS, message["basis_state"]["joint_positions_rad"]))
        target = dict(zip(JOINTS, message["target_joint_positions_rad"]))
        planner = object.__new__(fixtures.CosmosMoveItPlanner)
        planner.args = parse_args([])
        planner.args.enable_local_execution = True
        planner.limits = Limits()
        planner.snapshot = core.snapshot
        planner.validate_deltas = lambda values, args: None
        planner.validate_target_limits = mock.Mock()
        planner.node = mock.Mock(joint_state_sequence=1)
        planner.node.wait_for_joint_state_snapshot.return_value = SimpleNamespace(positions=current, velocities=dict.fromkeys(JOINTS, 0.0))
        validity = mock.Mock()
        validity.wait_for_service.return_value = True
        future = mock.Mock()
        future.done.return_value = True
        future.result.return_value = SimpleNamespace(valid=not collision)
        validity.call_async.return_value = future
        planner.node.create_client.return_value = validity
        planner.rclpy = mock.Mock()

        class RobotTrajectory:
            def __init__(self):
                self.joint_trajectory = SimpleNamespace(joint_names=[], points=[])

        class Point:
            pass

        class Request:
            def __init__(self):
                self.robot_state = SimpleNamespace(joint_state=SimpleNamespace())

        def module(**attributes):
            result = ModuleType("fake_ros")
            result.__dict__.update(attributes)
            return result

        modules = {
            "builtin_interfaces.msg": module(Duration=lambda **kw: SimpleNamespace(**kw)),
            "moveit_msgs.msg": module(RobotTrajectory=RobotTrajectory),
            "moveit_msgs.srv": module(GetStateValidity=SimpleNamespace(Request=Request)),
            "trajectory_msgs.msg": module(JointTrajectoryPoint=Point),
            "yaml": module(safe_load=lambda _: {"joint_limits": {j: {
                "has_velocity_limits": True, "has_acceleration_limits": True,
                "max_velocity": 0.2, "max_acceleration": 0.5} for j in JOINTS}}),
        }
        envelope = {"request_id": message["request_id"], "cosmos_proposal": message,
                    "action": {"action": "joint_deltas", "confidence": 1.0,
                               "deltas": {j: target[j] - current[j] for j in JOINTS}}}
        with mock.patch.dict(sys.modules, modules):
            prepared = prepare_chunk(planner, envelope)
            if recheck:
                planner.before_execute(prepared.execution_context)
        return prepared, validity

    def test_adapter_checks_dense_path_and_freezes_one_trajectory(self):
        prepared, validity = self.prepare()
        self.assertEqual(prepared.http_status, 200)
        points = prepared.execution_context.trajectory.joint_trajectory.points
        self.assertEqual(validity.call_async.call_count, len(points))
        self.assertGreater(len(points), 2)
        self.assertEqual(prepared.response["chunk"]["actions_per_chunk"], 2)
        changed, _ = self.prepare(middle_delta=0.0015)
        self.assertNotEqual(prepared.response["action_sha256"], changed.response["action_sha256"])

    def test_collision_prevents_returning_an_executable_context(self):
        with self.assertRaisesRegex(PlanInputRejected, "collision/bounds"):
            self.prepare(collision=True)

    def test_collision_samples_are_rechecked_before_execution(self):
        prepared, validity = self.prepare(recheck=True)
        self.assertEqual(validity.call_async.call_count, 2 * len(prepared.execution_context.trajectory.joint_trajectory.points))


if __name__ == "__main__":
    unittest.main()
