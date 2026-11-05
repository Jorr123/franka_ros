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
        message["action_chunk"]["joint_positions_rad"][0][0] += 0.31
        with self.assertRaisesRegex(ProposalError, "chunk waypoint"):
            evaluate_proposal(message, core.snapshot()[1], Limits(), message["created_at_unix_ms"], mode=PLAN_MODE)

    def test_larger_chunk_waypoint_inside_new_limits_is_accepted(self):
        message, core = self.proposal()
        message["action_chunk"]["joint_positions_rad"][0][6] += 0.21
        metrics = evaluate_proposal(message, core.snapshot()[1], Limits(), message["created_at_unix_ms"], mode=PLAN_MODE)
        self.assertAlmostEqual(metrics["max_joint_delta_rad"], 0.21)

    def test_interior_total_is_checked_even_when_final_target_is_small(self):
        for delta, accepted in ((0.249, True), (0.251, False)):
            message, core = self.proposal()
            start = message["basis_state"]["joint_positions_rad"]
            message["action_chunk"]["joint_positions_rad"][0] = [
                q + (delta if i < 6 else 0) for i, q in enumerate(start)
            ]
            if accepted:
                metrics = evaluate_proposal(message, core.snapshot()[1], Limits(),
                                            message["created_at_unix_ms"], mode=PLAN_MODE)
                self.assertAlmostEqual(metrics["total_joint_delta_rad"], 6 * delta)
            else:
                with self.assertRaisesRegex(ProposalError, "chunk waypoint"):
                    evaluate_proposal(message, core.snapshot()[1], Limits(),
                                      message["created_at_unix_ms"], mode=PLAN_MODE)

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

    def prepare(self, *, collision=False, middle_delta=0.001, recheck=False, timing="uniform", fingers=False):
        message, core = self.proposal()
        message["action_chunk"]["joint_positions_rad"][0][0] = message["basis_state"]["joint_positions_rad"][0] + middle_delta
        current = dict(zip(JOINTS, message["basis_state"]["joint_positions_rad"]))
        target = dict(zip(JOINTS, message["target_joint_positions_rad"]))
        planner = object.__new__(fixtures.CosmosMoveItPlanner)
        planner.args = parse_args(["--chunk-timing", timing])
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
            if fingers:
                from robot_bridge.chunk_planning import check_collisions
                check_collisions(planner, [list(current.values())] * 2, message, gripper_widths=[0.0798, 0.035])
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

    def test_adaptive_adapter_retains_collision_checks_and_opt_in(self):
        self.assertEqual(parse_args([]).chunk_timing, "uniform")
        prepared, validity = self.prepare(timing="adaptive", recheck=True)
        self.assertEqual(prepared.response["chunk"]["timing_mode"], "adaptive")
        points = prepared.execution_context.trajectory.joint_trajectory.points
        self.assertEqual(validity.call_async.call_count, 2 * len(points))

    def test_gripper_sweep_checks_both_fingers_and_whole_robot(self):
        _, validity = self.prepare(fingers=True)
        request = validity.call_async.call_args.args[0]
        self.assertEqual(request.group_name, "")
        self.assertEqual(request.robot_state.joint_state.name[-2:], ["fr3_finger_joint1", "fr3_finger_joint2"])
        self.assertEqual(request.robot_state.joint_state.position[-2:], [0.0175, 0.0175])

    def test_adaptive_gripper_sweep_preserves_both_features(self):
        prepared, validity = self.prepare(timing="adaptive", recheck=True, fingers=True)
        self.assertEqual(prepared.response["chunk"]["timing_mode"], "adaptive")
        points = prepared.execution_context.trajectory.joint_trajectory.points
        self.assertEqual(validity.call_async.call_count, 2 * len(points) + 2)
        self.assertEqual(validity.call_async.call_args.args[0].group_name, "")


class AdaptiveTimingTest(unittest.TestCase):
    def check_continuous_limits(self, start, targets, velocity, acceleration):
        """Independently reconstruct each full segment and check its extrema."""
        import numpy as np
        from numpy.polynomial import Polynomial

        samples, meta = timed_chunk(start, targets, 1 / 15, velocity, acceleration,
                                    timing_mode="adaptive")
        indices = [0] + meta["waypoint_sample_indices"]
        expected = [start] + targets
        self.assertEqual(meta["timing_mode"], "adaptive")
        self.assertEqual(len(meta["segment_durations_seconds"]), len(targets))
        self.assertAlmostEqual(sum(meta["segment_durations_seconds"]), samples[-1][0])
        for idx, target in zip(indices, expected):
            np.testing.assert_allclose(samples[idx][1], target, atol=1e-12, rtol=0)
            np.testing.assert_allclose(samples[idx][3], 0, atol=1e-10, rtol=0)
        for idx in [0, len(samples) - 1]:
            np.testing.assert_allclose(samples[idx][2], 0, atol=1e-12, rtol=0)
        for previous, current in zip(samples, samples[1:]):
            self.assertGreater(current[0], previous[0])
            self.assertLessEqual(current[0] - previous[0], 0.05 + 1e-12)
            self.assertLessEqual(max(abs(b - a) for a, b in zip(previous[1], current[1])), 0.001 + 1e-12)
        # Quintic coefficients from boundary position, velocity, acceleration;
        # roots of the derivative locate extrema between sampled points.
        matrix = np.array([[1, 1, 1], [3, 4, 5], [6, 12, 20]], dtype=float)
        for left, right in zip(indices, indices[1:]):
            s, e = samples[left], samples[right]
            h = e[0] - s[0]
            for j in range(7):
                c = [s[1][j], h * s[2][j], h * h * s[3][j] / 2]
                tail = np.linalg.solve(matrix, [e[1][j] - sum(c), h * e[2][j] - c[1] - 2*c[2], h*h*e[3][j] - 2*c[2]])
                poly = Polynomial(c + tail.tolist())
                for order, limit in [(0, None), (1, velocity[j]), (2, acceleration[j])]:
                    derivative = poly.deriv(order)
                    roots = derivative.deriv().roots()
                    times = [0., 1.] + [float(z.real) for z in roots if abs(z.imag) < 1e-8 and 0 < z.real < 1]
                    values = derivative(np.array(times)) / h**order
                    if order == 0:
                        self.assertGreaterEqual(min(values), min(s[1][j], e[1][j]) - 1e-10)
                        self.assertLessEqual(max(values), max(s[1][j], e[1][j]) + 1e-10)
                    else:
                        self.assertLessEqual(max(abs(values)), limit * (1 + 1e-8))
        return samples, meta

    def test_preserves_flow_without_stretching_every_segment_for_the_worst(self):
        targets = [[x] * 7 for x in [0.001, 0.002, 0.003, 0.03]]
        samples, meta = self.check_continuous_limits([0.] * 7, targets, [0.04]*7, [0.1]*7)
        self.assertGreater(samples[meta["waypoint_sample_indices"][0]][2][0], 0)
        self.assertGreater(meta["segment_durations_seconds"][-1], meta["segment_durations_seconds"][0])
        old, _ = timed_chunk([0.] * 7, targets, 1/15, [0.04]*7, [0.1]*7)
        self.assertLess(samples[-1][0], old[-1][0] * 0.6)

    def test_reversals_pauses_and_different_joint_limits(self):
        targets = [[x, -x, 0, x/2, 0, 0, 0] for x in [0.01, 0.01, -0.015, 0.002]]
        self.check_continuous_limits([0.]*7, targets, [0.03, 0.04, 0.02, 0.06, 0.02, 0.03, 0.04], [0.1]*7)

    def test_varied_paths_satisfy_unsampled_polynomial_extrema(self):
        import random
        rng = random.Random(19)
        for _ in range(12):
            start = [rng.uniform(-1, 1) for _ in range(7)]
            targets = [[x + rng.uniform(-0.02, 0.02) for x in start] for _ in range(6)]
            v = [rng.uniform(0.02, 0.06) for _ in range(7)]
            a = [rng.uniform(0.04, 0.15) for _ in range(7)]
            self.check_continuous_limits(start, targets, v, a)

    def test_invalid_mode_and_oversized_paths_fail(self):
        with self.assertRaisesRegex(ValueError, "timing mode"):
            timed_chunk([0.]*7, [[0.01]*7]*2, 1/15, [.04]*7, [.1]*7, timing_mode="unknown")
        with self.assertRaisesRegex(ValueError, "2048"):
            timed_chunk([0.]*7, [[(-1)**i * .29]*7 for i in range(32)], 1/15, [.04]*7, [.1]*7, timing_mode="adaptive")


if __name__ == "__main__":
    unittest.main()
