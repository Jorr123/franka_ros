"""Policy gripper events and arm/gripper sequencing with simulated hardware."""

from pathlib import Path
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest import mock

from tests import test_cosmos_action_planning as fixtures
from robot_bridge.gripper import GripperSettings, select_gripper_prefix, verify_gripper_width, FrankaPolicyGripper
from robot_bridge.gripper_integration import check_gripper_path
from robot_bridge.protocol import PLAN_MODE, EXECUTE_MODE, EXECUTE_PATH, ProposalError, validate_shape
from robot_bridge.client import submit_execution, submit_plan
from robot_bridge.planning import ExecuteOnceServer, CosmosMoveItPlanner
from fr3_ai_supervisor import ExecutionRejected
from fr3_cosmos_action_server import parse_args, validate_config


class PolicyTest(unittest.TestCase):
    def test_first_transition_ends_the_prefix_and_dead_band_holds(self):
        self.assertEqual(select_gripper_prefix([0.0, 0.5, 0.8, 0.1], 0.0798), (3, "grasp"))
        self.assertEqual(select_gripper_prefix([0.9, 0.5, 0.1, 0.8], 0.035, grasp_confirmed=True), (3, "open"))
        self.assertEqual(select_gripper_prefix([-0.02, 0.2, 0.5], 0.0798), (3, "hold"))
        self.assertEqual(select_gripper_prefix([0.8, 0.9], 0.035, grasp_confirmed=True), (2, "hold"))
        self.assertEqual(select_gripper_prefix([0.8, 0.9], 0.04), (1, "grasp"))

    def test_invalid_signals_and_unconfigured_gripper_are_rejected(self):
        with self.assertRaises(ValueError):
            select_gripper_prefix([float("nan")], 0.08)
        with self.assertRaisesRegex(ValueError, "explicit"):
            validate_config(parse_args(["--enable-gripper"]))
        for width, force in ((0.0, 30), (0.081, 30), (0.035, 0), (0.035, 71)):
            with self.subTest(width=width, force=force), self.assertRaises(ValueError):
                GripperSettings(width, force)
        validate_config(parse_args(["--enable-gripper", "--gripper-grasp-width", "0.035", "--gripper-force", "30"]))

    def test_empty_closure_and_wrong_opening_do_not_pass(self):
        config = GripperSettings(0.035, 30)
        verify_gripper_width("grasp", 0.035, config)
        verify_gripper_width("open", 0.0798, config)
        for operation, width in (("grasp", 0.0), ("grasp", 0.05), ("open", 0.04)):
            with self.subTest(operation=operation, width=width), self.assertRaises(RuntimeError):
                verify_gripper_width(operation, width, config)

    def test_ros_wrapper_verifies_another_sample_after_action_completion(self):
        gripper = object.__new__(FrankaPolicyGripper)
        gripper.config = GripperSettings(0.035, 30)
        gripper.client = mock.Mock()
        gripper.client.grasp.return_value = SimpleNamespace(sequence=10, width=0.035)
        gripper.client.wait_for_snapshot.return_value = SimpleNamespace(sequence=11, width=0.035)
        result = gripper.execute("grasp")
        self.assertEqual(result["status"], "PASSED")
        gripper.client.wait_for_snapshot.assert_called_once_with(10.0, after_sequence=10)

    def test_unavailable_gripper_prevents_sending_arm_trajectory(self):
        planner = object.__new__(CosmosMoveItPlanner)
        planner.args = parse_args([])
        planner.args.enable_local_execution = True
        planner.JOINTS = list(fixtures.JOINTS)
        planner.gripper = mock.Mock()
        planner.gripper.preflight.side_effect = RuntimeError("Gripper unavailable")
        positions = dict.fromkeys(planner.JOINTS, 0.1)
        planner.node = mock.Mock(joint_state_sequence=1)
        planner.node.wait_for_joint_state_snapshot.return_value = SimpleNamespace(positions=positions, velocities=dict.fromkeys(positions, 0.0))
        context = fixtures.ExecutionContext("fake", positions, positions,
                                            {"cosmos_proposal": {"gripper_command": {"operation": "grasp"}}})
        with self.assertRaisesRegex(ExecutionRejected, "unavailable"):
            planner.execute(context)
        planner.node.execute.assert_not_called()


class GripperServerTest(unittest.TestCase):
    snapshot = fixtures.CoreTest.snapshot

    def setUp(self):
        fixtures.CoreTest.setUp(self)
        self.live["gripper_width_m"] = 0.0798
        self.config = GripperSettings(0.035, 30)
        self.backend.gripper = mock.Mock(config=self.config)
        self.backend.gripper.confirmed_width_m = None
        self.backend.gripper.grasp_confirmed_at.side_effect = lambda width: self.backend.gripper.confirmed_width_m is not None and abs(width - self.backend.gripper.confirmed_width_m) <= 0.001
        self.backend.args = parse_args([])
        self.backend.node = mock.Mock(joint_state_sequence=1)
        self.backend.snapshot = self.snapshot
        self.backend.limits = self.core.limits
        self.events = []
        old_plan, old_execute = self.backend.plan, self.backend.execute

        def plan(envelope):
            prepared = old_plan(envelope)
            command = envelope["cosmos_proposal"].get("gripper_command")
            if command:
                prepared.response["gripper"] = self.config.plan(command["operation"])
            return prepared

        def arm(context):
            self.events.append("arm")
            result = old_execute(context)
            self.live["joint_positions_rad"] = list(context.target_positions.values())
            self.backend.node.wait_for_joint_state_snapshot.return_value = SimpleNamespace(
                positions=context.target_positions, velocities=dict.fromkeys(context.target_positions, 0.0))
            return result

        def grip(operation):
            self.events.append(operation)
            width = 0.035 if operation == "grasp" else 0.0798
            self.live["gripper_width_m"] = width
            self.backend.gripper.confirmed_width_m = width if operation == "grasp" else None
            return {"status": "PASSED", "operation": operation, "command_sent": True,
                    "target_width_m": self.config.plan(operation)["target_width_m"], "measured_width_m": width,
                    "force_n": 30 if operation == "grasp" else None}

        self.backend.plan, self.backend.execute = plan, arm
        self.backend.gripper.execute.side_effect = grip
        patcher = mock.patch("robot_bridge.gripper_integration.check_gripper_path")
        self.path_check = patcher.start()
        self.addCleanup(patcher.stop)
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.server = ExecuteOnceServer(("127.0.0.1", 0), core=self.core, allowed_clients={"127.0.0.1"},
                                        audit_log=Path(self.directory.name) / "audit.jsonl", max_auto_executions=5)
        self.thread = threading.Thread(target=self.server.serve_forever, kwargs={"poll_interval": 0.01}, daemon=True)
        self.thread.start()
        self.addCleanup(self.close)
        self.url = f"http://127.0.0.1:{self.server.server_port}" + EXECUTE_PATH

    def close(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)

    def message(self, operation="grasp", signal=0.8):
        message = fixtures.CoreTest.proposal(self)
        message.update(mode=EXECUTE_MODE, execution_allowed=True, gripper_mode="policy_binary",
                       gripper_command={"operation": operation, "policy_closed_fractions": [signal], "grasp_confirmed": False})
        return message

    def test_grasp_then_open_have_sequential_arm_and_gripper_results(self):
        for operation, signal in (("grasp", 0.8), ("open", 0.0)):
            receipt = submit_execution(self.message(operation, signal), url=self.url)
            self.assertEqual(receipt["decision"], "VERIFIED_SUCCESS")
            self.assertEqual(receipt["execution"]["gripper"]["status"], "PASSED")
        self.assertEqual(self.events, ["arm", "grasp", "arm", "open"])

    def test_disabled_gripper_is_rejected_before_arm_planning(self):
        self.backend.gripper = None
        receipt = submit_execution(self.message(), url=self.url)
        self.assertEqual(receipt["code"], "GRIPPER_DISABLED")
        self.assertEqual(self.events, [])
        self.assertEqual(self.backend.calls, 0)

    def test_plan_only_never_sends_arm_or_gripper_commands(self):
        message = self.message()
        message.update(mode=PLAN_MODE, execution_allowed=False)
        receipt = submit_plan(message, url=self.url.replace(EXECUTE_PATH, fixtures.PLAN_PATH))
        self.assertEqual(receipt["decision"], "PLAN_READY")
        self.assertEqual(receipt["plan"]["gripper"]["operation"], "grasp")
        self.assertEqual(self.events, [])
        self.backend.gripper.execute.assert_not_called()

    def test_arm_fault_prevents_grasp_and_disarms(self):
        self.backend.failure = True
        receipt = submit_execution(self.message(), url=self.url)
        self.assertEqual(receipt["decision"], "EXECUTION_FAILED")
        self.backend.gripper.execute.assert_not_called()
        self.assertFalse(self.server.auto_execution_armed)

    def test_gripper_timeout_preserves_arm_result_and_disarms(self):
        self.backend.gripper.execute.side_effect = RuntimeError("gripper timeout")
        receipt = submit_execution(self.message(), url=self.url)
        self.assertEqual(receipt["code"], "GRIPPER_FAILED")
        self.assertIn("arm_result", receipt["execution"])
        self.assertTrue(receipt["execution"]["gripper"]["command_may_have_started"])
        self.assertEqual(self.server.auto_executions_remaining, 0)

    def test_collision_after_arm_blocks_gripper_command(self):
        self.path_check.side_effect = RuntimeError("finger collision")
        receipt = submit_execution(self.message(), url=self.url)
        self.assertEqual(receipt["code"], "GRIPPER_FAILED")
        self.backend.gripper.execute.assert_not_called()
        self.assertFalse(receipt["execution"]["gripper"]["command_may_have_started"])

    def test_tampered_policy_command_is_rejected(self):
        message = self.message("open", 0.8)
        with self.assertRaises(ProposalError):
            validate_shape(message, expected_mode=EXECUTE_MODE)
        message = self.message()
        message["gripper_command"]["force_n"] = 70
        with self.assertRaises(ProposalError):
            validate_shape(message, expected_mode=EXECUTE_MODE)

    def test_client_cannot_forge_a_verified_grasp(self):
        self.live["gripper_width_m"] = 0.035
        message = self.message("hold", 0.8)
        message["gripper_command"]["grasp_confirmed"] = True
        receipt = submit_execution(message, url=self.url)
        self.assertEqual(receipt["code"], "GRASP_STATE_CHANGED")
        self.assertEqual(self.backend.calls, 0)

    def test_planned_finger_sweep_includes_the_whole_grasp_interval(self):
        proposal = self.message()
        positions = dict(zip(fixtures.JOINTS, proposal["target_joint_positions_rad"]))
        with mock.patch("robot_bridge.gripper_integration.check_collisions") as check:
            check_gripper_path(self.backend, proposal, positions)
        widths = check.call_args.kwargs["gripper_widths"]
        self.assertAlmostEqual(widths[0], 0.0798)
        self.assertAlmostEqual(widths[-1], 0.03)
        self.assertLessEqual(max(abs(b - a) for a, b in zip(widths, widths[1:])), 0.001 + 1e-12)


if __name__ == "__main__":
    unittest.main()
