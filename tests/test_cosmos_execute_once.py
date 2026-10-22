"""Automatic single-step execution against fake MoveIt; never robot hardware."""

from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import tempfile
import threading
import unittest
from unittest import mock

from tests import test_cosmos_action_planning as fixtures
from robot_bridge.client import submit_execution, submit_plan
from robot_bridge.planning import ExecuteOnceServer, PlanServer
from robot_bridge.protocol import EXECUTE_MODE, EXECUTE_PATH, PLAN_PATH
from fr3_cosmos_action_server import parse_args, validate_config


class ExecuteOnceTest(unittest.TestCase):
    snapshot = fixtures.CoreTest.snapshot
    proposal = fixtures.CoreTest.proposal

    def setUp(self):
        fixtures.CoreTest.setUp(self)
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.server = ExecuteOnceServer(
            ("127.0.0.1", 0), core=self.core, allowed_clients={"127.0.0.1"},
            audit_log=Path(self.directory.name) / "events.jsonl",
        )
        self.thread = threading.Thread(target=self.server.serve_forever, kwargs={"poll_interval": 0.01}, daemon=True)
        self.thread.start()
        self.addCleanup(self.close)
        self.base = f"http://127.0.0.1:{self.server.server_port}"

    def close(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)

    def execute(self, message=None):
        message = message or self.proposal()
        message.update(mode=EXECUTE_MODE, execution_allowed=True)
        return submit_execution(message, url=self.base + EXECUTE_PATH)

    def test_executes_and_verifies_once_then_rejects_another_request(self):
        receipt = self.execute()
        self.assertEqual(receipt["decision"], "VERIFIED_SUCCESS")
        self.assertTrue(receipt["robot_execution"])
        self.assertTrue(receipt["execution"]["verification"]["passed"])
        self.assertFalse(self.server.auto_execution_armed)
        receipt = self.execute()
        self.assertEqual(receipt["code"], "AUTO_EXECUTION_DISARMED")
        self.assertEqual(self.backend.executions, 1)
        self.assertEqual(self.backend.calls, 1)

    def test_plan_once_never_consumes_auto_permission_or_executes(self):
        receipt = submit_plan(self.proposal(), url=self.base + PLAN_PATH)
        self.assertEqual(receipt["decision"], "PLAN_READY")
        self.assertFalse(receipt["plan"]["local_approval_available"])
        self.assertTrue(self.server.auto_execution_armed)
        self.assertEqual(self.backend.executions, 0)
        self.assertEqual(self.execute()["decision"], "VERIFIED_SUCCESS")

    def test_concurrent_requests_can_only_execute_one_step(self):
        messages = [self.proposal(), self.proposal()]
        with ThreadPoolExecutor(max_workers=2) as pool:
            receipts = list(pool.map(self.execute, messages))
        self.assertEqual(sum(r["decision"] == "VERIFIED_SUCCESS" for r in receipts), 1)
        self.assertEqual(sum(r["decision"] == "REJECTED" for r in receipts), 1)
        self.assertEqual(self.backend.executions, 1)

    def test_verification_failure_disarms_and_is_not_retried(self):
        self.backend.failure = True
        receipt = self.execute()
        self.assertEqual(receipt["decision"], "EXECUTION_FAILED")
        self.assertEqual(receipt["code"], "VERIFICATION_FAILED")
        self.assertIsNone(receipt["robot_execution"])
        self.assertFalse(self.server.auto_execution_armed)
        self.assertEqual(self.execute()["code"], "AUTO_EXECUTION_DISARMED")
        self.assertEqual(self.backend.executions, 1)

    def test_report_only_completion_is_a_warning_and_still_disarms(self):
        result = {"status": "EXECUTED_WITH_WARNINGS", "code": "EXECUTED_WITH_WARNINGS",
                  "verification_mode": "report-only", "moveit_execution_result": "SUCCESS",
                  "verification": {"status": "FAILED", "measured_joint_positions": {"fr3_joint7": 0.01}}}
        with mock.patch.object(self.backend, "execute", return_value=result):
            receipt = self.execute()
        self.assertEqual(receipt["decision"], "EXECUTED_WITH_WARNINGS")
        self.assertTrue(receipt["robot_execution"])
        self.assertEqual(receipt["execution"]["verification"]["status"], "FAILED")
        self.assertEqual(self.execute()["code"], "AUTO_EXECUTION_DISARMED")

    def test_failed_verification_details_reach_the_receipt(self):
        failure = fixtures.ExecutionRejected("VERIFICATION_FAILED", "Test accuracy failure",
                                            details={"verification": {"status": "FAILED", "target_error": 0.003}})
        with mock.patch.object(self.backend, "execute", side_effect=failure):
            receipt = self.execute()
        self.assertEqual(receipt["execution"]["verification"]["target_error"], 0.003)

    def test_unknown_execution_outcome_is_not_reported_as_no_movement(self):
        with mock.patch.object(self.core, "handle_local", side_effect=RuntimeError("test lost result")):
            receipt = self.execute()
        self.assertEqual(receipt["decision"], "EXECUTION_FAILED")
        self.assertEqual(receipt["code"], "EXECUTION_OUTCOME_UNKNOWN")
        self.assertIsNone(receipt["robot_execution"])
        self.assertFalse(self.server.auto_execution_armed)

    def test_invalid_target_does_not_plan_or_execute(self):
        self.server.auto_executions_remaining = 5
        receipt = self.execute(self.proposal(0.31))
        self.assertEqual(receipt["code"], "DELTA_LIMIT_EXCEEDED")
        self.assertEqual(self.backend.executions, 0)
        self.assertEqual(self.backend.calls, 0)
        self.assertEqual(self.server.auto_executions_remaining, 5)

    def test_failed_audit_before_execution_prevents_motion_and_disarms(self):
        self.server.auto_executions_remaining = 5
        with mock.patch.object(self.server, "record", side_effect=OSError("test disk failure")):
            receipt = self.execute()
        self.assertEqual(receipt["decision"], "REJECTED")
        self.assertEqual(receipt["code"], "AUDIT_UNAVAILABLE")
        self.assertEqual(self.backend.executions, 0)
        self.assertFalse(self.server.auto_execution_armed)
        self.assertEqual(self.server.auto_executions_remaining, 0)

    def test_failed_receipt_audit_after_execution_preserves_motion_outcome(self):
        record = self.server.record

        def fail_final_receipt(proposal, receipt):
            if receipt.get("execution_attempted"):
                raise OSError("test final audit failure")
            return record(proposal, receipt)

        with mock.patch.object(self.server, "record", side_effect=fail_final_receipt):
            receipt = self.execute()
        self.assertEqual(receipt["decision"], "EXECUTION_FAILED")
        self.assertEqual(receipt["code"], "AUDIT_UNAVAILABLE")
        self.assertTrue(receipt["robot_execution"])
        self.assertEqual(self.backend.executions, 1)

    def test_automatic_execution_is_disabled_by_default(self):
        args = parse_args([])
        self.assertFalse(args.allow_one_auto_execution)
        with self.assertRaises(ValueError):
            validate_config(parse_args(["--allow-one-auto-execution", "--enable-local-execution"]))

    def test_multiple_execution_budget_is_consumed_and_cannot_be_exceeded(self):
        self.server.auto_executions_remaining = 3
        for remaining in (2, 1, 0):
            receipt = self.execute()
            self.assertEqual(receipt["decision"], "VERIFIED_SUCCESS")
            self.assertEqual(receipt["auto_executions_remaining"], remaining)
            self.assertEqual(receipt["auto_execution_armed"], remaining > 0)
        self.assertEqual(self.execute()["code"], "AUTO_EXECUTION_DISARMED")
        self.assertEqual(self.backend.executions, 3)

    def test_failure_discards_the_remaining_budget(self):
        self.server.auto_executions_remaining = 5
        self.backend.failure = True
        self.assertEqual(self.execute()["decision"], "EXECUTION_FAILED")
        self.assertEqual(self.server.auto_executions_remaining, 0)
        self.assertEqual(self.execute()["code"], "AUTO_EXECUTION_DISARMED")
        self.assertEqual(self.backend.executions, 1)

    def test_warning_allows_next_request_within_budget(self):
        self.server.auto_executions_remaining = 2
        result = {"status": "EXECUTED_WITH_WARNINGS", "code": "EXECUTED_WITH_WARNINGS",
                  "verification_mode": "report-only", "moveit_execution_result": "SUCCESS",
                  "verification": {"status": "FAILED"}}
        with mock.patch.object(self.backend, "execute", return_value=result):
            receipt = self.execute()
        self.assertTrue(receipt["auto_execution_armed"])
        self.assertEqual(self.execute()["decision"], "VERIFIED_SUCCESS")
        self.assertFalse(self.server.auto_execution_armed)

    def test_receipt_audit_failure_discards_unused_budget(self):
        self.server.auto_executions_remaining = 5
        record = self.server.record

        def fail_final(proposal, receipt):
            if receipt.get("execution_attempted"):
                raise OSError("test audit failure")
            return record(proposal, receipt)

        with mock.patch.object(self.server, "record", side_effect=fail_final):
            receipt = self.execute()
        self.assertEqual(receipt["code"], "AUDIT_UNAVAILABLE")
        self.assertFalse(receipt["auto_execution_armed"])
        self.assertEqual(receipt["auto_executions_remaining"], 0)
        self.assertEqual(self.execute()["code"], "AUTO_EXECUTION_DISARMED")

    def test_multi_attempt_flag_requires_positive_budget_and_separate_mode(self):
        args = parse_args(["--allow-auto-executions", "5"])
        validate_config(args)
        self.assertEqual(args.allow_auto_executions, 5)
        for arguments in (["--allow-auto-executions", "0"],
                          ["--allow-auto-executions", "-1"],
                          ["--allow-auto-executions", "5", "--enable-local-execution"]):
            with self.subTest(arguments=arguments), self.assertRaises(ValueError):
                validate_config(parse_args(arguments))
        server = PlanServer(("127.0.0.1", 0), core=self.core, allowed_clients={"127.0.0.1"},
                            audit_log=Path(self.directory.name) / "plan-only.jsonl")
        try:
            self.assertIsNone(server.request_mode(EXECUTE_PATH))
        finally:
            server.server_close()


if __name__ == "__main__":
    unittest.main()
