#!/usr/bin/env python3

import hashlib
import json
import sys
import threading
import time
from types import SimpleNamespace
import unittest
from pathlib import Path
import urllib.error
import urllib.request


ROOT = Path(__file__).resolve().parents[1]
WINDOWS_BRIDGE = ROOT / "windows_vlm_bridge"
SCRIPTS = ROOT / "scripts"
sys.path.insert(0, str(WINDOWS_BRIDGE))
sys.path.insert(0, str(SCRIPTS))

from fr3_action_protocol import (  # noqa: E402
    ProtocolError,
    ReplayCache,
    action_sha256,
    canonical_json_bytes,
    create_action_envelope,
    create_session_arm_envelope,
    create_session_step_envelope,
    create_session_stop_envelope,
    make_signature,
    validate_action_envelope,
    validate_session_arm_envelope,
    validate_session_step_envelope,
    validate_session_stop_envelope,
    verify_signature,
)
from fr3_ai_supervisor import (  # noqa: E402
    ExecutionContext,
    ExecutionRejected,
    PlanInputRejected,
    PreparedPlan,
    READY_STATUS,
    RosMoveItPlanner,
    RequestFailure,
    SupervisorCore,
    SupervisorHttpServer,
    validate_visual_action_contract,
)


SECRET = b"0123456789abcdef0123456789abcdef"


def valid_action(delta: float = 0.03, side: str = "right") -> dict:
    return {
        "action": "joint_deltas",
        "deltas": {"fr3_joint3": delta},
        "confidence": 0.95,
        "observation": {"red_marker_side": side},
    }


def valid_envelope(
    now_ms: int | None = None,
    request_id: str = "12345678-1234-4234-8234-123456789abc",
) -> dict:
    return create_action_envelope(
        valid_action(),
        frame_source="windows-static-file",
        frame_id="frame-1",
        frame_sha256=hashlib.sha256(b"image").hexdigest(),
        now_unix_ms=now_ms,
        request_id=request_id,
    )


class FakePlanner:
    def __init__(self, execution_error: ExecutionRejected | None = None) -> None:
        self.calls = 0
        self.execute_calls = 0
        self.execution_error = execution_error

    def plan(self, envelope):
        self.calls += 1
        current = {f"fr3_joint{index}": 0.0 for index in range(1, 8)}
        target = dict(current)
        target["fr3_joint3"] = 0.03
        response = {
                "schema_version": 1,
                "request_id": envelope["request_id"],
                "status": "PLAN_READY",
                "plan_only": True,
                "execution_available": False,
                "remote_execution_available": False,
                "action_sha256": action_sha256(envelope["action"]),
                "validated_deltas": {"fr3_joint3": 0.03},
                "current_joint_positions": current,
                "target_joint_positions": target,
                "planning": {
                    "result": "SUCCESS",
                    "point_count": 6,
                    "duration_seconds": 0.5,
                    "joints": list(current),
                },
            }
        return PreparedPlan(
            http_status=200,
            response=response,
            execution_context=ExecutionContext(
                trajectory="frozen-trajectory",
                start_positions=current,
                target_positions=target,
            ),
        )

    def execute(self, context):
        self.execute_calls += 1
        self.last_context = context
        if self.execution_error is not None:
            raise self.execution_error
        return {
            "status": "VERIFIED_SUCCESS",
            "code": "VERIFIED_SUCCESS",
            "moveit_execution_result": "SUCCESS",
        }


def make_core(
    planner: FakePlanner | None = None,
    *,
    execution_enabled: bool = False,
    plan_ttl_seconds: float = 30.0,
) -> SupervisorCore:
    return SupervisorCore(
        planner or FakePlanner(),
        SECRET,
        max_request_age_ms=10_000,
        max_future_skew_ms=2_000,
        replay_retention_seconds=300.0,
        execution_enabled=execution_enabled,
        plan_ttl_seconds=plan_ttl_seconds,
    )


def submit_plan(core: SupervisorCore, envelope: dict | None = None):
    selected = envelope or valid_envelope()
    body = canonical_json_bytes(selected)
    return core.handle_plan(body, make_signature(body, SECRET))


class ProtocolTest(unittest.TestCase):
    def test_valid_signed_envelope(self) -> None:
        now_ms = int(time.time() * 1000)
        envelope = valid_envelope(now_ms)
        body = canonical_json_bytes(envelope)
        verify_signature(body, make_signature(body, SECRET), SECRET)
        validated = validate_action_envelope(
            envelope,
            now_unix_ms=now_ms,
            max_age_ms=10_000,
            max_future_skew_ms=2_000,
        )
        self.assertEqual(validated["request_id"], envelope["request_id"])

    def test_tampered_body_fails_signature(self) -> None:
        body = canonical_json_bytes(valid_envelope())
        signature = make_signature(body, SECRET)
        tampered = body.replace(b"0.03", b"0.02")
        with self.assertRaisesRegex(ProtocolError, "signature"):
            verify_signature(tampered, signature, SECRET)

    def test_stale_envelope_is_rejected(self) -> None:
        now_ms = int(time.time() * 1000)
        with self.assertRaisesRegex(ProtocolError, "maximum age"):
            validate_action_envelope(
                valid_envelope(now_ms - 10_001),
                now_unix_ms=now_ms,
                max_age_ms=10_000,
                max_future_skew_ms=2_000,
            )

    def test_future_envelope_is_rejected(self) -> None:
        now_ms = int(time.time() * 1000)
        with self.assertRaisesRegex(ProtocolError, "future"):
            validate_action_envelope(
                valid_envelope(now_ms + 2_001),
                now_unix_ms=now_ms,
                max_age_ms=10_000,
                max_future_skew_ms=2_000,
            )

    def test_boolean_schema_version_is_rejected(self) -> None:
        now_ms = int(time.time() * 1000)
        envelope = valid_envelope(now_ms)
        envelope["schema_version"] = True
        with self.assertRaisesRegex(ProtocolError, "schema_version"):
            validate_action_envelope(
                envelope,
                now_unix_ms=now_ms,
                max_age_ms=10_000,
                max_future_skew_ms=2_000,
            )

    def test_replay_cache_rejects_duplicate_id(self) -> None:
        cache = ReplayCache(retention_seconds=10.0)
        self.assertTrue(cache.record_if_new("request", now_monotonic=1.0))
        self.assertFalse(cache.record_if_new("request", now_monotonic=2.0))

    def test_valid_session_step_envelope(self) -> None:
        now_ms = int(time.time() * 1000)
        envelope = create_session_step_envelope(
            valid_action(),
            session_id="aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa",
            sequence=1,
            frame_source="windows-static-execution-session",
            frame_id="session-step-1",
            frame_sha256=hashlib.sha256(b"image").hexdigest(),
            now_unix_ms=now_ms,
            request_id="bbbbbbbb-bbbb-4bbb-8bbb-bbbbbbbbbbbb",
        )
        validated = validate_session_step_envelope(
            envelope,
            now_unix_ms=now_ms,
            max_age_ms=10_000,
            max_future_skew_ms=2_000,
        )
        self.assertEqual(validated["sequence"], 1)
        self.assertEqual(validated["session_id"], envelope["session_id"])

    def test_valid_session_arm_envelope(self) -> None:
        now_ms = int(time.time() * 1000)
        envelope = create_session_arm_envelope(
            session_id="aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa",
            allowed_joints=["fr3_joint1", "fr3_joint3"],
            max_moving_joints=2,
            max_steps=2,
            max_duration_seconds=60.0,
            max_delta=0.01,
            max_total_delta=0.02,
            max_cumulative_delta=0.04,
            now_unix_ms=now_ms,
            request_id="bbbbbbbb-bbbb-4bbb-8bbb-bbbbbbbbbbbb",
        )
        validated = validate_session_arm_envelope(
            envelope,
            now_unix_ms=now_ms,
            max_age_ms=10_000,
            max_future_skew_ms=2_000,
        )
        self.assertEqual(validated["operation"], "arm")
        self.assertEqual(
            validated["limits"]["allowed_joints"], ["fr3_joint1", "fr3_joint3"]
        )

    def test_session_arm_rejects_invalid_limits(self) -> None:
        now_ms = int(time.time() * 1000)
        envelope = create_session_arm_envelope(
            session_id="aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa",
            allowed_joints=["fr3_joint3"],
            max_moving_joints=1,
            max_steps=True,
            max_duration_seconds=60.0,
            max_delta=0.01,
            max_total_delta=0.01,
            max_cumulative_delta=0.02,
            now_unix_ms=now_ms,
        )
        with self.assertRaisesRegex(ProtocolError, "positive integer"):
            validate_session_arm_envelope(
                envelope,
                now_unix_ms=now_ms,
                max_age_ms=10_000,
                max_future_skew_ms=2_000,
            )

    def test_session_arm_rejects_duplicate_allowed_joints(self) -> None:
        now_ms = int(time.time() * 1000)
        envelope = create_session_arm_envelope(
            session_id="aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa",
            allowed_joints=["fr3_joint1", "fr3_joint1"],
            max_moving_joints=1,
            max_steps=2,
            max_duration_seconds=60.0,
            max_delta=0.01,
            max_total_delta=0.01,
            max_cumulative_delta=0.02,
            now_unix_ms=now_ms,
        )
        with self.assertRaisesRegex(ProtocolError, "duplicates"):
            validate_session_arm_envelope(
                envelope,
                now_unix_ms=now_ms,
                max_age_ms=10_000,
                max_future_skew_ms=2_000,
            )

    def test_session_arm_rejects_noncanonical_joint_order(self) -> None:
        now_ms = int(time.time() * 1000)
        envelope = create_session_arm_envelope(
            session_id="aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa",
            allowed_joints=["fr3_joint3", "fr3_joint1"],
            max_moving_joints=2,
            max_steps=2,
            max_duration_seconds=60.0,
            max_delta=0.01,
            max_total_delta=0.02,
            max_cumulative_delta=0.04,
            now_unix_ms=now_ms,
        )
        with self.assertRaisesRegex(ProtocolError, "canonical"):
            validate_session_arm_envelope(
                envelope,
                now_unix_ms=now_ms,
                max_age_ms=10_000,
                max_future_skew_ms=2_000,
            )

    def test_session_arm_rejects_total_above_per_joint_budget(self) -> None:
        now_ms = int(time.time() * 1000)
        envelope = create_session_arm_envelope(
            session_id="aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa",
            allowed_joints=["fr3_joint1", "fr3_joint3"],
            max_moving_joints=2,
            max_steps=2,
            max_duration_seconds=60.0,
            max_delta=0.01,
            max_total_delta=0.021,
            max_cumulative_delta=0.042,
            now_unix_ms=now_ms,
        )
        with self.assertRaisesRegex(ProtocolError, "max_delta times max_moving_joints"):
            validate_session_arm_envelope(
                envelope,
                now_unix_ms=now_ms,
                max_age_ms=10_000,
                max_future_skew_ms=2_000,
            )

    def test_session_step_rejects_boolean_sequence(self) -> None:
        now_ms = int(time.time() * 1000)
        envelope = create_session_step_envelope(
            valid_action(),
            session_id="aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa",
            sequence=True,
            frame_source="windows-static-execution-session",
            frame_id="session-step-1",
            frame_sha256=hashlib.sha256(b"image").hexdigest(),
            now_unix_ms=now_ms,
        )
        with self.assertRaisesRegex(ProtocolError, "positive integer"):
            validate_session_step_envelope(
                envelope,
                now_unix_ms=now_ms,
                max_age_ms=10_000,
                max_future_skew_ms=2_000,
            )

    def test_valid_signed_session_stop_envelope(self) -> None:
        now_ms = int(time.time() * 1000)
        envelope = create_session_stop_envelope(
            "aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa",
            now_unix_ms=now_ms,
            request_id="bbbbbbbb-bbbb-4bbb-8bbb-bbbbbbbbbbbb",
        )
        body = canonical_json_bytes(envelope)
        verify_signature(body, make_signature(body, SECRET), SECRET)
        validated = validate_session_stop_envelope(
            envelope,
            now_unix_ms=now_ms,
            max_age_ms=10_000,
            max_future_skew_ms=2_000,
        )
        self.assertEqual(validated["operation"], "stop")

    def test_visual_contract_rejects_wrong_direction(self) -> None:
        with self.assertRaisesRegex(PlanInputRejected, "positive"):
            validate_visual_action_contract(valid_action(-0.03, "right"))

    def test_supervisor_core_rejects_replay(self) -> None:
        planner = FakePlanner()
        core = make_core(planner)
        body = canonical_json_bytes(valid_envelope())
        signature = make_signature(body, SECRET)
        status, response = core.handle_plan(body, signature)
        self.assertEqual(status, 200)
        self.assertEqual(response["status"], "PLAN_READY")
        self.assertEqual(planner.calls, 1)

        with self.assertRaises(RequestFailure) as context:
            core.handle_plan(body, signature)
        self.assertEqual(context.exception.status, 409)
        self.assertEqual(context.exception.code, "REPLAY_REJECTED")
        self.assertEqual(planner.calls, 1)

    def test_nonstandard_nan_json_is_rejected(self) -> None:
        planner = FakePlanner()
        core = make_core(planner)
        envelope = valid_envelope()
        body = json.dumps(envelope).replace("0.95", "NaN").encode("utf-8")
        signature = make_signature(body, SECRET)
        with self.assertRaises(RequestFailure) as context:
            core.handle_plan(body, signature)
        self.assertEqual(context.exception.code, "INVALID_JSON")

    def test_authenticated_http_plan_exchange(self) -> None:
        core = make_core()
        server = SupervisorHttpServer(("127.0.0.1", 0), core, {"127.0.0.1"})
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            base_url = f"http://127.0.0.1:{server.server_port}"
            with urllib.request.urlopen(f"{base_url}/health", timeout=2.0) as response:
                health = json.loads(response.read())
            self.assertEqual(health["mode"], "PLAN_ONLY")
            self.assertFalse(health["execution_available"])

            body = canonical_json_bytes(valid_envelope())
            request = urllib.request.Request(
                f"{base_url}/v1/actions/plan",
                data=body,
                headers={
                    "Content-Type": "application/json",
                    "X-FR3-Signature": make_signature(body, SECRET),
                },
                method="POST",
            )
            with urllib.request.urlopen(request, timeout=2.0) as response:
                plan = json.loads(response.read())
            self.assertEqual(plan["status"], "PLAN_READY")
            self.assertTrue(plan["plan_only"])
            self.assertFalse(plan["execution_available"])
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2.0)

    def test_supervised_mode_has_no_http_execution_endpoint(self) -> None:
        core = make_core(execution_enabled=True)
        server = SupervisorHttpServer(("127.0.0.1", 0), core, {"127.0.0.1"})
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            base_url = f"http://127.0.0.1:{server.server_port}"
            with urllib.request.urlopen(f"{base_url}/health", timeout=2.0) as response:
                health = json.loads(response.read())
            self.assertEqual(health["mode"], "SUPERVISED_SINGLE_STEP")
            self.assertFalse(health["remote_execution_available"])
            self.assertTrue(health["local_approval_available"])

            request = urllib.request.Request(
                f"{base_url}/v1/actions/execute",
                data=b"{}",
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            with self.assertRaises(urllib.error.HTTPError) as context:
                urllib.request.urlopen(request, timeout=2.0)
            self.assertEqual(context.exception.code, 404)
            response = json.loads(context.exception.read())
            self.assertEqual(response["code"], "NOT_FOUND")
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2.0)

    def test_local_execution_is_disabled_by_default(self) -> None:
        core = make_core()
        with self.assertRaises(RequestFailure) as context:
            core.handle_local(
                {
                    "operation": "execute",
                    "request_id": valid_envelope()["request_id"],
                    "action_sha256": "0" * 64,
                }
            )
        self.assertEqual(context.exception.code, "EXECUTION_DISABLED")

    def test_frozen_plan_can_be_inspected_and_executed_once(self) -> None:
        planner = FakePlanner()
        core = make_core(planner, execution_enabled=True)
        status, plan = submit_plan(core)
        self.assertEqual(status, 200)
        self.assertEqual(plan["approval_status"], READY_STATUS)
        self.assertTrue(plan["local_approval_available"])

        local_status, stored = core.handle_local(
            {"operation": "status", "request_id": plan["request_id"]}
        )
        self.assertEqual(local_status, 200)
        self.assertEqual(stored["status"], READY_STATUS)
        self.assertEqual(stored["action_sha256"], plan["action_sha256"])

        result_status, result = core.handle_local(
            {
                "operation": "execute",
                "request_id": plan["request_id"],
                "action_sha256": plan["action_sha256"],
            }
        )
        self.assertEqual(result_status, 200)
        self.assertEqual(result["status"], "VERIFIED_SUCCESS")
        self.assertEqual(planner.execute_calls, 1)
        self.assertEqual(planner.last_context.trajectory, "frozen-trajectory")

        with self.assertRaises(RequestFailure) as context:
            core.handle_local(
                {
                    "operation": "execute",
                    "request_id": plan["request_id"],
                    "action_sha256": plan["action_sha256"],
                }
            )
        self.assertEqual(context.exception.code, "PLAN_NOT_READY")
        self.assertEqual(planner.execute_calls, 1)

    def test_wrong_action_hash_does_not_consume_plan(self) -> None:
        planner = FakePlanner()
        core = make_core(planner, execution_enabled=True)
        _, plan = submit_plan(core)

        with self.assertRaises(RequestFailure) as context:
            core.handle_local(
                {
                    "operation": "execute",
                    "request_id": plan["request_id"],
                    "action_sha256": "0" * 64,
                }
            )
        self.assertEqual(context.exception.code, "ACTION_HASH_MISMATCH")
        self.assertEqual(planner.execute_calls, 0)
        _, stored = core.handle_local(
            {"operation": "status", "request_id": plan["request_id"]}
        )
        self.assertEqual(stored["status"], READY_STATUS)

    def test_expired_plan_cannot_execute(self) -> None:
        planner = FakePlanner()
        core = make_core(
            planner,
            execution_enabled=True,
            plan_ttl_seconds=0.001,
        )
        _, plan = submit_plan(core)
        time.sleep(0.01)

        _, stored = core.handle_local(
            {"operation": "status", "request_id": plan["request_id"]}
        )
        self.assertEqual(stored["status"], "EXPIRED")
        with self.assertRaises(RequestFailure) as context:
            core.handle_local(
                {
                    "operation": "execute",
                    "request_id": plan["request_id"],
                    "action_sha256": plan["action_sha256"],
                }
            )
        self.assertEqual(context.exception.code, "PLAN_NOT_READY")
        self.assertEqual(planner.execute_calls, 0)

    def test_new_plan_supersedes_previous_ready_plan(self) -> None:
        core = make_core(execution_enabled=True)
        first = valid_envelope(request_id="12345678-1234-4234-8234-123456789abc")
        second = valid_envelope(request_id="22345678-1234-4234-8234-123456789abc")
        _, first_plan = submit_plan(core, first)
        _, second_plan = submit_plan(core, second)

        _, first_status = core.handle_local(
            {"operation": "status", "request_id": first_plan["request_id"]}
        )
        _, second_status = core.handle_local(
            {"operation": "status", "request_id": second_plan["request_id"]}
        )
        self.assertEqual(first_status["status"], "SUPERSEDED")
        self.assertEqual(second_status["status"], READY_STATUS)

    def test_execution_fault_is_terminal_and_not_retried(self) -> None:
        planner = FakePlanner(
            ExecutionRejected("START_STATE_CHANGED", "Joint state changed after planning.")
        )
        core = make_core(planner, execution_enabled=True)
        _, plan = submit_plan(core)

        result_status, result = core.handle_local(
            {
                "operation": "execute",
                "request_id": plan["request_id"],
                "action_sha256": plan["action_sha256"],
            }
        )
        self.assertEqual(result_status, 500)
        self.assertEqual(result["status"], "FAULT")
        self.assertEqual(result["code"], "START_STATE_CHANGED")
        self.assertEqual(planner.execute_calls, 1)

        with self.assertRaises(RequestFailure) as context:
            core.handle_local(
                {
                    "operation": "execute",
                    "request_id": plan["request_id"],
                    "action_sha256": plan["action_sha256"],
                }
            )
        self.assertEqual(context.exception.code, "PLAN_NOT_READY")
        self.assertEqual(planner.execute_calls, 1)

    def test_real_planner_rejects_changed_start_state_before_moveit(self) -> None:
        joints = [f"fr3_joint{index}" for index in range(1, 8)]
        planned_start = {joint: 0.0 for joint in joints}
        measured = dict(planned_start)
        measured["fr3_joint3"] = 0.02

        class FakeStateNode:
            joint_state_sequence = 1

            def wait_for_joint_state_snapshot(self, **_kwargs):
                return SimpleNamespace(
                    positions=measured,
                    velocities={joint: 0.0 for joint in joints},
                )

            def wait_for_execute_action(self, **_kwargs):
                raise AssertionError("MoveIt must not be contacted after state drift.")

        planner = RosMoveItPlanner.__new__(RosMoveItPlanner)
        planner.args = SimpleNamespace(
            enable_local_execution=True,
            state_timeout=1.0,
            start_state_tolerance=0.01,
            max_start_velocity=0.01,
        )
        planner.node = FakeStateNode()
        planner.JOINTS = joints

        context = ExecutionContext(
            trajectory="must-not-execute",
            start_positions=planned_start,
            target_positions=planned_start,
        )
        with self.assertRaises(ExecutionRejected) as rejected:
            planner.execute(context)
        self.assertEqual(rejected.exception.code, "START_STATE_CHANGED")

    def test_real_planner_rejects_unsettled_robot_before_moveit(self) -> None:
        joints = [f"fr3_joint{index}" for index in range(1, 8)]
        positions = {joint: 0.0 for joint in joints}
        velocities = {joint: 0.0 for joint in joints}
        velocities["fr3_joint3"] = 0.02

        class FakeStateNode:
            joint_state_sequence = 1

            def wait_for_joint_state_snapshot(self, **_kwargs):
                return SimpleNamespace(positions=positions, velocities=velocities)

            def wait_for_execute_action(self, **_kwargs):
                raise AssertionError("MoveIt must not be contacted while robot is moving.")

        planner = RosMoveItPlanner.__new__(RosMoveItPlanner)
        planner.args = SimpleNamespace(
            enable_local_execution=True,
            state_timeout=1.0,
            start_state_tolerance=0.01,
            max_start_velocity=0.01,
        )
        planner.node = FakeStateNode()
        planner.JOINTS = joints

        context = ExecutionContext(
            trajectory="must-not-execute",
            start_positions=positions,
            target_positions=positions,
        )
        with self.assertRaises(ExecutionRejected) as rejected:
            planner.execute(context)
        self.assertEqual(rejected.exception.code, "ROBOT_NOT_SETTLED")


if __name__ == "__main__":
    unittest.main()
