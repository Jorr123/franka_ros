#!/usr/bin/env python3

import hashlib
from pathlib import Path
import sys
import time
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "windows_vlm_bridge"))
sys.path.insert(0, str(ROOT / "scripts"))

from fr3_action_protocol import (  # noqa: E402
    canonical_json_bytes,
    create_session_arm_envelope,
    create_session_step_envelope,
    create_session_stop_envelope,
    make_signature,
)
from fr3_ai_supervisor import (  # noqa: E402
    ExecutionContext,
    ExecutionRejected,
    PreparedPlan,
    RequestFailure,
)
from fr3_bounded_session_supervisor import (  # noqa: E402
    BoundedSessionCore,
    SessionLimits,
    parse_args,
    validate_args,
)
from tests.test_fr3_action_protocol import SECRET  # noqa: E402


SESSION_ID = "aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa"
OTHER_SESSION_ID = "cccccccc-cccc-4ccc-8ccc-cccccccccccc"


class BoundedSessionConfigurationTest(unittest.TestCase):
    def test_absolute_per_joint_delta_cap_is_point_one_rad(self) -> None:
        with mock.patch.object(
            sys,
            "argv",
            ["fr3_bounded_session_supervisor.py", "--enable-bounded-session-execution"],
        ):
            args = parse_args()

        args.max_delta = 0.10
        validate_args(args)

        args.max_delta = 0.100001
        with self.assertRaisesRegex(RuntimeError, "absolute supervisor cap"):
            validate_args(args)


def action(delta=0.03, side="right", joint="fr3_joint3", confidence=0.95):
    return {
        "action": "joint_deltas",
        "deltas": {joint: delta},
        "confidence": confidence,
        "observation": {"red_marker_side": side},
    }


class FakeSessionPlanner:
    def __init__(self, execution_error=None, on_plan=None):
        self.plan_calls = 0
        self.execute_calls = 0
        self.execution_error = execution_error
        self.on_plan = on_plan

    def plan(self, envelope):
        self.plan_calls += 1
        if self.on_plan is not None:
            self.on_plan()
        current = {f"fr3_joint{index}": 0.0 for index in range(1, 8)}
        target = dict(current)
        for joint, delta in envelope["action"]["deltas"].items():
            target[joint] += float(delta)
        return PreparedPlan(
            http_status=200,
            response={
                "status": "PLAN_READY",
                "validated_deltas": dict(envelope["action"]["deltas"]),
                "current_joint_positions": current,
                "target_joint_positions": target,
                "planning": {
                    "result": "SUCCESS",
                    "point_count": 6,
                    "duration_seconds": 0.5,
                },
            },
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
        moving = [
            joint
            for joint in context.start_positions
            if abs(context.target_positions[joint] - context.start_positions[joint]) > 1e-9
        ]
        commanded = []
        for joint in moving:
            requested_delta = (
                context.target_positions[joint] - context.start_positions[joint]
            )
            commanded.append(
                {
                    "joint": joint,
                    "requested_delta": requested_delta,
                    "measured_delta": requested_delta,
                    "target_error": 0.0,
                    "direction_ok": True,
                    "movement_ok": True,
                    "target_ok": True,
                }
            )
        return {
            "status": "VERIFIED_SUCCESS",
            "code": "VERIFIED_SUCCESS",
            "moveit_execution_result": "SUCCESS",
            "verification": {
                "status": "PASSED",
                "commanded_joints": commanded,
                "max_non_commanded_drift": 0.0,
                "max_non_commanded_drift_joint": "fr3_joint1",
                "non_commanded_drift_ok": True,
                "max_settled_velocity": 0.0,
                "max_settled_velocity_joint": "fr3_joint1",
                "settled_velocity_ok": True,
                "measured_joint_positions": dict(context.target_positions),
                "settled_joint_velocities": {
                    selected_joint: 0.0 for selected_joint in context.start_positions
                },
            },
        }


def make_core(planner=None, event_sink=None, *, remote_arm_joints=None):
    return BoundedSessionCore(
        planner or FakeSessionPlanner(),
        SECRET,
        max_request_age_ms=10_000,
        max_future_skew_ms=2_000,
        replay_retention_seconds=600.0,
        server_limits=SessionLimits(
            allowed_joints=tuple(f"fr3_joint{index}" for index in range(1, 8)),
            max_moving_joints=2,
            max_steps=4,
            max_duration_seconds=60.0,
            max_delta=0.03,
            max_total_delta=0.06,
            max_cumulative_delta=0.12,
        ),
        minimum_confidence=0.70,
        remote_arm_joints=remote_arm_joints,
        event_sink=event_sink,
    )


def arm(
    core,
    *,
    session_id=SESSION_ID,
    allowed_joints=("fr3_joint3",),
    max_moving_joints=1,
    max_steps=4,
    duration=60.0,
    max_delta=0.03,
    max_total_delta=0.03,
    cumulative=0.12,
):
    return core.handle_local(
        {
            "operation": "arm_session",
            "session_id": session_id,
            "allowed_joints": list(allowed_joints),
            "max_moving_joints": max_moving_joints,
            "max_steps": max_steps,
            "max_duration_seconds": duration,
            "max_delta": max_delta,
            "max_total_delta": max_total_delta,
            "max_cumulative_delta": cumulative,
        }
    )


def signed_step(session_id=SESSION_ID, sequence=1, candidate=None):
    envelope = create_session_step_envelope(
        candidate or action(),
        session_id=session_id,
        sequence=sequence,
        frame_source="windows-static-execution-session",
        frame_id=f"execute-{session_id}-seq-{sequence:04d}",
        frame_sha256=hashlib.sha256(f"image-{sequence}".encode()).hexdigest(),
    )
    body = canonical_json_bytes(envelope)
    return body, make_signature(body, SECRET)


def signed_stop(session_id=SESSION_ID):
    envelope = create_session_stop_envelope(session_id)
    body = canonical_json_bytes(envelope)
    return body, make_signature(body, SECRET)


def signed_arm(
    *,
    session_id=SESSION_ID,
    joints=("fr3_joint3",),
    max_moving_joints=1,
    max_steps=2,
    duration=60.0,
    max_delta=0.01,
    max_total_delta=0.01,
    cumulative=0.02,
):
    envelope = create_session_arm_envelope(
        session_id=session_id,
        allowed_joints=list(joints),
        max_moving_joints=max_moving_joints,
        max_steps=max_steps,
        max_duration_seconds=duration,
        max_delta=max_delta,
        max_total_delta=max_total_delta,
        max_cumulative_delta=cumulative,
    )
    body = canonical_json_bytes(envelope)
    return body, make_signature(body, SECRET)


class BoundedSessionCoreTest(unittest.TestCase):
    def test_remote_arm_is_disabled_by_default(self) -> None:
        core = make_core()
        self.assertFalse(core.health_payload()["remote_session_arm_available"])
        body, signature = signed_arm()
        with self.assertRaises(RequestFailure) as rejected:
            core.handle_remote_arm(body, signature)
        self.assertEqual(rejected.exception.code, "REMOTE_ARM_DISABLED")
        self.assertIsNone(core.active_session)

    def test_authenticated_remote_arm_uses_ubuntu_policy(self) -> None:
        events = []
        core = make_core(event_sink=events.append, remote_arm_joints=("fr3_joint3",))
        health = core.health_payload()
        self.assertTrue(health["remote_session_arm_available"])
        self.assertEqual(health["remote_arm_policy"]["allowed_joints"], ["fr3_joint3"])
        self.assertTrue(health["multi_joint_bounded_sessions"])

        body, signature = signed_arm()
        status, response = core.handle_remote_arm(body, signature)
        self.assertEqual(status, 201)
        self.assertEqual(response["status"], "SESSION_ARMED")
        self.assertEqual(response["session"]["status"], "ARMED")
        self.assertEqual(response["session"]["limits"]["max_delta"], 0.01)
        self.assertEqual(events[-1]["source"], "authenticated_windows_arm")

        with self.assertRaises(RequestFailure) as replayed:
            core.handle_remote_arm(body, signature)
        self.assertEqual(replayed.exception.code, "REPLAY_REJECTED")

    def test_remote_arm_rejects_wrong_joint_and_server_limit(self) -> None:
        core = make_core(remote_arm_joints=("fr3_joint3",))
        body, signature = signed_arm(joints=("fr3_joint7",))
        with self.assertRaises(RequestFailure) as wrong_joint:
            core.handle_remote_arm(body, signature)
        self.assertEqual(wrong_joint.exception.code, "REMOTE_JOINT_REJECTED")
        self.assertIsNone(core.active_session)

        body, signature = signed_arm(
            max_delta=0.04, max_total_delta=0.04, cumulative=0.04
        )
        with self.assertRaises(RequestFailure) as excessive:
            core.handle_remote_arm(body, signature)
        self.assertEqual(excessive.exception.code, "SESSION_LIMIT_REJECTED")
        self.assertIsNone(core.active_session)

    def test_remote_arm_rejects_invalid_signature(self) -> None:
        core = make_core(remote_arm_joints=("fr3_joint3",))
        body, _signature = signed_arm()
        with self.assertRaises(RequestFailure) as rejected:
            core.handle_remote_arm(body, "sha256=" + "0" * 64)
        self.assertEqual(rejected.exception.code, "INVALID_SIGNATURE")
        self.assertIsNone(core.active_session)

    def test_no_step_is_planned_without_local_arm(self) -> None:
        planner = FakeSessionPlanner()
        core = make_core(planner)
        body, signature = signed_step()
        with self.assertRaises(RequestFailure) as rejected:
            core.handle_step(body, signature)
        self.assertEqual(rejected.exception.code, "SESSION_NOT_FOUND")
        self.assertEqual(planner.plan_calls, 0)
        self.assertEqual(planner.execute_calls, 0)

    def test_armed_step_plans_executes_verifies_and_advances_once(self) -> None:
        events = []
        planner = FakeSessionPlanner()
        core = make_core(planner, events.append)
        arm_status, armed = arm(core)
        self.assertEqual(arm_status, 201)
        self.assertEqual(armed["status"], "ARMED")

        body, signature = signed_step()
        status, response = core.handle_step(body, signature)
        self.assertEqual(status, 200)
        self.assertEqual(response["status"], "VERIFIED_SUCCESS")
        self.assertEqual(response["session"]["next_sequence"], 2)
        self.assertEqual(response["session"]["completed_steps"], 1)
        self.assertAlmostEqual(response["session"]["cumulative_absolute_delta"], 0.03)
        measured = response["execution"]["verification"]["commanded_joints"][0]
        self.assertEqual(measured["joint"], "fr3_joint3")
        self.assertAlmostEqual(measured["measured_delta"], 0.03)
        self.assertEqual(planner.plan_calls, 1)
        self.assertEqual(planner.execute_calls, 1)
        self.assertIn("STEP_VERIFIED", [event["event"] for event in events])

        with self.assertRaises(RequestFailure) as replayed:
            core.handle_step(body, signature)
        self.assertEqual(replayed.exception.code, "REPLAY_REJECTED")
        self.assertEqual(planner.execute_calls, 1)

    def test_two_joint_step_uses_l1_budget_and_verifies_exact_joint_set(self) -> None:
        core = make_core()
        arm(
            core,
            allowed_joints=("fr3_joint1", "fr3_joint3"),
            max_moving_joints=2,
            max_steps=2,
            max_delta=0.01,
            max_total_delta=0.02,
            cumulative=0.04,
        )
        candidate = {
            "action": "joint_deltas",
            "deltas": {"fr3_joint1": 0.01, "fr3_joint3": -0.01},
            "confidence": 0.95,
        }
        body, signature = signed_step(candidate=candidate)
        status, response = core.handle_step(body, signature)
        self.assertEqual(status, 200)
        self.assertAlmostEqual(response["session"]["cumulative_absolute_delta"], 0.02)
        commanded = response["execution"]["verification"]["commanded_joints"]
        self.assertEqual(
            {result["joint"] for result in commanded},
            {"fr3_joint1", "fr3_joint3"},
        )

    def test_missing_multi_joint_verification_faults_session(self) -> None:
        class MissingJointPlanner(FakeSessionPlanner):
            def execute(self, context):
                result = super().execute(context)
                result["verification"]["commanded_joints"].pop()
                return result

        core = make_core(MissingJointPlanner())
        arm(
            core,
            allowed_joints=("fr3_joint1", "fr3_joint3"),
            max_moving_joints=2,
            max_delta=0.01,
            max_total_delta=0.02,
            cumulative=0.04,
        )
        candidate = {
            "action": "joint_deltas",
            "deltas": {"fr3_joint1": 0.01, "fr3_joint3": -0.01},
            "confidence": 0.95,
        }
        body, signature = signed_step(candidate=candidate)
        status, response = core.handle_step(body, signature)
        self.assertEqual(status, 500)
        self.assertEqual(response["code"], "INVALID_VERIFICATION_RESULT")
        self.assertEqual(response["session"]["completed_steps"], 0)
        self.assertEqual(response["session"]["cumulative_absolute_delta"], 0.0)

    def test_malformed_measured_result_faults_after_execution(self) -> None:
        class MissingVerificationPlanner(FakeSessionPlanner):
            def execute(self, context):
                result = super().execute(context)
                del result["verification"]
                return result

        planner = MissingVerificationPlanner()
        core = make_core(planner)
        arm(core)
        body, signature = signed_step()
        status, response = core.handle_step(body, signature)
        self.assertEqual(status, 500)
        self.assertEqual(response["status"], "FAULT")
        self.assertEqual(response["code"], "INVALID_VERIFICATION_RESULT")
        self.assertEqual(response["session"]["completed_steps"], 0)
        self.assertEqual(planner.execute_calls, 1)

    def test_wrong_sequence_faults_session_before_planning(self) -> None:
        planner = FakeSessionPlanner()
        core = make_core(planner)
        arm(core)
        body, signature = signed_step(sequence=2)
        with self.assertRaises(RequestFailure) as rejected:
            core.handle_step(body, signature)
        self.assertEqual(rejected.exception.code, "SEQUENCE_MISMATCH")
        self.assertEqual(core.health_payload()["session"]["status"], "FAULT")
        self.assertEqual(planner.plan_calls, 0)

    def test_wrong_joint_faults_session_before_planning(self) -> None:
        planner = FakeSessionPlanner()
        core = make_core(planner)
        arm(core)
        body, signature = signed_step(candidate=action(joint="fr3_joint7"))
        with self.assertRaises(RequestFailure) as rejected:
            core.handle_step(body, signature)
        self.assertEqual(rejected.exception.code, "ACTION_REJECTED")
        self.assertEqual(core.health_payload()["session"]["status"], "FAULT")
        self.assertEqual(planner.plan_calls, 0)

    def test_execution_failure_is_terminal_and_cannot_retry(self) -> None:
        planner = FakeSessionPlanner(
            ExecutionRejected("VERIFICATION_FAILED", "Measured target error was too large.")
        )
        core = make_core(planner)
        arm(core)
        body, signature = signed_step()
        status, response = core.handle_step(body, signature)
        self.assertEqual(status, 500)
        self.assertEqual(response["status"], "FAULT")
        self.assertEqual(response["code"], "VERIFICATION_FAILED")
        self.assertEqual(planner.execute_calls, 1)

        new_body, new_signature = signed_step(sequence=1, candidate=action(-0.03, "left"))
        with self.assertRaises(RequestFailure) as rejected:
            core.handle_step(new_body, new_signature)
        self.assertEqual(rejected.exception.code, "SESSION_NOT_ARMED")
        self.assertEqual(planner.execute_calls, 1)

    def test_max_steps_completes_session(self) -> None:
        core = make_core()
        arm(core, max_steps=2, cumulative=0.06)
        body, signature = signed_step(sequence=1)
        core.handle_step(body, signature)
        body, signature = signed_step(sequence=2, candidate=action(-0.03, "left"))
        status, response = core.handle_step(body, signature)
        self.assertEqual(status, 200)
        self.assertEqual(response["session"]["status"], "COMPLETED")
        self.assertEqual(response["session"]["terminal_code"], "MAX_STEPS_REACHED")

    def test_cumulative_limit_faults_before_excess_motion(self) -> None:
        planner = FakeSessionPlanner()
        core = make_core(planner)
        arm(core, max_steps=4, cumulative=0.05)
        body, signature = signed_step(sequence=1)
        core.handle_step(body, signature)
        body, signature = signed_step(sequence=2, candidate=action(-0.03, "left"))
        with self.assertRaises(RequestFailure) as rejected:
            core.handle_step(body, signature)
        self.assertEqual(rejected.exception.code, "ACTION_REJECTED")
        self.assertEqual(planner.execute_calls, 1)
        self.assertEqual(core.health_payload()["session"]["status"], "FAULT")

    def test_expired_session_rejects_before_planning(self) -> None:
        planner = FakeSessionPlanner()
        core = make_core(planner)
        arm(core)
        core.active_session.expires_monotonic = time.monotonic() - 1.0
        body, signature = signed_step()
        with self.assertRaises(RequestFailure) as rejected:
            core.handle_step(body, signature)
        self.assertEqual(rejected.exception.code, "SESSION_NOT_ARMED")
        self.assertEqual(core.health_payload()["session"]["status"], "EXPIRED")
        self.assertEqual(planner.execute_calls, 0)

    def test_expiration_during_planning_prevents_execution(self) -> None:
        core = None

        def expire_during_plan():
            core.active_session.expires_monotonic = time.monotonic() - 1.0

        planner = FakeSessionPlanner(on_plan=expire_during_plan)
        core = make_core(planner)
        arm(core)
        body, signature = signed_step()
        with self.assertRaises(RequestFailure) as rejected:
            core.handle_step(body, signature)
        self.assertEqual(rejected.exception.code, "SESSION_STOPPED")
        self.assertEqual(core.health_payload()["session"]["status"], "EXPIRED")
        self.assertEqual(planner.execute_calls, 0)

    def test_signed_stop_removes_authority(self) -> None:
        planner = FakeSessionPlanner()
        core = make_core(planner)
        arm(core)
        body, signature = signed_stop()
        status, response = core.handle_stop(body, signature)
        self.assertEqual(status, 200)
        self.assertEqual(response["session"]["status"], "STOPPED")

        step_body, step_signature = signed_step()
        with self.assertRaises(RequestFailure) as rejected:
            core.handle_step(step_body, step_signature)
        self.assertEqual(rejected.exception.code, "SESSION_NOT_ARMED")
        self.assertEqual(planner.execute_calls, 0)

    def test_second_session_cannot_arm_while_first_is_active(self) -> None:
        core = make_core()
        arm(core)
        with self.assertRaises(RequestFailure) as rejected:
            arm(core, session_id=OTHER_SESSION_ID)
        self.assertEqual(rejected.exception.code, "SESSION_ALREADY_ACTIVE")

    def test_local_status_and_disarm_default_to_current_session(self) -> None:
        core = make_core()
        arm(core)

        status, current = core.handle_local({"operation": "session_status"})
        self.assertEqual(status, 200)
        self.assertEqual(current["session_id"], SESSION_ID)
        self.assertEqual(current["status"], "ARMED")

        status, stopped = core.handle_local({"operation": "disarm_session"})
        self.assertEqual(status, 200)
        self.assertEqual(stopped["session"]["session_id"], SESSION_ID)
        self.assertEqual(stopped["session"]["status"], "DISARMED")

    def test_audit_failure_prevents_session_from_arming(self) -> None:
        def failing_audit(_event):
            raise OSError("disk full")

        core = make_core(event_sink=failing_audit)
        with self.assertRaises(OSError):
            arm(core)
        self.assertIsNone(core.active_session)


if __name__ == "__main__":
    unittest.main()
