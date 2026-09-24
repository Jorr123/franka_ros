#!/usr/bin/env python3

from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "windows_vlm_bridge"))
sys.path.insert(0, str(ROOT / "scripts"))

from fr3_bounded_session_supervisor import BoundedSessionHttpServer  # noqa: E402
from isaac_qwen_coordinator import post_json  # noqa: E402
from static_vla_execution_session import (  # noqa: E402
    InferenceResult,
    make_step_function,
    run_execution_session,
    send_stop,
    validate_action_for_session,
    validate_armed_health,
    validate_endpoints,
    validate_step_receipt,
)
from static_vla_shadow_session import get_json  # noqa: E402
from tests.test_bounded_session_supervisor import (  # noqa: E402
    SESSION_ID,
    action,
    arm,
    make_core,
)
from tests.test_fr3_action_protocol import SECRET  # noqa: E402


def args(**overrides):
    values = {
        "min_confidence": 0.70,
        "max_delta": 0.03,
        "max_total_delta": 0.03,
        "max_moving_joints": 1,
        "max_images": 5,
        "max_session_seconds": 60.0,
        "interval": 0.0,
        "ubuntu_timeout": 5.0,
        "ubuntu_health_url": "http://127.0.0.1:8080/health",
        "ubuntu_step_url": "http://127.0.0.1:8080/v1/sessions/step",
        "ubuntu_stop_url": "http://127.0.0.1:8080/v1/sessions/stop",
    }
    values.update(overrides)
    return SimpleNamespace(**values)


def session(max_steps=4, cumulative=0.12):
    return {
        "session_id": SESSION_ID,
        "status": "ARMED",
        "next_sequence": 1,
        "completed_steps": 0,
        "remaining_seconds": 60.0,
        "limits": {
            "allowed_joints": ["fr3_joint3"],
            "max_moving_joints": 1,
            "max_steps": max_steps,
            "max_duration_seconds": 60.0,
            "max_delta": 0.03,
            "max_total_delta": 0.03,
            "max_cumulative_delta": cumulative,
        },
    }


def health(session_value=None):
    selected = session_value or session()
    return {
        "status": "ok",
        "mode": "BOUNDED_SESSION_EXECUTION",
        "schema_version": 2,
        "multi_joint_bounded_sessions": True,
        "remote_direct_execution_available": False,
        "session_step_available": True,
        "session_armed": True,
        "session": selected,
    }


def inference(delta=0.03, side="right", joint="fr3_joint3", confidence=0.95):
    return InferenceResult(
        action=action(delta, side, joint, confidence),
        image_sha256="a" * 64,
        mime_type="image/png",
    )


def receipt(sequence, status="ARMED", requested=0.03):
    joints = [f"fr3_joint{index}" for index in range(1, 8)]
    before = {joint: 0.0 for joint in joints}
    target = dict(before)
    target["fr3_joint3"] = requested
    after = dict(target)
    return {
        "status": "VERIFIED_SUCCESS",
        "request_id": f"request-{sequence}",
        "action_sha256": "b" * 64,
        "session_id": SESSION_ID,
        "sequence": sequence,
        "planning": {"result": "SUCCESS"},
        "current_joint_positions": before,
        "target_joint_positions": target,
        "execution": {
            "status": "VERIFIED_SUCCESS",
            "verification": {
                "status": "PASSED",
                "commanded_joints": [
                    {
                        "joint": "fr3_joint3",
                        "requested_delta": requested,
                        "measured_delta": requested,
                        "target_error": 0.0,
                        "direction_ok": True,
                        "movement_ok": True,
                        "target_ok": True,
                    }
                ],
                "non_commanded_drift_ok": True,
                "settled_velocity_ok": True,
                "measured_joint_positions": after,
                "settled_joint_velocities": {joint: 0.0 for joint in joints},
            },
        },
        "session": {
            **session(),
            "status": status,
            "next_sequence": sequence + 1,
            "completed_steps": sequence,
            "cumulative_absolute_delta": sequence * abs(requested),
        },
    }


class ExecutionClientValidationTest(unittest.TestCase):
    def test_health_discovers_one_fresh_locally_armed_session(self) -> None:
        selected = validate_armed_health(
            200,
            health(),
            expected_session_id=None,
            args=args(),
        )
        self.assertEqual(selected["session_id"], SESSION_ID)

        wrong = health()
        wrong["session_armed"] = False
        with self.assertRaisesRegex(RuntimeError, "No bounded session"):
            validate_armed_health(
                200,
                wrong,
                expected_session_id=None,
                args=args(),
            )

    def test_explicit_session_id_must_match_discovered_session(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "does not match --session-id"):
            validate_armed_health(
                200,
                health(),
                expected_session_id="bbbbbbbb-bbbb-4bbb-8bbb-bbbbbbbbbbbb",
                args=args(),
            )

    def test_discovery_rejects_session_that_already_started(self) -> None:
        started = session()
        started["next_sequence"] = 2
        started["completed_steps"] = 1
        with self.assertRaisesRegex(RuntimeError, "fresh session"):
            validate_armed_health(
                200,
                health(started),
                expected_session_id=None,
                args=args(),
            )

    def test_discovery_rejects_larger_windows_cumulative_budget(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "max-cumulative-delta"):
            validate_armed_health(
                200,
                health(session(cumulative=0.04)),
                expected_session_id=None,
                args=args(max_cumulative_delta=0.05),
            )

    def test_urls_use_fixed_paths_and_same_server(self) -> None:
        validate_endpoints(
            "http://10.10.10.1:8080/health",
            "http://10.10.10.1:8080/v1/sessions/step",
            "http://10.10.10.1:8080/v1/sessions/stop",
        )
        with self.assertRaisesRegex(RuntimeError, "path must be exactly"):
            validate_endpoints(
                "http://10.10.10.1:8080/health",
                "http://10.10.10.1:8080/v1/actions/execute",
                "http://10.10.10.1:8080/v1/sessions/stop",
            )

    def test_action_must_use_locally_armed_joint(self) -> None:
        moving, total = validate_action_for_session(action(), args(), session(), 0.0)
        self.assertEqual(moving, {"fr3_joint3": 0.03})
        self.assertAlmostEqual(total, 0.03)
        with self.assertRaisesRegex(RuntimeError, "permits"):
            validate_action_for_session(
                action(joint="fr3_joint7"), args(), session(), 0.0
            )

    def test_receipt_mismatch_is_rejected(self) -> None:
        response = receipt(1)
        response["action_sha256"] = "c" * 64
        with self.assertRaisesRegex(RuntimeError, "action_sha256"):
            validate_step_receipt(
                200,
                response,
                session_id=SESSION_ID,
                sequence=1,
                request_id="request-1",
                action_digest="b" * 64,
                requested_deltas={"fr3_joint3": 0.03},
            )


class ExecutionSessionStateMachineTest(unittest.TestCase):
    def test_two_steps_are_strictly_synchronous_then_stop(self) -> None:
        order = []
        stops = []
        images = [Path("right.png"), Path("left.png")]

        def infer(image, _remaining):
            order.append(f"infer:{image.name}")
            return inference() if image.name == "right.png" else inference(-0.03, "left")

        def submit(_action, _digest, _frame, sequence, _remaining):
            order.append(f"submit:{sequence}")
            return receipt(sequence)

        exit_code, summary = run_execution_session(
            args(),
            images,
            session(),
            event_sink=lambda _event: None,
            infer=infer,
            submit_step=submit,
            stop_session=lambda _timeout: stops.append(True) or {"status": "SESSION_STOPPED"},
        )
        self.assertEqual(exit_code, 0)
        self.assertEqual(summary["status"], "INPUT_EXHAUSTED_STOPPED")
        self.assertEqual(summary["verified_steps"], 2)
        self.assertEqual(order, ["infer:right.png", "submit:1", "infer:left.png", "submit:2"])
        self.assertEqual(stops, [True])

    def test_no_action_sends_stop_without_step(self) -> None:
        submits = []
        stops = []
        exit_code, summary = run_execution_session(
            args(),
            [Path("nothing.png")],
            session(),
            event_sink=lambda _event: None,
            infer=lambda _image, _remaining: inference(0.0, "unclear", confidence=0.0),
            submit_step=lambda *_values: submits.append(True),
            stop_session=lambda _timeout: stops.append(True) or {"status": "SESSION_STOPPED"},
        )
        self.assertEqual(exit_code, 0)
        self.assertEqual(summary["status"], "TASK_COMPLETE_NO_ACTION")
        self.assertEqual(submits, [])
        self.assertEqual(stops, [True])

    def test_failed_step_is_not_retried_and_requests_stop(self) -> None:
        submits = []
        stops = []

        def fail_once(*_values):
            submits.append(True)
            raise RuntimeError("ambiguous timeout")

        exit_code, summary = run_execution_session(
            args(),
            [Path("one.png"), Path("two.png")],
            session(),
            event_sink=lambda _event: None,
            infer=lambda _image, _remaining: inference(),
            submit_step=fail_once,
            stop_session=lambda _timeout: stops.append(True) or {"status": "STOP_REQUESTED"},
        )
        self.assertEqual(exit_code, 1)
        self.assertEqual(summary["status"], "FAILED")
        self.assertEqual(len(submits), 1)
        self.assertEqual(stops, [True])

    def test_server_completed_status_ends_without_extra_stop(self) -> None:
        stops = []
        exit_code, summary = run_execution_session(
            args(),
            [Path("right.png"), Path("unused.png")],
            session(max_steps=1, cumulative=0.03),
            event_sink=lambda _event: None,
            infer=lambda _image, _remaining: inference(),
            submit_step=lambda *_values: receipt(1, "COMPLETED"),
            stop_session=lambda _timeout: stops.append(True),
        )
        self.assertEqual(exit_code, 0)
        self.assertEqual(summary["status"], "SESSION_COMPLETED")
        self.assertEqual(summary["verified_steps"], 1)
        self.assertEqual(stops, [])

    def test_local_cumulative_limit_stops_before_second_submit(self) -> None:
        submits = []
        stops = []
        exit_code, summary = run_execution_session(
            args(),
            [Path("one.png"), Path("two.png")],
            session(cumulative=0.05),
            event_sink=lambda _event: None,
            infer=lambda _image, _remaining: inference(),
            submit_step=lambda *_values: submits.append(True) or receipt(len(submits)),
            stop_session=lambda _timeout: stops.append(True) or {"status": "STOPPED"},
        )
        self.assertEqual(exit_code, 1)
        self.assertEqual(summary["status"], "FAILED")
        self.assertEqual(len(submits), 1)
        self.assertEqual(stops, [True])


class ExecutionSessionHttpIntegrationTest(unittest.TestCase):
    def test_bounded_server_has_no_generic_execution_endpoint(self) -> None:
        core = make_core()
        server = BoundedSessionHttpServer(("127.0.0.1", 0), core, {"127.0.0.1"})
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            base = f"http://127.0.0.1:{server.server_port}"
            status, response = post_json(
                f"{base}/v1/actions/execute",
                b"{}",
                timeout=2.0,
            )
            self.assertEqual(status, 404)
            self.assertEqual(response["code"], "NOT_FOUND")
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2.0)

    def test_signed_client_runs_one_verified_step_through_real_http_server(self) -> None:
        core = make_core()
        arm(core, max_steps=2, cumulative=0.06)
        server = BoundedSessionHttpServer(("127.0.0.1", 0), core, {"127.0.0.1"})
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            base = f"http://127.0.0.1:{server.server_port}"
            client_args = args(
                ubuntu_health_url=f"{base}/health",
                ubuntu_step_url=f"{base}/v1/sessions/step",
                ubuntu_stop_url=f"{base}/v1/sessions/stop",
            )
            health_status, health_response = get_json(
                client_args.ubuntu_health_url, timeout=2.0
            )
            armed = validate_armed_health(
                health_status,
                health_response,
                expected_session_id=None,
                args=client_args,
            )
            self.assertEqual(armed["status"], "ARMED")

            submit = make_step_function(client_args, SECRET, SESSION_ID)
            result = submit(
                action(),
                "a" * 64,
                f"execute-{SESSION_ID}-seq-0001",
                1,
                2.0,
            )
            self.assertEqual(result["status"], "VERIFIED_SUCCESS")
            self.assertEqual(result["session"]["next_sequence"], 2)

            stopped = send_stop(
                client_args,
                SECRET,
                SESSION_ID,
                timeout=2.0,
            )
            self.assertEqual(stopped["session"]["status"], "STOPPED")
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2.0)


if __name__ == "__main__":
    unittest.main()
