#!/usr/bin/env python3

from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "windows_vlm_bridge"))
sys.path.insert(0, str(ROOT / "scripts"))

from static_vla_shadow_session import (  # noqa: E402
    InferenceResult,
    get_json,
    make_plan_function,
    run_shadow_session,
    validate_plan_receipt,
    validate_shadow_health,
    validate_ubuntu_endpoints,
)
from fr3_ai_supervisor import SupervisorHttpServer  # noqa: E402
from tests.test_fr3_action_protocol import SECRET, make_core  # noqa: E402


def args(**overrides):
    values = {
        "min_confidence": 0.70,
        "max_delta": 0.03,
        "max_total_delta": 0.03,
        "max_moving_joints": 1,
        "max_cumulative_delta": 0.15,
        "max_steps": 10,
        "max_session_seconds": 120.0,
        "interval": 0.0,
        "no_action_policy": "stop",
    }
    values.update(overrides)
    return SimpleNamespace(**values)


def action(delta=0.03, side="right", confidence=0.95):
    return {
        "action": "joint_deltas",
        "deltas": {"fr3_joint3": delta},
        "confidence": confidence,
        "observation": {"red_marker_side": side},
    }


def inference(delta=0.03, side="right", confidence=0.95):
    return InferenceResult(
        action=action(delta, side, confidence),
        image_sha256="a" * 64,
        mime_type="image/png",
    )


class ShadowHealthTest(unittest.TestCase):
    def test_plan_only_health_is_required(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "PLAN_ONLY"):
            validate_shadow_health(
                200,
                {
                    "mode": "SUPERVISED_SINGLE_STEP",
                    "execution_available": False,
                    "remote_execution_available": False,
                    "local_approval_available": True,
                },
            )

    def test_plan_only_health_is_accepted(self) -> None:
        validate_shadow_health(
            200,
            {
                "mode": "PLAN_ONLY",
                "execution_available": False,
                "remote_execution_available": False,
                "local_approval_available": False,
            },
        )

    def test_health_and_plan_must_use_same_server_and_fixed_paths(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "same server"):
            validate_ubuntu_endpoints(
                "http://10.10.10.1:8080/health",
                "http://10.10.10.9:8080/v1/actions/plan",
            )
        with self.assertRaisesRegex(RuntimeError, "path must be exactly"):
            validate_ubuntu_endpoints(
                "http://10.10.10.1:8080/health",
                "http://10.10.10.1:8080/v1/actions/execute",
            )


class PlanReceiptTest(unittest.TestCase):
    def test_mismatched_action_hash_is_rejected(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "action hash"):
            validate_plan_receipt(
                200,
                {
                    "status": "PLAN_READY",
                    "request_id": "request-1",
                    "action_sha256": "b" * 64,
                    "plan_only": True,
                    "execution_available": False,
                    "remote_execution_available": False,
                    "local_approval_available": False,
                },
                request_id="request-1",
                expected_action_digest="a" * 64,
            )


class ShadowSessionStateMachineTest(unittest.TestCase):
    def test_steps_are_infer_then_plan_with_one_request_at_a_time(self) -> None:
        events = []
        call_order = []
        images = [Path("right.png"), Path("left.png")]

        def infer(image, _remaining):
            call_order.append(f"infer:{image.name}")
            return inference(0.03, "right") if image.name == "right.png" else inference(-0.03, "left")

        def submit(_action, _digest, frame_id, _remaining):
            call_order.append(f"plan:{frame_id[-4:]}")
            return {
                "request_id": f"request-{frame_id[-4:]}",
                "action_sha256": "c" * 64,
                "planning": {"result": "SUCCESS"},
            }

        exit_code, summary = run_shadow_session(
            args(),
            images,
            event_sink=events.append,
            infer=infer,
            submit_plan=submit,
            session_id="12345678-1234-4234-8234-123456789abc",
        )

        self.assertEqual(exit_code, 0)
        self.assertEqual(summary["status"], "COMPLETED")
        self.assertEqual(summary["accepted_steps"], 2)
        self.assertEqual(summary["plan_ready_steps"], 2)
        self.assertAlmostEqual(summary["cumulative_absolute_delta"], 0.06)
        self.assertEqual(
            call_order,
            ["infer:right.png", "plan:0001", "infer:left.png", "plan:0002"],
        )
        self.assertEqual(
            [event["event"] for event in events if event["event"] == "PLAN_READY"],
            ["PLAN_READY", "PLAN_READY"],
        )

    def test_cumulative_limit_stops_before_second_plan(self) -> None:
        plan_calls = []

        def submit(*_args):
            plan_calls.append(True)
            return {
                "request_id": "request",
                "action_sha256": "c" * 64,
                "planning": {"result": "SUCCESS"},
            }

        exit_code, summary = run_shadow_session(
            args(max_cumulative_delta=0.05),
            [Path("one.png"), Path("two.png")],
            event_sink=lambda _event: None,
            infer=lambda _image, _remaining: inference(),
            submit_plan=submit,
        )

        self.assertEqual(exit_code, 1)
        self.assertEqual(summary["status"], "CUMULATIVE_LIMIT_REACHED")
        self.assertEqual(summary["accepted_steps"], 1)
        self.assertEqual(len(plan_calls), 1)

    def test_no_action_stops_without_plan_request(self) -> None:
        plan_calls = []
        exit_code, summary = run_shadow_session(
            args(),
            [Path("nothing.png"), Path("right.png")],
            event_sink=lambda _event: None,
            infer=lambda _image, _remaining: inference(0.0, "unclear", 0.0),
            submit_plan=lambda *_args: plan_calls.append(True),
        )

        self.assertEqual(exit_code, 0)
        self.assertEqual(summary["status"], "TASK_COMPLETE_NO_ACTION")
        self.assertEqual(summary["no_action_steps"], 1)
        self.assertEqual(summary["accepted_steps"], 0)
        self.assertEqual(plan_calls, [])

    def test_max_steps_truncates_input_sequence(self) -> None:
        infer_calls = []
        exit_code, summary = run_shadow_session(
            args(max_steps=2),
            [Path("one.png"), Path("two.png"), Path("three.png")],
            event_sink=lambda _event: None,
            infer=lambda image, _remaining: infer_calls.append(image) or inference(),
            submit_plan=None,
        )

        self.assertEqual(exit_code, 0)
        self.assertEqual(summary["status"], "STEP_LIMIT_REACHED")
        self.assertEqual(summary["accepted_steps"], 2)
        self.assertEqual(len(infer_calls), 2)


class ShadowHttpIntegrationTest(unittest.TestCase):
    def test_real_shadow_client_exchanges_signed_plan_with_plan_only_server(self) -> None:
        core = make_core(execution_enabled=False)
        server = SupervisorHttpServer(("127.0.0.1", 0), core, {"127.0.0.1"})
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            base_url = f"http://127.0.0.1:{server.server_port}"
            health_status, health = get_json(f"{base_url}/health", timeout=2.0)
            validate_shadow_health(health_status, health)

            client_args = SimpleNamespace(
                ubuntu_plan_url=f"{base_url}/v1/actions/plan",
                ubuntu_timeout=2.0,
            )
            submit = make_plan_function(client_args, SECRET)
            receipt = submit(
                action(),
                "a" * 64,
                "shadow-12345678-1234-4234-8234-123456789abc-step-0001",
                2.0,
            )
            self.assertEqual(receipt["status"], "PLAN_READY")
            self.assertTrue(receipt["plan_only"])
            self.assertFalse(receipt["local_approval_available"])
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2.0)


if __name__ == "__main__":
    unittest.main()
