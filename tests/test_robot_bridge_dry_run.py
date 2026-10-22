"""Dry-run protocol and real localhost HTTP checks, without ROS or robot access."""

from copy import deepcopy
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from unittest import mock
from urllib.error import HTTPError
from urllib.request import ProxyHandler, Request, build_opener

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from robot_bridge.client import submit_proposal
from robot_bridge.protocol import (
    JOINTS, Limits, MODE, PATH, ProposalError, canonical_json,
    decode_json, evaluate_proposal, make_proposal,
)
from robot_bridge.server import DryRunServer


def state():
    now_ms = int(time.time() * 1000)
    return {
        "joint_names": list(JOINTS), "joint_positions_rad": [0.1] * 7,
        "joint_state_publisher": {"node_name": "joint_state_broadcaster", "node_namespace": "/", "gid": "01" * 24},
        "joint_state_sequence": 12, "joint_state_stamp_ns": now_ms * 1_000_000,
        "joint_state_received_unix_ms": now_ms,
        "joint_state_age_seconds": 0.02,
        "gripper_width_m": 0.04, "gripper_state_age_seconds": 0.03,
    }


def proposal(measured=None, delta=0.001):
    measured = measured or state()
    now_ms = int(time.time() * 1000)
    return make_proposal(
        source="test-vla", target_joint_positions_rad=[q + delta for q in measured["joint_positions_rad"]],
        observation_state=measured, basis_state=measured,
        observation_created_at_unix_ms=now_ms - 100, inference_completed_at_unix_ms=now_ms,
    )


class ProtocolTest(unittest.TestCase):
    def setUp(self):
        self.state = state()
        self.proposal = proposal(self.state)
        self.now_ms = int(time.time() * 1000)

    def evaluate(self, message=None, measured=None):
        return evaluate_proposal(message or self.proposal, measured or self.state, Limits(), self.now_ms)

    def test_absolute_target_is_converted_from_fresh_measured_basis(self):
        for joint in JOINTS:
            self.assertAlmostEqual(self.proposal["joint_deltas_rad"][joint], 0.001)
        metrics = self.evaluate()
        self.assertAlmostEqual(metrics["total_joint_delta_rad"], 0.007)
        self.assertIs(self.proposal["execution_allowed"], False)
        self.assertEqual(self.proposal["gripper_mode"], "hold")

    def test_stationary_observation_survives_eighteen_second_inference_but_not_thirty_one(self):
        self.proposal["prediction"]["observation_created_at_unix_ms"] = self.now_ms - 18000
        self.proposal["observation_state"]["joint_state_received_unix_ms"] = self.now_ms - 18000
        self.assertAlmostEqual(self.evaluate()["observation_age_seconds"], 18.0)
        self.proposal["prediction"]["observation_created_at_unix_ms"] = self.now_ms - 31000
        with self.assertRaisesRegex(ProposalError, "observation age"):
            self.evaluate()

    def test_rejects_each_delta_and_total_without_clipping(self):
        for delta in (0.301, 0.13):
            with self.subTest(delta=delta), self.assertRaisesRegex(ProposalError, "delta"):
                self.evaluate(proposal(self.state, delta))

    def test_recorded_cosmos_step_fits_current_limits(self):
        deltas = [0.0001361966, -0.0075237565, 0.0207660794, 0.0041997433,
                  -0.0166081041, 0.0001435280, -0.0028752089]
        message = make_proposal(
            source="cosmos3-edge-droid", observation_state=self.state, basis_state=self.state,
            target_joint_positions_rad=[q + delta for q, delta in zip(self.state["joint_positions_rad"], deltas)],
            observation_created_at_unix_ms=self.now_ms - 100, inference_completed_at_unix_ms=self.now_ms,
        )
        metrics = self.evaluate(message)
        self.assertAlmostEqual(metrics["max_joint_delta_rad"], 0.0207660794)
        self.assertAlmostEqual(metrics["total_joint_delta_rad"], sum(abs(delta) for delta in deltas))
        self.assertEqual(Limits().max_joint_delta_rad, 0.3)
        self.assertEqual(Limits().max_total_delta_rad, 0.9)

    def test_absolute_sum_above_old_limit_is_accepted_below_point_nine(self):
        metrics = self.evaluate(proposal(self.state, 0.12))
        self.assertAlmostEqual(metrics["total_joint_delta_rad"], 0.84)
        self.assertAlmostEqual(metrics["max_joint_delta_rad"], 0.12)

    def test_single_joint_uses_point_three_limit_independently_of_total(self):
        for delta in (0.299, 0.301):
            message = proposal(self.state, 0.0)
            message["target_joint_positions_rad"][6] += delta
            message["joint_deltas_rad"][JOINTS[6]] = delta
            if delta < 0.3:
                self.assertAlmostEqual(self.evaluate(message)["max_joint_delta_rad"], delta)
            else:
                with self.assertRaises(ProposalError):
                    self.evaluate(message)

    def test_rejects_tampered_delta_or_execution_flag(self):
        self.proposal["joint_deltas_rad"][JOINTS[0]] = 0.0
        with self.assertRaisesRegex(ProposalError, "does not equal"):
            self.evaluate()
        for field, value in (("mode", "EXECUTE"), ("execution_allowed", True), ("gripper_mode", "close")):
            changed = proposal(self.state)
            changed[field] = value
            with self.subTest(field=field), self.assertRaisesRegex(ProposalError, "validation-only"):
                self.evaluate(changed)

    def test_rejects_state_drift_gripper_motion_or_source_change(self):
        for change in ("joints", "gripper", "publisher", "basis"):
            message = deepcopy(self.proposal)
            measured = deepcopy(self.state)
            if change == "joints":
                measured["joint_positions_rad"][0] += 0.006
            elif change == "gripper":
                measured["gripper_width_m"] += 0.002
            elif change == "publisher":
                measured["joint_state_publisher"]["gid"] = "02" * 24
            else:
                # A small observed drift may pass the start-pose bound but must
                # still fail the tighter last-measurement comparison.
                measured["joint_positions_rad"][0] += 0.003
            with self.subTest(change=change), self.assertRaises(ProposalError):
                self.evaluate(message, measured)

    def test_rejects_stale_basis_current_state_request_and_prediction(self):
        for change in ("basis", "current", "request", "prediction", "future", "rewind"):
            message = deepcopy(self.proposal)
            measured = deepcopy(self.state)
            if change == "basis":
                message["basis_state"]["joint_state_received_unix_ms"] -= 2000
            elif change == "current":
                measured["joint_state_age_seconds"] = 2.0
            elif change == "request":
                message["created_at_unix_ms"] -= 6000
            elif change == "prediction":
                message["prediction"]["inference_completed_at_unix_ms"] -= 6000
            elif change == "future":
                message["created_at_unix_ms"] += 2000
            else:
                message["basis_state"]["joint_state_stamp_ns"] -= 1
            with self.subTest(change=change), self.assertRaises(ProposalError):
                self.evaluate(message, measured)

    def test_bad_numbers_names_and_missing_gripper_fail_closed(self):
        for value in (True, float("nan"), float("inf"), None):
            changed = deepcopy(self.proposal)
            changed["target_joint_positions_rad"][0] = value
            with self.subTest(value=value), self.assertRaises(ProposalError):
                self.evaluate(changed)
        self.proposal["observation_state"]["joint_names"] = list(reversed(JOINTS))
        with self.assertRaises(ProposalError):
            self.evaluate()
        measured = deepcopy(self.state)
        measured["gripper_width_m"] = None
        with self.assertRaises(ProposalError):
            self.evaluate(proposal(self.state), measured)

    def test_strict_json_rejects_duplicate_keys_and_nonfinite_constants(self):
        for body in (b'{"mode":1,"mode":2}', b'{"value":NaN}', b'[] trailing'):
            with self.subTest(body=body), self.assertRaises(ProposalError):
                decode_json(body)


class HttpTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.log = Path(self.directory.name) / "audit.jsonl"
        self.state = state()
        self.state_status = 200
        self.server = DryRunServer(
            ("127.0.0.1", 0), snapshot=lambda: (self.state_status, self.state),
            allowed_clients={"127.0.0.1"}, audit_log=self.log,
        )
        self.thread = threading.Thread(target=self.server.serve_forever, kwargs={"poll_interval": 0.01}, daemon=True)
        self.thread.start()
        self.url = f"http://127.0.0.1:{self.server.server_port}" + PATH
        self.addCleanup(self.close_server)

    def close_server(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)

    def raw_post(self, body, path=None):
        request = Request(path or self.url, data=body, headers={
            "Content-Type": "application/json",
        }, method="POST")
        opener = build_opener(ProxyHandler({}))
        try:
            response = opener.open(request, timeout=2)
        except HTTPError as error:
            response = error
        with response:
            return response.code, json.loads(response.read())

    def test_delivery_without_secret_and_receipt_audit(self):
        message = proposal(self.state)
        receipt = submit_proposal(message, url=self.url)
        self.assertEqual(receipt["decision"], "ACCEPTED_FOR_DRY_RUN")
        self.assertFalse(receipt["robot_execution"])
        record = json.loads(self.log.read_text().strip())
        self.assertEqual(record["proposal"], message)
        self.assertEqual(record["receipt"], receipt)

    def test_rejected_prediction_is_still_delivered_and_receipt_verified(self):
        receipt = submit_proposal(proposal(self.state, 0.13), url=self.url)
        self.assertEqual(receipt["decision"], "REJECTED")
        self.assertEqual(receipt["code"], "DELTA_LIMIT_EXCEEDED")
        self.assertFalse(receipt["execution_allowed"])

    def test_repeated_uuid_rejected_without_retry(self):
        message = proposal(self.state)
        submit_proposal(message, url=self.url)
        receipt = submit_proposal(message, url=self.url)
        self.assertEqual(receipt["code"], "REPLAYED_REQUEST")
        self.assertEqual(len(self.log.read_text().splitlines()), 2)

    def test_execution_intent_rejected_without_secret(self):
        message = proposal(self.state)
        message["execution_allowed"] = True
        status, receipt = self.raw_post(canonical_json(message))
        self.assertEqual(status, 422)
        self.assertEqual(receipt["code"], "EXECUTION_FORBIDDEN")

    def test_missing_live_state_and_unwritable_audit_reject(self):
        self.state_status = 503
        receipt = submit_proposal(proposal(self.state), url=self.url)
        self.assertEqual(receipt["code"], "STATE_UNAVAILABLE")
        self.state_status = 200
        with mock.patch.object(self.server, "record", side_effect=OSError("test disk failure")):
            receipt = submit_proposal(proposal(self.state), url=self.url)
        self.assertEqual(receipt["code"], "AUDIT_UNAVAILABLE")
        self.assertEqual(receipt["decision"], "REJECTED")

    def test_ip_restriction_and_execution_urls(self):
        self.server.allowed_clients = {"192.0.2.1"}
        status, receipt = self.raw_post(canonical_json(proposal(self.state)))
        self.assertEqual(status, 403)
        self.assertFalse(receipt["execution_allowed"])
        self.server.allowed_clients = {"127.0.0.1"}
        status, _ = self.raw_post(canonical_json(proposal(self.state)), path=self.url.replace(PATH, "/v1/sessions/step"))
        self.assertEqual(status, 404)
        with self.assertRaises(ValueError):
            submit_proposal(proposal(self.state), url=self.url.replace(PATH, "/v1/sessions/step"))

    def test_receipt_claiming_execution_is_rejected(self):
        import robot_bridge.server as server_module

        original = server_module.canonical_json

        def tamper(value):
            value = dict(value)
            value["robot_execution"] = True
            return original(value)

        with mock.patch.object(server_module, "canonical_json", side_effect=tamper):
            with self.assertRaisesRegex(ValueError, "validation-only"):
                submit_proposal(proposal(self.state), url=self.url)

    def test_receipt_for_another_proposal_is_rejected(self):
        import robot_bridge.server as server_module

        original = server_module.canonical_json

        def tamper(value):
            value = dict(value)
            value["proposal_sha256"] = "0" * 64
            return original(value)

        with mock.patch.object(server_module, "canonical_json", side_effect=tamper):
            with self.assertRaisesRegex(ValueError, "does not match"):
                submit_proposal(proposal(self.state), url=self.url)


if __name__ == "__main__":
    unittest.main()
