"""Cosmos planning/local approval with fake MoveIt and actual localhost sockets."""

from copy import deepcopy
from pathlib import Path
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest import mock
from urllib.error import HTTPError
from urllib.request import ProxyHandler, Request, build_opener

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

from robot_bridge.client import get_plan_status, submit_plan
from robot_bridge.protocol import JOINTS, PLAN_MODE, PLAN_PATH, ProposalError, canonical_json, make_proposal
from robot_bridge.planning import (
    CosmosMoveItPlanner, CosmosSupervisor, PlanServer, trajectory_payload,
)
from fr3_ai_supervisor import (
    ExecutionContext, ExecutionRejected, PreparedPlan, RequestFailure, create_approval_server,
)
from fr3_action_protocol import action_sha256
from fr3_supervisor_approve import send_local_request
from fr3_cosmos_action_server import parse_args, validate_config
from tests.test_robot_bridge_dry_run import state


def point(positions, seconds):
    return SimpleNamespace(positions=list(positions), velocities=[0.0] * 7, accelerations=[0.0] * 7,
                           time_from_start=SimpleNamespace(sec=int(seconds), nanosec=int((seconds % 1) * 1e9)))


class Backend:
    def __init__(self):
        self.calls = 0
        self.executions = 0
        self.failure = False

    def plan(self, envelope):
        self.calls += 1
        proposal = envelope["cosmos_proposal"]
        current = dict(zip(JOINTS, proposal["basis_state"]["joint_positions_rad"]))
        target = dict(zip(JOINTS, proposal["target_joint_positions_rad"]))
        self.context = ExecutionContext("immutable-test-trajectory", current, target)
        return PreparedPlan(200, {
            "request_id": envelope["request_id"], "status": "PLAN_READY",
            "action_sha256": action_sha256(envelope["action"]),
            "validated_deltas": envelope["action"]["deltas"],
            "current_joint_positions": current, "target_joint_positions": target,
            "planning": {"result": "SUCCESS", "point_count": 2, "duration_seconds": 1.0},
            "trajectory": {"joint_names": JOINTS, "points": [
                {"positions_rad": list(current.values()), "time_from_start_seconds": 0.0},
                {"positions_rad": list(target.values()), "time_from_start_seconds": 1.0},
            ]},
        }, self.context)

    def execute(self, context):
        self.executions += 1
        if self.failure:
            raise ExecutionRejected("VERIFICATION_FAILED", "Test measured target mismatch")
        assert context is self.context
        return {"status": "VERIFIED_SUCCESS", "code": "VERIFIED_SUCCESS",
                "verification": {"passed": True, "measured_joint_positions": context.target_positions}}


class CoreTest(unittest.TestCase):
    def setUp(self):
        self.live = state()
        self.backend = Backend()
        self.core = CosmosSupervisor(self.backend, snapshot=self.snapshot, execution_enabled=True)

    def snapshot(self):
        live = deepcopy(self.live)
        live["joint_state_received_unix_ms"] = int(time.time() * 1000)
        live["joint_state_stamp_ns"] = int(time.time() * 1e9)
        return 200, live

    def proposal(self, delta=0.001):
        measured = self.snapshot()[1]
        now = int(time.time() * 1000)
        return make_proposal(source="cosmos3-edge-droid", observation_state=measured, basis_state=measured,
                             target_joint_positions_rad=[q + delta for q in measured["joint_positions_rad"]],
                             observation_created_at_unix_ms=now - 100, inference_completed_at_unix_ms=now,
                             mode=PLAN_MODE)

    def prepare(self):
        self.message = self.proposal()
        _, receipt = self.core.prepare_proposal(self.message)
        self.plan = receipt["plan"]
        return receipt

    def approve(self):
        return self.core.handle_local({"operation": "execute", "request_id": self.message["request_id"],
                                       "action_sha256": self.plan["action_sha256"]})

    def test_no_execution_during_plan_and_exactly_one_after_local_approval(self):
        receipt = self.prepare()
        self.assertEqual(receipt["decision"], "PLAN_READY")
        self.assertEqual(self.backend.executions, 0)
        self.assertEqual(list(self.backend.context.target_positions.values()), self.message["target_joint_positions_rad"])
        code, result = self.approve()
        self.assertEqual((code, result["status"]), (200, "VERIFIED_SUCCESS"))
        self.assertTrue(result["verification"]["passed"])
        with self.assertRaisesRegex(RequestFailure, "cannot be executed"):
            self.approve()
        self.assertEqual(self.backend.executions, 1)

    def test_plan_only_cannot_execute(self):
        self.core.execution_enabled = False
        self.prepare()
        self.assertFalse(self.plan["local_approval_available"])
        with self.assertRaisesRegex(RequestFailure, "disabled"):
            self.approve()
        self.assertEqual(self.backend.executions, 0)

    def test_excessive_target_rejected_before_planner(self):
        with self.assertRaises(ProposalError):
            self.core.prepare_proposal(self.proposal(0.02))
        self.assertEqual(self.backend.calls, 0)

    def test_wrong_hash_expiry_and_supersession_cannot_execute(self):
        self.prepare()
        with self.assertRaisesRegex(RequestFailure, "hash"):
            self.core.handle_local({"operation": "execute", "request_id": self.message["request_id"], "action_sha256": "0" * 64})
        old = self.message["request_id"]
        self.core.prepare_proposal(self.proposal())
        self.assertEqual(self.core.records[old].status, "SUPERSEDED")
        self.prepare()
        self.core.records[self.message["request_id"]].expires_monotonic = time.monotonic() - 1
        with self.assertRaises(RequestFailure):
            self.approve()
        self.assertEqual(self.backend.executions, 0)

    def test_pose_gripper_or_publisher_change_blocks_approval(self):
        for kind in ("pose", "gripper", "source"):
            with self.subTest(kind=kind):
                self.live = state()
                self.prepare()
                if kind == "pose":
                    self.live["joint_positions_rad"][0] += 0.006
                elif kind == "gripper":
                    self.live["gripper_width_m"] += 0.002
                else:
                    self.live["joint_state_publisher"]["gid"] = "02" * 24
                _, result = self.approve()
                self.assertEqual(result["status"], "FAULT")
        self.assertEqual(self.backend.executions, 0)

    def test_original_observation_deadline_limits_approval_ttl(self):
        message = self.proposal()
        message["prediction"]["observation_created_at_unix_ms"] = int(time.time() * 1000) - 18000
        _, receipt = self.core.prepare_proposal(message)
        remaining = receipt["plan"]["plan_expires_at_unix_ms"] / 1000 - time.time()
        self.assertGreater(remaining, 10)
        self.assertLessEqual(remaining, 12.1)

    def test_audit_failure_or_verification_failure_is_not_retried(self):
        self.prepare()
        self.core.audit = mock.Mock(side_effect=OSError("disk full"))
        _, result = self.approve()
        self.assertEqual(result["status"], "FAULT")
        self.assertEqual(self.backend.executions, 0)
        self.core.audit = lambda event: None
        self.prepare()
        self.backend.failure = True
        _, result = self.approve()
        self.assertEqual(result["code"], "VERIFICATION_FAILED")
        with self.assertRaises(RequestFailure):
            self.approve()
        self.assertEqual(self.backend.executions, 1)

    def test_absolute_target_not_rebased_on_a_new_planning_start(self):
        message = self.proposal(0.002)
        planner = CosmosMoveItPlanner.__new__(CosmosMoveItPlanner)
        planner.snapshot = self.snapshot
        planner.limits = self.core.limits
        planner.args = SimpleNamespace()
        planner.validate_deltas = mock.Mock()
        current = dict(zip(JOINTS, [q + 0.0005 for q in self.live["joint_positions_rad"]]))
        deltas = dict(message["joint_deltas_rad"])
        target = planner.resolve_target({"cosmos_proposal": message}, current, deltas)
        self.assertEqual(list(target.values()), message["target_joint_positions_rad"])
        self.assertAlmostEqual(deltas[JOINTS[0]], 0.0015)

    def test_trajectory_waypoints_and_timing_are_checked(self):
        start = dict(zip(JOINTS, [0.1] * 7))
        target = dict(zip(JOINTS, [0.101] * 7))
        trajectory = SimpleNamespace(joint_names=JOINTS, points=[point(start.values(), 0), point(target.values(), 1)])
        self.assertEqual(len(trajectory_payload(trajectory, start, target, self.core.limits, 0.0001)["points"]), 2)
        for kind in ("overshoot", "nan", "time", "endpoint", "joint_order"):
            changed = deepcopy(trajectory)
            if kind == "overshoot":
                changed.points.insert(1, point([0.12] * 7, 0.5))
            elif kind == "nan":
                changed.points[1].positions[0] = float("nan")
            elif kind == "time":
                changed.points[1].time_from_start.sec = 0
            elif kind == "endpoint":
                changed.points[1].positions = [0.103] * 7
            else:
                changed.joint_names = JOINTS[::-1]
            with self.subTest(kind=kind), self.assertRaises(ValueError if kind == "nan" else RuntimeError):
                trajectory_payload(changed, start, target, self.core.limits, 0.0001)

    def test_initial_launcher_defaults_and_limit_ceilings(self):
        args = parse_args([])
        self.assertFalse(args.enable_local_execution)
        self.assertEqual(args.state_topic, "/franka/joint_states")
        validate_config(args)
        for option, value in (("--max-delta", "0.02"), ("--velocity-scaling", "0.1"), ("--start-state-tolerance", "0.01")):
            with self.subTest(option=option), self.assertRaises(ValueError):
                validate_config(parse_args([option, value]))

    def test_final_execution_check_rejects_motion_source_change_or_velocity(self):
        for kind in ("pose", "source", "velocity"):
            with self.subTest(kind=kind):
                self.live = state()
                message = self.proposal()
                current = dict(zip(JOINTS, self.live["joint_positions_rad"]))
                measured = SimpleNamespace(positions=dict(current), velocities=dict(zip(JOINTS, [0.0] * 7)))
                planner = CosmosMoveItPlanner.__new__(CosmosMoveItPlanner)
                planner.snapshot = self.snapshot
                planner.limits = self.core.limits
                planner.args = SimpleNamespace(state_timeout=1.0, start_state_tolerance=0.002, max_start_velocity=0.01)
                planner.node = SimpleNamespace(joint_state_sequence=1, wait_for_joint_state_snapshot=lambda **kwargs: measured)
                context = ExecutionContext("frozen", current, current, {"cosmos_proposal": message})
                if kind == "pose":
                    measured.positions[JOINTS[0]] += 0.003
                elif kind == "source":
                    self.live["joint_state_publisher"]["gid"] = "02" * 24
                else:
                    measured.velocities[JOINTS[0]] = 0.02
                with self.assertRaises(ExecutionRejected):
                    planner.before_execute(context)


class HttpAndApprovalTest(unittest.TestCase):
    snapshot = CoreTest.snapshot
    proposal = CoreTest.proposal

    def setUp(self):
        CoreTest.setUp(self)
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.server = PlanServer(("127.0.0.1", 0), core=self.core, allowed_clients={"127.0.0.1"},
                                 audit_log=Path(self.directory.name) / "audit.jsonl")
        self.thread = threading.Thread(target=self.server.serve_forever, kwargs={"poll_interval": 0.01}, daemon=True)
        self.thread.start()
        self.base_url = f"http://127.0.0.1:{self.server.server_port}"
        self.addCleanup(self.close_http)

    def close_http(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)

    def test_http_can_plan_and_read_status_but_cannot_execute(self):
        self.message = self.proposal()
        receipt = submit_plan(self.message, url=self.base_url + PLAN_PATH)
        self.plan = receipt["plan"]
        result = get_plan_status(self.message["request_id"], base_url=self.base_url)
        self.assertEqual(result["status"], "READY_FOR_LOCAL_APPROVAL")
        request = Request(self.base_url + "/v1/actions/execute", data=b"{}", method="POST")
        with self.assertRaises(HTTPError) as error:
            build_opener(ProxyHandler({})).open(request, timeout=2)
        self.assertEqual(error.exception.code, 404)
        self.assertEqual(self.backend.executions, 0)

    def test_local_socket_returns_verified_result_once(self):
        self.message = self.proposal()
        self.plan = submit_plan(self.message, url=self.base_url + PLAN_PATH)["plan"]
        path = Path(self.directory.name) / "approval.sock"
        server = create_approval_server(path, self.core)
        thread = threading.Thread(target=server.serve_forever, kwargs={"poll_interval": 0.01}, daemon=True)
        thread.start()
        try:
            result = send_local_request(path, {"operation": "execute", "request_id": self.message["request_id"],
                                              "action_sha256": self.plan["action_sha256"]}, timeout=2)
            self.assertEqual(result["status"], "VERIFIED_SUCCESS")
            result = get_plan_status(self.message["request_id"], base_url=self.base_url)
            self.assertEqual(result["result"]["status"], "VERIFIED_SUCCESS")
            self.assertEqual(path.stat().st_mode & 0o777, 0o600)
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2)
            path.unlink(missing_ok=True)

    def test_audit_failure_revokes_frozen_plan(self):
        self.message = self.proposal()
        with mock.patch.object(self.server, "record", side_effect=OSError("test disk failure")):
            receipt = submit_plan(self.message, url=self.base_url + PLAN_PATH)
        self.assertEqual(receipt["decision"], "REJECTED")
        self.assertEqual(self.core.records[self.message["request_id"]].status, "FAULT")
        self.assertEqual(self.backend.executions, 0)


if __name__ == "__main__":
    unittest.main()
