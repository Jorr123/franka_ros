#!/usr/bin/env python3
"""Authenticated FR3 planner with optional local-only supervised execution.

Windows may submit bounded action proposals over authenticated HTTP. HTTP can
only create MoveIt plans. By default the process is plan-only. When explicitly
enabled, a Linux Unix socket permits the local Ubuntu operator to execute one
fresh, frozen plan exactly once after start-state revalidation.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import ipaddress
import json
import math
import os
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import socketserver
import stat
import sys
import threading
import time
from typing import Any, Callable, Mapping


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
WINDOWS_BRIDGE_DIR = WORKSPACE_ROOT / "windows_vlm_bridge"
if str(WINDOWS_BRIDGE_DIR) not in sys.path:
    sys.path.insert(0, str(WINDOWS_BRIDGE_DIR))

from fr3_action_protocol import (  # noqa: E402
    DEFAULT_SECRET_ENV,
    MAX_BODY_BYTES,
    ReplayCache,
    ProtocolError,
    SCHEMA_VERSION,
    SIGNATURE_HEADER,
    action_sha256,
    load_shared_secret,
    validate_action_envelope,
    verify_signature,
)


PLAN_PATH = "/v1/actions/plan"
HEALTH_PATH = "/health"
MAX_LOCAL_REQUEST_BYTES = 4096
READY_STATUS = "READY_FOR_LOCAL_APPROVAL"
TERMINAL_STATUSES = {"VERIFIED_SUCCESS", "EXECUTED_WITH_WARNINGS", "FAULT", "EXPIRED", "SUPERSEDED"}


class RequestFailure(RuntimeError):
    def __init__(self, status: int, code: str, message: str) -> None:
        super().__init__(message)
        self.status = status
        self.code = code


class PlanInputRejected(RuntimeError):
    pass


class PlannerUnavailable(RuntimeError):
    pass


class ExecutionRejected(RuntimeError):
    def __init__(self, code: str, message: str, *, details=None) -> None:
        super().__init__(message)
        self.code = code
        self.details = details or {}


def verification_outcome(report, verification, *, mode="enforce", maximum_target_error=0.1):
    """Accuracy may be report-only; velocity, drift and large errors still block."""
    if mode not in {"enforce", "report-only"}:
        raise ValueError("Unknown verification mode")
    if report.passed:
        return "VERIFIED_SUCCESS"
    hard_failure = (
        not report.settled_velocity_ok or not report.non_commanded_drift_ok
        or any(j.target_error > maximum_target_error for j in report.commanded_joints)
    )
    if mode == "report-only" and not hard_failure:
        return "EXECUTED_WITH_WARNINGS"
    raise ExecutionRejected(
        "VERIFICATION_FAILED", "Measured post-execution joint verification failed.",
        details={"verification": verification, "verification_mode": mode,
                 "moveit_execution_result": "SUCCESS", "verification_hard_failure": hard_failure},
    )


@dataclass
class ExecutionContext:
    trajectory: Any
    start_positions: dict[str, float]
    target_positions: dict[str, float]
    metadata: dict | None = None


@dataclass
class PreparedPlan:
    http_status: int
    response: dict[str, Any]
    execution_context: ExecutionContext | None


@dataclass
class StoredPlan:
    request_id: str
    action_digest: str
    plan_response: dict[str, Any]
    execution_context: ExecutionContext
    created_monotonic: float
    expires_monotonic: float
    expires_unix_ms: int
    status: str = READY_STATUS
    result: dict[str, Any] | None = None


def parse_strict_json(body: bytes) -> Any:
    try:
        text = body.decode("utf-8")
    except UnicodeDecodeError as error:
        raise RequestFailure(400, "INVALID_JSON", "Body must be UTF-8 JSON.") from error

    def reject_constant(value: str) -> None:
        raise ValueError(f"Non-standard numeric constant {value!r} is not permitted.")

    try:
        return json.loads(text, parse_constant=reject_constant)
    except (json.JSONDecodeError, ValueError) as error:
        raise RequestFailure(400, "INVALID_JSON", f"Invalid JSON: {error}") from error


def validate_joint_delta_action_contract(action: Mapping[str, Any]) -> None:
    required_keys = {"action", "deltas", "confidence"}
    allowed_keys = required_keys | {"observation"}
    if not required_keys.issubset(action) or not set(action).issubset(allowed_keys):
        raise PlanInputRejected(
            f"Action keys must contain {sorted(required_keys)} and may include observation."
        )
    if action.get("action") != "joint_deltas":
        raise PlanInputRejected("Only action='joint_deltas' is accepted.")

    confidence = action.get("confidence")
    if isinstance(confidence, bool):
        raise PlanInputRejected("confidence must be numeric, not Boolean.")
    try:
        parsed_confidence = float(confidence)
    except (TypeError, ValueError) as error:
        raise PlanInputRejected("confidence must be numeric.") from error
    if not math.isfinite(parsed_confidence) or not 0.0 <= parsed_confidence <= 1.0:
        raise PlanInputRejected("confidence must be finite and in [0, 1].")

    deltas = action.get("deltas")
    if not isinstance(deltas, dict):
        raise PlanInputRejected("deltas must be an object.")
    moving_deltas = []
    for value in deltas.values():
        if isinstance(value, bool):
            raise PlanInputRejected("Joint deltas must be numbers, not Booleans.")
        try:
            delta = float(value)
        except (TypeError, ValueError) as error:
            raise PlanInputRejected("Joint deltas must be numeric.") from error
        if not math.isfinite(delta):
            raise PlanInputRejected("Joint deltas must be finite.")
        if abs(delta) > 1e-9:
            moving_deltas.append(delta)

    if not moving_deltas:
        raise PlanInputRejected("A plan request must contain a non-zero joint delta.")


def validate_visual_action_contract(action: Mapping[str, Any]) -> None:
    validate_joint_delta_action_contract(action)
    if set(action) != {"action", "deltas", "confidence", "observation"}:
        raise PlanInputRejected(
            "Visual actions must contain exactly action, deltas, confidence, and observation."
        )
    observation = action.get("observation")
    if not isinstance(observation, dict) or set(observation) != {"red_marker_side"}:
        raise PlanInputRejected("observation must contain only red_marker_side.")
    side = observation["red_marker_side"]
    if side not in {"left", "right"}:
        raise PlanInputRejected("Moving actions require red_marker_side left or right.")
    moving_deltas = [float(value) for value in action["deltas"].values() if abs(float(value)) > 1e-9]
    if side == "right" and any(delta <= 0.0 for delta in moving_deltas):
        raise PlanInputRejected("A right observation must map to positive deltas.")
    if side == "left" and any(delta >= 0.0 for delta in moving_deltas):
        raise PlanInputRejected("A left observation must map to negative deltas.")


class RosMoveItPlanner:
    def __init__(
        self,
        args: argparse.Namespace,
        *,
        action_validator: Callable[[Mapping[str, Any]], None] = validate_visual_action_contract,
    ) -> None:
        import rclpy
        from moveit_msgs.msg import MoveItErrorCodes

        from moveit_joint_goal_test import (
            JOINTS,
            MoveItJointGoalClient,
            collect_post_execution_verification,
            error_name,
            make_plan_request,
        )
        from joint_motion_verification import verification_report_payload
        from vla_joint_command_test import (
            build_target,
            decode_action,
            validate_deltas,
            validate_target_limits,
        )

        self.args = args
        self.action_validator = action_validator
        self.rclpy = rclpy
        self.MoveItErrorCodes = MoveItErrorCodes
        self.JOINTS = JOINTS
        self.error_name = error_name
        self.make_plan_request = make_plan_request
        self.collect_post_execution_verification = collect_post_execution_verification
        self.verification_report_payload = verification_report_payload
        self.build_target = build_target
        self.decode_action = decode_action
        self.validate_deltas = validate_deltas
        self.validate_target_limits = validate_target_limits

        rclpy.init()
        self.node = MoveItJointGoalClient(
            args.state_topic,
            args.plan_service,
            args.execute_action if args.enable_local_execution else None,
            node_name="fr3_ai_supervisor",
        )

    def close(self) -> None:
        self.node.destroy_node()
        self.rclpy.shutdown()

    def plan(self, envelope: Mapping[str, Any]) -> PreparedPlan:
        action = envelope["action"]
        try:
            self.action_validator(action)
            deltas, action_label = self.decode_action(action, self.args)
            self.validate_deltas(deltas, self.args)
        except (PlanInputRejected, RuntimeError) as error:
            raise PlanInputRejected(str(error)) from error

        sequence = self.node.joint_state_sequence
        try:
            snapshot = self.node.wait_for_joint_state_snapshot(
                timeout=self.args.state_timeout,
                after_sequence=sequence,
                require_velocities=True,
            )
        except RuntimeError as error:
            raise PlannerUnavailable(str(error)) from error
        current = dict(snapshot.positions)

        try:
            target = self.resolve_target(envelope, current, deltas)
            self.validate_target_limits(target, self.args)
        except RuntimeError as error:
            raise PlanInputRejected(str(error)) from error

        try:
            self.node.wait_for_plan_service(timeout=self.args.server_timeout)
            request = self.make_plan_request(self.node, current, target, self.args)
            response = self.node.plan(request, timeout=self.args.result_timeout)
        except RuntimeError as error:
            raise PlannerUnavailable(str(error)) from error

        error_code = response.error_code.val
        trajectory = response.trajectory.joint_trajectory
        point_count = len(trajectory.points)
        if point_count:
            final_time = trajectory.points[-1].time_from_start
            duration = final_time.sec + final_time.nanosec * 1e-9
        else:
            duration = 0.0

        success = error_code == self.MoveItErrorCodes.SUCCESS and point_count > 0
        payload = {
            "schema_version": SCHEMA_VERSION,
            "request_id": envelope["request_id"],
            "status": "PLAN_READY" if success else "PLAN_REJECTED",
            "plan_only": True,
            "execution_available": False,
            "plan_request_executed": False,
            "remote_execution_available": False,
            "action_sha256": action_sha256(action),
            "action_label": action_label,
            "validated_deltas": deltas,
            "current_joint_positions": current,
            "target_joint_positions": target,
            "planning": {
                "result": self.error_name(error_code),
                "error_code": error_code,
                "point_count": point_count,
                "duration_seconds": duration,
                "joints": list(trajectory.joint_names),
            },
        }
        if success:
            payload.update(self.inspect_trajectory(trajectory, current, target))
        execution_context = None
        if success and self.args.enable_local_execution:
            execution_context = ExecutionContext(
                trajectory=response.trajectory,
                start_positions=current,
                target_positions=target,
            )
        return PreparedPlan(
            http_status=200 if success else 422,
            response=payload,
            execution_context=execution_context,
        )

    def resolve_target(self, envelope, current, deltas):
        """Override for adapters that carry an absolute target and starting pose."""
        return self.build_target(current, deltas)

    def inspect_trajectory(self, trajectory, current, target):
        return {}

    def execute(self, context: ExecutionContext) -> dict[str, Any]:
        if not self.args.enable_local_execution:
            raise ExecutionRejected("EXECUTION_DISABLED", "Local execution is disabled.")

        sequence = self.node.joint_state_sequence
        try:
            snapshot = self.node.wait_for_joint_state_snapshot(
                timeout=self.args.state_timeout,
                after_sequence=sequence,
                require_velocities=True,
            )
        except RuntimeError as error:
            raise ExecutionRejected("JOINT_STATE_UNAVAILABLE", str(error)) from error

        current = snapshot.positions
        drifts = {
            joint: abs(current[joint] - context.start_positions[joint])
            for joint in self.JOINTS
        }
        drift_joint = max(drifts, key=drifts.get)
        maximum_drift = drifts[drift_joint]
        if maximum_drift > self.args.start_state_tolerance:
            raise ExecutionRejected(
                "START_STATE_CHANGED",
                f"{drift_joint} changed by {maximum_drift:.6f} rad since planning; "
                f"limit is {self.args.start_state_tolerance:.6f} rad.",
            )

        start_velocity_joint = max(
            self.JOINTS,
            key=lambda joint: abs(snapshot.velocities[joint]),
        )
        maximum_start_velocity = abs(snapshot.velocities[start_velocity_joint])
        if maximum_start_velocity > self.args.max_start_velocity:
            raise ExecutionRejected(
                "ROBOT_NOT_SETTLED",
                f"{start_velocity_joint} velocity is {maximum_start_velocity:.6f} rad/s; "
                f"limit is {self.args.max_start_velocity:.6f} rad/s.",
            )

        try:
            self.node.wait_for_execute_action(timeout=self.args.server_timeout)
            self.before_execute(context)
            result = self.node.execute(
                context.trajectory,
                timeout=self.args.result_timeout,
            )
        except ExecutionRejected:
            raise
        except RuntimeError as error:
            raise ExecutionRejected("EXECUTION_ERROR", str(error)) from error

        execution_code = result.error_code.val
        execution_name = self.error_name(execution_code)
        if execution_code != self.MoveItErrorCodes.SUCCESS:
            raise ExecutionRejected(
                "MOVEIT_EXECUTION_FAILED",
                f"MoveIt execution returned {execution_name}.",
            )

        try:
            report, final_snapshot = self.collect_post_execution_verification(
                self.node,
                context.start_positions,
                context.target_positions,
                self.args,
            )
        except RuntimeError as error:
            raise ExecutionRejected("VERIFICATION_ERROR", str(error)) from error
        verification = self.verification_report_payload(report)
        verification["measured_joint_positions"] = dict(final_snapshot.positions)
        verification["settled_joint_velocities"] = dict(final_snapshot.velocities)
        mode = getattr(self.args, "verification_mode", "enforce")
        outcome = verification_outcome(
            report, verification, mode=mode, maximum_target_error=self.args.max_delta,
        )

        return {
            "status": outcome,
            "code": outcome,
            "verification_mode": mode,
            "moveit_execution_result": execution_name,
            "maximum_start_state_drift": maximum_drift,
            "maximum_start_state_drift_joint": drift_joint,
            "maximum_start_velocity": maximum_start_velocity,
            "maximum_start_velocity_joint": start_velocity_joint,
            "verification": verification,
        }

    def before_execute(self, context):
        """Optional adapter check immediately before the trajectory is sent."""


class SupervisorCore:
    def __init__(
        self,
        planner: Any,
        secret: bytes,
        *,
        max_request_age_ms: int,
        max_future_skew_ms: int,
        replay_retention_seconds: float,
        execution_enabled: bool = False,
        plan_ttl_seconds: float = 30.0,
        record_retention_seconds: float = 300.0,
    ) -> None:
        self.planner = planner
        self.secret = secret
        self.max_request_age_ms = max_request_age_ms
        self.max_future_skew_ms = max_future_skew_ms
        self.replay_cache = ReplayCache(replay_retention_seconds)
        self.execution_enabled = execution_enabled
        self.plan_ttl_seconds = plan_ttl_seconds
        self.record_retention_seconds = record_retention_seconds
        self.operation_lock = threading.Lock()
        self.record_lock = threading.Lock()
        self.records: dict[str, StoredPlan] = {}

    @property
    def mode(self) -> str:
        return "SUPERVISED_SINGLE_STEP" if self.execution_enabled else "PLAN_ONLY"

    def handle_plan(self, body: bytes, signature: str) -> tuple[int, dict[str, Any]]:
        try:
            verify_signature(body, signature, self.secret)
            envelope = validate_action_envelope(
                parse_strict_json(body),
                now_unix_ms=int(time.time() * 1000),
                max_age_ms=self.max_request_age_ms,
                max_future_skew_ms=self.max_future_skew_ms,
            )
        except ProtocolError as error:
            status = 401 if error.code == "INVALID_SIGNATURE" else 400
            if error.code == "STALE_REQUEST":
                status = 408
            raise RequestFailure(status, error.code, str(error)) from error

        request_id = envelope["request_id"]
        if not self.replay_cache.record_if_new(request_id):
            raise RequestFailure(
                409, "REPLAY_REJECTED", f"request_id {request_id} was already used."
            )

        return self.prepare_validated_plan(envelope)

    def prepare_validated_plan(self, envelope) -> tuple[int, dict[str, Any]]:
        """Internal entry point after a transport has validated its own contract.

        HTTP callers of the legacy supervisor still pass signature, source,
        freshness and replay checks in handle_plan before reaching this method.
        """
        request_id = envelope["request_id"]

        if not self.operation_lock.acquire(blocking=False):
            raise RequestFailure(409, "SUPERVISOR_BUSY", "Another operation is in progress.")
        try:
            try:
                prepared = self.planner.plan(envelope)
            except PlanInputRejected as error:
                raise RequestFailure(422, "ACTION_REJECTED", str(error)) from error
            except PlannerUnavailable as error:
                raise RequestFailure(503, "MOVEIT_UNAVAILABLE", str(error)) from error

            response = dict(prepared.response)
            response["local_approval_available"] = bool(
                self.execution_enabled and prepared.execution_context is not None
            )
            if self.execution_enabled and prepared.execution_context is not None:
                now_monotonic = time.monotonic()
                expires_unix_ms = int((time.time() + self.plan_ttl_seconds) * 1000)
                record = StoredPlan(
                    request_id=request_id,
                    action_digest=response["action_sha256"],
                    plan_response=dict(response),
                    execution_context=prepared.execution_context,
                    created_monotonic=now_monotonic,
                    expires_monotonic=now_monotonic + self.plan_ttl_seconds,
                    expires_unix_ms=expires_unix_ms,
                )
                with self.record_lock:
                    self._prune_records_locked(now_monotonic)
                    self.on_store_plan(record, envelope)
                    for existing in self.records.values():
                        if existing.status == READY_STATUS:
                            existing.status = "SUPERSEDED"
                            existing.result = {
                                "status": "SUPERSEDED",
                                "code": "NEWER_PLAN_CREATED",
                            }
                    self.records[request_id] = record
                response["approval_status"] = READY_STATUS
                response["plan_expires_at_unix_ms"] = record.expires_unix_ms
            return prepared.http_status, response
        finally:
            self.operation_lock.release()

    def on_store_plan(self, record, envelope):
        """Adapter hook under the record lock, before a plan becomes visible."""

    def execute_context(self, context, request_id):
        return self.planner.execute(context)

    def handle_local(self, request: Any) -> tuple[int, dict[str, Any]]:
        if not isinstance(request, dict):
            raise RequestFailure(400, "INVALID_LOCAL_REQUEST", "Request must be an object.")
        operation = request.get("operation")
        if operation == "status":
            if set(request) != {"operation", "request_id"}:
                raise RequestFailure(400, "INVALID_LOCAL_REQUEST", "Invalid status request.")
            return 200, self._status_payload(str(request["request_id"]))
        if operation == "execute":
            if set(request) != {"operation", "request_id", "action_sha256"}:
                raise RequestFailure(400, "INVALID_LOCAL_REQUEST", "Invalid execute request.")
            return self._execute_stored_plan(
                str(request["request_id"]),
                str(request["action_sha256"]),
            )
        raise RequestFailure(400, "INVALID_OPERATION", "operation must be status or execute.")

    def _execute_stored_plan(
        self,
        request_id: str,
        supplied_digest: str,
    ) -> tuple[int, dict[str, Any]]:
        if not self.execution_enabled:
            raise RequestFailure(403, "EXECUTION_DISABLED", "Local execution is disabled.")
        if not self.operation_lock.acquire(blocking=False):
            raise RequestFailure(409, "SUPERVISOR_BUSY", "Another operation is in progress.")

        record = None
        try:
            with self.record_lock:
                record = self._get_record_locked(request_id)
                self._expire_record_locked(record)
                if record.status != READY_STATUS:
                    raise RequestFailure(
                        409,
                        "PLAN_NOT_READY",
                        f"Plan status is {record.status}; it cannot be executed.",
                    )
                if supplied_digest != record.action_digest:
                    raise RequestFailure(
                        409,
                        "ACTION_HASH_MISMATCH",
                        "The supplied action hash does not match the frozen plan.",
                    )
                record.status = "EXECUTING"
                record.result = {
                    "status": "EXECUTING",
                    "code": "PLAN_CONSUMED_BEFORE_EXECUTION",
                }

            try:
                result = self.execute_context(record.execution_context, request_id)
            except ExecutionRejected as error:
                result = {
                    "status": "FAULT",
                    "code": error.code,
                    "message": str(error),
                    **error.details,
                }
            except Exception as error:
                result = {
                    "status": "FAULT",
                    "code": "UNEXPECTED_EXECUTION_ERROR",
                    "message": type(error).__name__,
                }

            with self.record_lock:
                record.status = result["status"]
                record.result = result
            return (200 if result["status"] in {"VERIFIED_SUCCESS", "EXECUTED_WITH_WARNINGS"} else 500), {
                "schema_version": SCHEMA_VERSION,
                "request_id": request_id,
                "action_sha256": record.action_digest,
                **result,
            }
        finally:
            self.operation_lock.release()

    def _status_payload(self, request_id: str) -> dict[str, Any]:
        with self.record_lock:
            record = self._get_record_locked(request_id)
            self._expire_record_locked(record)
            plan = record.plan_response
            return {
                "schema_version": SCHEMA_VERSION,
                "request_id": request_id,
                "status": record.status,
                "action_sha256": record.action_digest,
                "plan_expires_at_unix_ms": record.expires_unix_ms,
                "validated_deltas": plan["validated_deltas"],
                "current_joint_positions": plan["current_joint_positions"],
                "target_joint_positions": plan["target_joint_positions"],
                "planning": plan["planning"],
                "result": record.result,
            }

    def _get_record_locked(self, request_id: str) -> StoredPlan:
        record = self.records.get(request_id)
        if record is None:
            raise RequestFailure(404, "PLAN_NOT_FOUND", "No stored plan has that request ID.")
        return record

    def _expire_record_locked(self, record: StoredPlan) -> None:
        if record.status == READY_STATUS and time.monotonic() > record.expires_monotonic:
            record.status = "EXPIRED"
            record.result = {"status": "EXPIRED", "code": "PLAN_TTL_EXCEEDED"}

    def _prune_records_locked(self, now_monotonic: float) -> None:
        cutoff = now_monotonic - self.record_retention_seconds
        stale_ids = [
            request_id
            for request_id, record in self.records.items()
            if record.created_monotonic < cutoff and record.status in TERMINAL_STATUSES
        ]
        for request_id in stale_ids:
            del self.records[request_id]


class SupervisorHttpServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(
        self,
        server_address: tuple[str, int],
        core: SupervisorCore,
        allowed_clients: set[str],
    ) -> None:
        super().__init__(server_address, SupervisorRequestHandler)
        self.core = core
        self.allowed_clients = allowed_clients


class SupervisorRequestHandler(BaseHTTPRequestHandler):
    server: SupervisorHttpServer

    def do_GET(self) -> None:
        if self.path != HEALTH_PATH:
            self._send_json(404, {"status": "error", "code": "NOT_FOUND"})
            return
        self._send_json(
            200,
            {
                "status": "ok",
                "mode": self.server.core.mode,
                "schema_version": SCHEMA_VERSION,
                "plan_endpoint": PLAN_PATH,
                "execution_available": False,
                "remote_execution_available": False,
                "local_approval_available": self.server.core.execution_enabled,
            },
        )

    def do_POST(self) -> None:
        if self.path != PLAN_PATH:
            self._send_json(
                404,
                {
                    "status": "error",
                    "code": "NOT_FOUND",
                    "message": "No remote execution endpoint exists.",
                },
            )
            return
        if self.client_address[0] not in self.server.allowed_clients:
            self._send_json(403, {"status": "error", "code": "CLIENT_NOT_ALLOWED"})
            return
        if not self.headers.get("Content-Type", "").lower().startswith(
            "application/json"
        ):
            self._send_json(415, {"status": "error", "code": "JSON_REQUIRED"})
            return

        try:
            content_length = int(self.headers.get("Content-Length", ""))
        except ValueError:
            self._send_json(411, {"status": "error", "code": "LENGTH_REQUIRED"})
            return
        if content_length <= 0 or content_length > MAX_BODY_BYTES:
            self._send_json(413, {"status": "error", "code": "BODY_SIZE_REJECTED"})
            return

        body = self.rfile.read(content_length)
        signature = self.headers.get(SIGNATURE_HEADER, "")
        try:
            status, response = self.server.core.handle_plan(body, signature)
        except RequestFailure as error:
            self._send_json(
                error.status,
                {"status": "error", "code": error.code, "message": str(error)},
            )
            return
        except Exception as error:
            self._send_json(
                500,
                {
                    "status": "error",
                    "code": "INTERNAL_ERROR",
                    "message": type(error).__name__,
                },
            )
            return
        self._send_json(status, response)

    def _send_json(self, status: int, payload: Mapping[str, Any]) -> None:
        body = json.dumps(payload, allow_nan=False, indent=2).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, message_format: str, *args: Any) -> None:
        print(f"[HTTP] {self.client_address[0]} {message_format % args}")


class ApprovalUnixServer(socketserver.ThreadingMixIn, socketserver.UnixStreamServer):
    daemon_threads = True

    def __init__(self, socket_path: str, core: SupervisorCore) -> None:
        self.core = core
        super().__init__(socket_path, ApprovalRequestHandler)


class ApprovalRequestHandler(socketserver.StreamRequestHandler):
    server: ApprovalUnixServer

    def handle(self) -> None:
        self.connection.settimeout(5.0)
        request_body = self.rfile.readline(MAX_LOCAL_REQUEST_BYTES + 1)
        if not request_body or len(request_body) > MAX_LOCAL_REQUEST_BYTES:
            self._send(413, {"status": "error", "code": "BODY_SIZE_REJECTED"})
            return
        try:
            request = parse_strict_json(request_body)
            status, response = self.server.core.handle_local(request)
        except RequestFailure as error:
            self._send(
                error.status,
                {"status": "error", "code": error.code, "message": str(error)},
            )
            return
        except Exception as error:
            self._send(
                500,
                {
                    "status": "error",
                    "code": "INTERNAL_ERROR",
                    "message": type(error).__name__,
                },
            )
            return
        self._send(status, response)

    def _send(self, status: int, payload: Mapping[str, Any]) -> None:
        response = {"local_status": status, **payload}
        self.wfile.write(
            json.dumps(response, allow_nan=False, separators=(",", ":")).encode("utf-8")
            + b"\n"
        )


def create_approval_server(socket_path: Path, core: SupervisorCore) -> ApprovalUnixServer:
    socket_path.parent.mkdir(parents=True, exist_ok=True)
    if socket_path.exists() or socket_path.is_symlink():
        metadata = socket_path.lstat()
        if not stat.S_ISSOCK(metadata.st_mode) or metadata.st_uid != os.getuid():
            raise RuntimeError(
                f"Refusing to replace non-owned or non-socket path: {socket_path}"
            )
        socket_path.unlink()

    previous_umask = os.umask(0o177)
    try:
        server = ApprovalUnixServer(str(socket_path), core)
    finally:
        os.umask(previous_umask)
    os.chmod(socket_path, 0o600)
    return server


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Receive authenticated Windows proposals, plan with MoveIt, and "
            "optionally permit explicit local Ubuntu single-step execution."
        )
    )
    parser.add_argument("--bind-host", default="10.10.10.1")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--allowed-client", action="append", default=[])
    parser.add_argument("--shared-secret-env", default=DEFAULT_SECRET_ENV)
    parser.add_argument("--max-request-age", type=float, default=10.0)
    parser.add_argument("--max-future-skew", type=float, default=2.0)
    parser.add_argument("--replay-retention", type=float, default=300.0)

    parser.add_argument("--enable-local-execution", action="store_true")
    parser.add_argument(
        "--approval-socket",
        type=Path,
        default=Path("/tmp/fr3_ai_supervisor_approval.sock"),
    )
    parser.add_argument("--plan-ttl", type=float, default=30.0)
    parser.add_argument("--record-retention", type=float, default=300.0)
    parser.add_argument("--start-state-tolerance", type=float, default=0.01)
    parser.add_argument("--max-start-velocity", type=float, default=0.01)

    add_motion_arguments(parser)
    return parser.parse_args()


def add_motion_arguments(parser) -> None:
    """MoveIt and verification options shared with the Cosmos adapter."""
    parser.add_argument("--min-confidence", type=float, default=0.70)
    parser.add_argument("--max-delta", type=float, default=0.03)
    parser.add_argument("--max-total-delta", type=float, default=0.03)
    parser.add_argument("--max-moving-joints", type=int, default=1)
    parser.add_argument("--joint-limit-margin", type=float, default=0.02)

    parser.add_argument("--state-topic", default="/joint_states")
    parser.add_argument("--state-timeout", type=float, default=3.0)
    parser.add_argument("--plan-service", default="/plan_kinematic_path")
    parser.add_argument("--execute-action", default="/execute_trajectory")
    parser.add_argument("--group", default="fr3_arm")
    parser.add_argument("--planning-frame", default="base")
    parser.add_argument("--pipeline", default="")
    parser.add_argument("--planner", default="")
    parser.add_argument("--planning-attempts", type=int, default=5)
    parser.add_argument("--planning-time", type=float, default=5.0)
    parser.add_argument("--velocity-scaling", type=float, default=0.03)
    parser.add_argument("--acceleration-scaling", type=float, default=0.03)
    parser.add_argument("--goal-tolerance", type=float, default=0.002)
    parser.add_argument("--workspace-size", type=float, default=2.0)
    parser.add_argument("--server-timeout", type=float, default=10.0)
    parser.add_argument("--result-timeout", type=float, default=60.0)

    parser.add_argument("--settle-time", type=float, default=1.0)
    parser.add_argument("--verification-timeout", type=float, default=2.0)
    parser.add_argument("--verification-tolerance", type=float, default=0.01)
    parser.add_argument(
        "--joint7-verification-tolerance", type=float, default=0.01
    )
    parser.add_argument("--non-commanded-drift-limit", type=float, default=0.01)
    parser.add_argument("--max-settled-velocity", type=float, default=0.01)
    parser.add_argument("--minimum-measured-delta", type=float, default=0.001)


def validate_args(args: argparse.Namespace) -> set[str]:
    if not 1 <= args.port <= 65535:
        raise RuntimeError("--port must be between 1 and 65535.")
    try:
        ipaddress.ip_address(args.bind_host)
    except ValueError as error:
        raise RuntimeError("--bind-host must be an IP address.") from error

    clients = set(args.allowed_client or ["10.10.10.2", "127.0.0.1"])
    for client in clients:
        try:
            ipaddress.ip_address(client)
        except ValueError as error:
            raise RuntimeError(f"Invalid --allowed-client address: {client}") from error

    positive_values = {
        "max_request_age": args.max_request_age,
        "max_future_skew": args.max_future_skew,
        "replay_retention": args.replay_retention,
        "plan_ttl": args.plan_ttl,
        "record_retention": args.record_retention,
        "start_state_tolerance": args.start_state_tolerance,
        "max_start_velocity": args.max_start_velocity,
        "max_delta": args.max_delta,
        "max_total_delta": args.max_total_delta,
        "joint_limit_margin": args.joint_limit_margin,
        "state_timeout": args.state_timeout,
        "planning_time": args.planning_time,
        "velocity_scaling": args.velocity_scaling,
        "acceleration_scaling": args.acceleration_scaling,
        "goal_tolerance": args.goal_tolerance,
        "workspace_size": args.workspace_size,
        "server_timeout": args.server_timeout,
        "result_timeout": args.result_timeout,
        "verification_timeout": args.verification_timeout,
        "verification_tolerance": args.verification_tolerance,
        "joint7_verification_tolerance": args.joint7_verification_tolerance,
        "non_commanded_drift_limit": args.non_commanded_drift_limit,
        "max_settled_velocity": args.max_settled_velocity,
        "minimum_measured_delta": args.minimum_measured_delta,
    }
    for name, value in positive_values.items():
        if not math.isfinite(value) or value <= 0.0:
            raise RuntimeError(f"--{name.replace('_', '-')} must be finite and positive.")
    if not math.isfinite(args.settle_time) or args.settle_time < 0.0:
        raise RuntimeError("--settle-time must be finite and non-negative.")
    if not 0.0 < args.min_confidence <= 1.0:
        raise RuntimeError("--min-confidence must be in (0, 1].")
    if not 0.0 < args.velocity_scaling <= 1.0:
        raise RuntimeError("--velocity-scaling must be in (0, 1].")
    if not 0.0 < args.acceleration_scaling <= 1.0:
        raise RuntimeError("--acceleration-scaling must be in (0, 1].")
    if args.max_moving_joints <= 0 or args.planning_attempts <= 0:
        raise RuntimeError("Moving-joint and planning-attempt limits must be positive.")
    return clients


def main() -> int:
    args = parse_args()
    try:
        allowed_clients = validate_args(args)
        secret = load_shared_secret(args.shared_secret_env)
    except (RuntimeError, ProtocolError) as error:
        print(f"Configuration error: {error}", file=sys.stderr)
        return 2

    planner = None
    http_server = None
    approval_server = None
    approval_thread = None
    try:
        planner = RosMoveItPlanner(args)
        core = SupervisorCore(
            planner,
            secret,
            max_request_age_ms=int(args.max_request_age * 1000),
            max_future_skew_ms=int(args.max_future_skew * 1000),
            replay_retention_seconds=args.replay_retention,
            execution_enabled=args.enable_local_execution,
            plan_ttl_seconds=args.plan_ttl,
            record_retention_seconds=args.record_retention,
        )
        http_server = SupervisorHttpServer(
            (args.bind_host, args.port), core, allowed_clients
        )

        if args.enable_local_execution:
            approval_server = create_approval_server(args.approval_socket, core)
            approval_thread = threading.Thread(
                target=approval_server.serve_forever,
                kwargs={"poll_interval": 0.2},
                daemon=True,
            )
            approval_thread.start()

        print("FR3 AI supervisor started.")
        print(f"Health: http://{args.bind_host}:{args.port}{HEALTH_PATH}")
        print(f"Plan:   http://{args.bind_host}:{args.port}{PLAN_PATH}")
        print(f"Allowed clients: {sorted(allowed_clients)}")
        print("Remote execution: DISABLED (no HTTP execution endpoint exists).")
        if args.enable_local_execution:
            print("MODE: SUPERVISED SINGLE STEP")
            print(f"Local approval socket: {args.approval_socket} (mode 0600)")
            print(f"Frozen plan lifetime: {args.plan_ttl:.1f} s")
        else:
            print("MODE: PLAN ONLY. This process cannot execute robot motion.")

        http_server.serve_forever(poll_interval=0.2)
        return 0
    except KeyboardInterrupt:
        print("\nSupervisor stopped.")
        return 130
    except Exception as error:
        print(f"Supervisor error: {type(error).__name__}: {error}", file=sys.stderr)
        return 1
    finally:
        if approval_server is not None:
            approval_server.shutdown()
            approval_server.server_close()
        if approval_thread is not None:
            approval_thread.join(timeout=2.0)
        if args.enable_local_execution:
            try:
                if args.approval_socket.is_socket():
                    args.approval_socket.unlink()
            except OSError:
                pass
        if http_server is not None:
            http_server.server_close()
        if planner is not None:
            planner.close()


if __name__ == "__main__":
    sys.exit(main())
