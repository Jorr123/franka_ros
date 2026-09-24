#!/usr/bin/env python3
"""Bounded, authenticated execution-session supervisor for the FR3."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import ipaddress
import json
import math
import os
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import sys
import threading
import time
from typing import Any, Callable, Mapping
import uuid


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
WINDOWS_BRIDGE_DIR = WORKSPACE_ROOT / "windows_vlm_bridge"
if str(WINDOWS_BRIDGE_DIR) not in sys.path:
    sys.path.insert(0, str(WINDOWS_BRIDGE_DIR))

from fr3_action_protocol import (  # noqa: E402
    DEFAULT_SECRET_ENV,
    MAX_BODY_BYTES,
    ProtocolError,
    ReplayCache,
    SCHEMA_VERSION,
    SIGNATURE_HEADER,
    action_sha256,
    load_shared_secret,
    validate_session_arm_envelope,
    validate_session_step_envelope,
    validate_session_stop_envelope,
    verify_signature,
)
from fr3_ai_supervisor import (  # noqa: E402
    ExecutionRejected,
    PlanInputRejected,
    PlannerUnavailable,
    RequestFailure,
    RosMoveItPlanner,
    create_approval_server,
    parse_strict_json,
    validate_joint_delta_action_contract,
)


HEALTH_PATH = "/health"
SESSION_ARM_PATH = "/v1/sessions/arm"
SESSION_STEP_PATH = "/v1/sessions/step"
SESSION_STOP_PATH = "/v1/sessions/stop"
JOINTS = tuple(f"fr3_joint{index}" for index in range(1, 8))
ACTIVE_STATUSES = {"ARMED", "STEP_IN_PROGRESS"}
TERMINAL_STATUSES = {"STOPPED", "DISARMED", "COMPLETED", "EXPIRED", "FAULT"}

ABSOLUTE_MAX_STEPS = 10
ABSOLUTE_MAX_DURATION_SECONDS = 300.0
ABSOLUTE_MAX_CUMULATIVE_DELTA = 0.30
ABSOLUTE_MAX_ACTION_DELTA = 0.10


@dataclass(frozen=True)
class SessionLimits:
    allowed_joints: tuple[str, ...]
    max_moving_joints: int
    max_steps: int
    max_duration_seconds: float
    max_delta: float
    max_total_delta: float
    max_cumulative_delta: float


@dataclass
class ActiveSession:
    session_id: str
    limits: SessionLimits
    created_monotonic: float
    expires_monotonic: float
    created_unix_ms: int
    expires_unix_ms: int
    status: str = "ARMED"
    next_sequence: int = 1
    completed_steps: int = 0
    cumulative_absolute_delta: float = 0.0
    current_request_id: str | None = None
    stop_requested: bool = False
    terminal_code: str | None = None
    last_result: dict[str, Any] | None = None


class SessionAuditLog:
    def __init__(self, directory: Path) -> None:
        directory.mkdir(parents=True, exist_ok=True)
        os.chmod(directory, 0o700)
        self.directory = directory
        self._lock = threading.Lock()

    def write(self, event: Mapping[str, Any]) -> None:
        session_id = str(event.get("session_id", ""))
        try:
            parsed = uuid.UUID(session_id)
        except ValueError as error:
            raise RuntimeError("Audit event has an invalid session ID.") from error
        if parsed.version != 4 or str(parsed) != session_id.lower():
            raise RuntimeError("Audit event session ID must be a canonical UUIDv4.")

        payload = {
            "timestamp_unix_ms": int(time.time() * 1000),
            **event,
        }
        line = (
            json.dumps(
                payload,
                allow_nan=False,
                ensure_ascii=True,
                separators=(",", ":"),
                sort_keys=True,
            )
            + "\n"
        ).encode("utf-8")
        path = self.directory / f"session_{session_id}.jsonl"
        with self._lock:
            descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o600)
            try:
                os.write(descriptor, line)
                os.fsync(descriptor)
            finally:
                os.close(descriptor)


class BoundedSessionCore:
    def __init__(
        self,
        planner: Any,
        secret: bytes,
        *,
        max_request_age_ms: int,
        max_future_skew_ms: int,
        replay_retention_seconds: float,
        server_limits: SessionLimits,
        minimum_confidence: float,
        remote_arm_joints: tuple[str, ...] | None = None,
        event_sink: Callable[[Mapping[str, Any]], None] | None = None,
    ) -> None:
        self.planner = planner
        self.secret = secret
        self.max_request_age_ms = max_request_age_ms
        self.max_future_skew_ms = max_future_skew_ms
        self.replay_cache = ReplayCache(replay_retention_seconds)
        self.server_limits = server_limits
        self.minimum_confidence = minimum_confidence
        self.remote_arm_joints = remote_arm_joints
        self.event_sink = event_sink or (lambda _event: None)
        self.operation_lock = threading.Lock()
        self.session_lock = threading.Lock()
        self.active_session: ActiveSession | None = None

    def health_payload(self) -> dict[str, Any]:
        with self.session_lock:
            session = self.active_session
            if session is not None:
                self._expire_if_needed_locked(session)
                session_payload = self._session_payload_locked(session)
            else:
                session_payload = None
        return {
            "status": "ok",
            "mode": "BOUNDED_SESSION_EXECUTION",
            "schema_version": SCHEMA_VERSION,
            "multi_joint_bounded_sessions": True,
            "session_arm_endpoint": (
                SESSION_ARM_PATH if self.remote_arm_joints is not None else None
            ),
            "session_step_endpoint": SESSION_STEP_PATH,
            "session_stop_endpoint": SESSION_STOP_PATH,
            "remote_direct_execution_available": False,
            "remote_session_arm_available": self.remote_arm_joints is not None,
            "remote_arm_policy": (
                {
                    **self._limits_payload(self.server_limits),
                    "allowed_joints": list(self.remote_arm_joints),
                }
                if self.remote_arm_joints is not None
                else None
            ),
            "session_step_available": True,
            "session_armed": bool(
                session_payload and session_payload["status"] in ACTIVE_STATUSES
            ),
            "session": session_payload,
        }

    def handle_local(self, request: Any) -> tuple[int, dict[str, Any]]:
        if not isinstance(request, dict):
            raise RequestFailure(400, "INVALID_LOCAL_REQUEST", "Request must be an object.")
        operation = request.get("operation")
        if operation == "arm_session":
            expected = {
                "operation",
                "session_id",
                "allowed_joints",
                "max_moving_joints",
                "max_steps",
                "max_duration_seconds",
                "max_delta",
                "max_total_delta",
                "max_cumulative_delta",
            }
            if set(request) != expected:
                raise RequestFailure(400, "INVALID_LOCAL_REQUEST", "Invalid arm request.")
            return self._arm_session(request, source="local_operator")
        if operation in {"session_status", "disarm_session"}:
            if set(request) not in ({"operation"}, {"operation", "session_id"}):
                raise RequestFailure(400, "INVALID_LOCAL_REQUEST", "Invalid session request.")
            if "session_id" in request:
                session_id = self._parse_uuid4(request["session_id"], "session_id")
            else:
                with self.session_lock:
                    session_id = self._get_active_session_locked().session_id
            if operation == "session_status":
                with self.session_lock:
                    session = self._get_session_locked(session_id)
                    self._expire_if_needed_locked(session)
                    return 200, self._session_payload_locked(session)
            return self._disarm_session(session_id, source="local_operator")
        raise RequestFailure(
            400,
            "INVALID_OPERATION",
            "operation must be arm_session, session_status, or disarm_session.",
        )

    def handle_remote_arm(self, body: bytes, signature: str) -> tuple[int, dict[str, Any]]:
        if self.remote_arm_joints is None:
            raise RequestFailure(404, "REMOTE_ARM_DISABLED", "Remote session arm is disabled.")
        envelope = self._authenticate_arm(body, signature)
        request_id = envelope["request_id"]
        if not self.replay_cache.record_if_new(request_id):
            raise RequestFailure(409, "REPLAY_REJECTED", "request_id was already used.")
        if not self.operation_lock.acquire(blocking=False):
            raise RequestFailure(409, "SESSION_BUSY", "A session operation is in progress.")
        try:
            limits = envelope["limits"]
            if not set(limits["allowed_joints"]).issubset(self.remote_arm_joints):
                raise RequestFailure(
                    422,
                    "REMOTE_JOINT_REJECTED",
                    f"Remote arm is restricted to {list(self.remote_arm_joints)}.",
                )
            status, session = self._arm_session(
                {
                    "session_id": envelope["session_id"],
                    "allowed_joints": limits["allowed_joints"],
                    "max_moving_joints": limits["max_moving_joints"],
                    "max_steps": limits["max_steps"],
                    "max_duration_seconds": limits["max_duration_seconds"],
                    "max_delta": limits["max_delta"],
                    "max_total_delta": limits["max_total_delta"],
                    "max_cumulative_delta": limits["max_cumulative_delta"],
                },
                source="authenticated_windows_arm",
            )
            return status, {
                "schema_version": SCHEMA_VERSION,
                "status": "SESSION_ARMED",
                "request_id": request_id,
                "session": session,
            }
        finally:
            self.operation_lock.release()

    def handle_step(self, body: bytes, signature: str) -> tuple[int, dict[str, Any]]:
        envelope = self._authenticate_step(body, signature)
        request_id = envelope["request_id"]
        if not self.replay_cache.record_if_new(request_id):
            raise RequestFailure(409, "REPLAY_REJECTED", "request_id was already used.")
        if not self.operation_lock.acquire(blocking=False):
            raise RequestFailure(409, "SESSION_BUSY", "A session step is already in progress.")

        session: ActiveSession | None = None
        requested_deltas: dict[str, float] = {}
        step_total = 0.0
        try:
            with self.session_lock:
                session = self._get_session_locked(envelope["session_id"])
                self._expire_if_needed_locked(session)
                if session.status != "ARMED":
                    raise RequestFailure(
                        409,
                        "SESSION_NOT_ARMED",
                        f"Session status is {session.status}.",
                    )
                if envelope["sequence"] != session.next_sequence:
                    self._fault_locked(
                        session,
                        "SEQUENCE_MISMATCH",
                        f"Expected sequence {session.next_sequence}, got {envelope['sequence']}.",
                    )
                    raise RequestFailure(409, "SEQUENCE_MISMATCH", "Session sequence mismatch.")
                try:
                    requested_deltas, step_total = self._validate_session_action(
                        envelope["action"], session
                    )
                except PlanInputRejected as error:
                    self._fault_locked(session, "ACTION_REJECTED", str(error))
                    raise RequestFailure(422, "ACTION_REJECTED", str(error)) from error

                session.status = "STEP_IN_PROGRESS"
                session.current_request_id = request_id
                self._write_event(
                    {
                        "event": "STEP_ACCEPTED",
                        "session_id": session.session_id,
                        "sequence": envelope["sequence"],
                        "request_id": request_id,
                        "action_sha256": action_sha256(envelope["action"]),
                        "deltas": requested_deltas,
                        "step_total_delta": step_total,
                    }
                )

            try:
                prepared = self.planner.plan(envelope)
            except PlanInputRejected as error:
                return self._step_fault(session, 422, "ACTION_REJECTED", str(error))
            except PlannerUnavailable as error:
                return self._step_fault(session, 503, "MOVEIT_UNAVAILABLE", str(error))
            except Exception as error:
                return self._step_fault(
                    session,
                    500,
                    "UNEXPECTED_PLANNING_ERROR",
                    type(error).__name__,
                )

            if prepared.http_status != 200 or prepared.execution_context is None:
                return self._step_fault(
                    session,
                    422,
                    "PLAN_REJECTED",
                    f"MoveIt plan status was {prepared.response.get('status')}.",
                )

            with self.session_lock:
                self._expire_if_needed_locked(session)
                if session.stop_requested or session.status != "STEP_IN_PROGRESS":
                    if session.status == "STEP_IN_PROGRESS":
                        if session.terminal_code == "SESSION_DURATION_EXCEEDED":
                            session.status = "EXPIRED"
                        else:
                            session.status = "STOPPED"
                            session.terminal_code = "STOP_REQUESTED_BEFORE_EXECUTION"
                    self._write_event(
                        {
                            "event": "STEP_CANCELLED_BEFORE_EXECUTION",
                            "session_id": session.session_id,
                            "sequence": envelope["sequence"],
                            "request_id": request_id,
                            "status": session.status,
                        }
                    )
                    raise RequestFailure(
                        409,
                        "SESSION_STOPPED",
                        "Session stopped or expired before trajectory execution.",
                    )
                self._write_event(
                    {
                        "event": "PLAN_READY",
                        "session_id": session.session_id,
                        "sequence": envelope["sequence"],
                        "request_id": request_id,
                        "planning": prepared.response.get("planning"),
                    }
                )

            try:
                execution_result = self.planner.execute(prepared.execution_context)
            except ExecutionRejected as error:
                return self._step_fault(session, 500, error.code, str(error))
            except Exception as error:
                return self._step_fault(
                    session,
                    500,
                    "UNEXPECTED_EXECUTION_ERROR",
                    type(error).__name__,
                )

            try:
                measured_motion = self._validate_verified_execution(
                    execution_result,
                    prepared.response,
                    session,
                    requested_deltas,
                )
            except RuntimeError as error:
                return self._step_fault(
                    session,
                    500,
                    "INVALID_VERIFICATION_RESULT",
                    str(error),
                )

            with self.session_lock:
                session.completed_steps += 1
                session.cumulative_absolute_delta += step_total
                session.next_sequence += 1
                session.current_request_id = None
                session.last_result = {
                    "status": "VERIFIED_SUCCESS",
                    "sequence": envelope["sequence"],
                    "request_id": request_id,
                }
                if session.stop_requested:
                    session.status = "STOPPED"
                    session.terminal_code = "STOP_REQUESTED_DURING_EXECUTION"
                elif session.completed_steps >= session.limits.max_steps:
                    session.status = "COMPLETED"
                    session.terminal_code = "MAX_STEPS_REACHED"
                elif time.monotonic() >= session.expires_monotonic:
                    session.status = "EXPIRED"
                    session.terminal_code = "SESSION_DURATION_EXCEEDED"
                else:
                    session.status = "ARMED"
                session_payload = self._session_payload_locked(session)
                self._write_event(
                    {
                        "event": "STEP_VERIFIED",
                        "session_id": session.session_id,
                        "sequence": envelope["sequence"],
                        "request_id": request_id,
                        "session_status": session.status,
                        "completed_steps": session.completed_steps,
                        "cumulative_absolute_delta": session.cumulative_absolute_delta,
                        "measured_motion": measured_motion,
                        "execution": execution_result,
                    }
                )

            response = {
                "schema_version": SCHEMA_VERSION,
                "status": "VERIFIED_SUCCESS",
                "request_id": request_id,
                "action_sha256": action_sha256(envelope["action"]),
                "session_id": session.session_id,
                "sequence": envelope["sequence"],
                "validated_deltas": prepared.response.get("validated_deltas"),
                "current_joint_positions": prepared.response.get("current_joint_positions"),
                "target_joint_positions": prepared.response.get("target_joint_positions"),
                "planning": prepared.response.get("planning"),
                "execution": execution_result,
                "session": session_payload,
            }
            return 200, response
        finally:
            self.operation_lock.release()

    def handle_stop(self, body: bytes, signature: str) -> tuple[int, dict[str, Any]]:
        envelope = self._authenticate_stop(body, signature)
        request_id = envelope["request_id"]
        if not self.replay_cache.record_if_new(request_id):
            raise RequestFailure(409, "REPLAY_REJECTED", "request_id was already used.")
        status, response = self._disarm_session(
            envelope["session_id"], source="authenticated_windows_stop"
        )
        return status, {
            "schema_version": SCHEMA_VERSION,
            "request_id": request_id,
            **response,
        }

    def _arm_session(
        self, request: Mapping[str, Any], *, source: str
    ) -> tuple[int, dict[str, Any]]:
        session_id = self._parse_uuid4(request["session_id"], "session_id")
        requested_joints = request["allowed_joints"]
        if not isinstance(requested_joints, (list, tuple)) or not requested_joints:
            raise RequestFailure(
                422, "INVALID_ALLOWED_JOINTS", "allowed_joints must be non-empty."
            )
        if any(not isinstance(joint, str) or joint not in JOINTS for joint in requested_joints):
            raise RequestFailure(
                422, "INVALID_ALLOWED_JOINTS", "Unknown FR3 arm joint."
            )
        if len(set(requested_joints)) != len(requested_joints):
            raise RequestFailure(
                422, "INVALID_ALLOWED_JOINTS", "allowed_joints contains duplicates."
            )
        allowed_joints = tuple(joint for joint in JOINTS if joint in set(requested_joints))
        if list(requested_joints) != list(allowed_joints):
            raise RequestFailure(
                422,
                "INVALID_ALLOWED_JOINTS",
                "allowed_joints must use canonical FR3 joint order.",
            )
        max_moving_joints = self._parse_positive_int(
            request["max_moving_joints"], "max_moving_joints"
        )
        if max_moving_joints > len(allowed_joints):
            raise RequestFailure(
                422,
                "SESSION_LIMIT_REJECTED",
                "max_moving_joints exceeds the allowlist size.",
            )
        max_steps = self._parse_positive_int(request["max_steps"], "max_steps")
        duration = self._parse_positive_float(
            request["max_duration_seconds"], "max_duration_seconds"
        )
        max_delta = self._parse_positive_float(request["max_delta"], "max_delta")
        max_total_delta = self._parse_positive_float(
            request["max_total_delta"], "max_total_delta"
        )
        if max_total_delta > max_delta * max_moving_joints + 1e-12:
            raise RequestFailure(
                422,
                "SESSION_LIMIT_REJECTED",
                "max_total_delta exceeds max_delta times max_moving_joints.",
            )
        max_cumulative = self._parse_positive_float(
            request["max_cumulative_delta"], "max_cumulative_delta"
        )
        if max_cumulative + 1e-12 < max_total_delta:
            raise RequestFailure(
                422,
                "SESSION_LIMIT_REJECTED",
                "max_cumulative_delta is smaller than max_total_delta.",
            )
        caps = self.server_limits
        if not set(allowed_joints).issubset(caps.allowed_joints):
            raise RequestFailure(
                422, "SESSION_LIMIT_REJECTED", "allowed_joints exceeds server policy."
            )
        if max_moving_joints > caps.max_moving_joints:
            raise RequestFailure(
                422,
                "SESSION_LIMIT_REJECTED",
                "max_moving_joints exceeds server cap.",
            )
        if max_steps > caps.max_steps:
            raise RequestFailure(422, "SESSION_LIMIT_REJECTED", "max_steps exceeds server cap.")
        if duration > caps.max_duration_seconds:
            raise RequestFailure(
                422, "SESSION_LIMIT_REJECTED", "max_duration_seconds exceeds server cap."
            )
        if max_delta > caps.max_delta:
            raise RequestFailure(422, "SESSION_LIMIT_REJECTED", "max_delta exceeds server cap.")
        if max_total_delta > caps.max_total_delta:
            raise RequestFailure(
                422, "SESSION_LIMIT_REJECTED", "max_total_delta exceeds server cap."
            )
        if max_cumulative > caps.max_cumulative_delta:
            raise RequestFailure(
                422,
                "SESSION_LIMIT_REJECTED",
                "max_cumulative_delta exceeds server cap.",
            )

        now_monotonic = time.monotonic()
        now_unix_ms = int(time.time() * 1000)
        limits = SessionLimits(
            allowed_joints=allowed_joints,
            max_moving_joints=max_moving_joints,
            max_steps=max_steps,
            max_duration_seconds=duration,
            max_delta=max_delta,
            max_total_delta=max_total_delta,
            max_cumulative_delta=max_cumulative,
        )
        session = ActiveSession(
            session_id=session_id,
            limits=limits,
            created_monotonic=now_monotonic,
            expires_monotonic=now_monotonic + duration,
            created_unix_ms=now_unix_ms,
            expires_unix_ms=now_unix_ms + int(duration * 1000),
        )
        with self.session_lock:
            existing = self.active_session
            if existing is not None:
                self._expire_if_needed_locked(existing)
                if existing.status in ACTIVE_STATUSES:
                    raise RequestFailure(
                        409,
                        "SESSION_ALREADY_ACTIVE",
                        f"Session {existing.session_id} is still {existing.status}.",
                    )
            self._write_event(
                {
                    "event": "SESSION_ARMED",
                    "session_id": session.session_id,
                    "source": source,
                    "limits": self._limits_payload(limits),
                    "expires_unix_ms": session.expires_unix_ms,
                }
            )
            self.active_session = session
            return 201, self._session_payload_locked(session)

    def _disarm_session(self, session_id: str, *, source: str) -> tuple[int, dict[str, Any]]:
        with self.session_lock:
            session = self._get_session_locked(session_id)
            self._expire_if_needed_locked(session)
            if session.status == "STEP_IN_PROGRESS":
                session.stop_requested = True
                code = "STOP_REQUESTED"
            elif session.status in TERMINAL_STATUSES:
                code = session.terminal_code or session.status
            else:
                session.status = "STOPPED" if source.startswith("authenticated") else "DISARMED"
                session.terminal_code = "SESSION_STOPPED"
                code = session.terminal_code
            self._write_event(
                {
                    "event": "SESSION_STOP_REQUESTED",
                    "session_id": session.session_id,
                    "source": source,
                    "status": session.status,
                    "code": code,
                }
            )
            return 200, {
                "status": code,
                "session": self._session_payload_locked(session),
            }

    def _validate_session_action(
        self, action: Mapping[str, Any], session: ActiveSession
    ) -> tuple[dict[str, float], float]:
        validate_joint_delta_action_contract(action)
        confidence = float(action["confidence"])
        if confidence < self.minimum_confidence:
            raise PlanInputRejected(
                f"confidence {confidence:.3f} is below {self.minimum_confidence:.3f}."
            )
        deltas = action["deltas"]
        moving = {
            str(joint): float(value)
            for joint, value in deltas.items()
            if abs(float(value)) > 1e-9
        }
        if not set(moving).issubset(session.limits.allowed_joints):
            raise PlanInputRejected(
                f"Session permits {list(session.limits.allowed_joints)}; "
                f"got {sorted(moving)}."
            )
        if len(moving) > session.limits.max_moving_joints:
            raise PlanInputRejected("Action exceeds the maximum moving-joint count.")
        for joint, delta in moving.items():
            if abs(delta) > session.limits.max_delta + 1e-12:
                raise PlanInputRejected(
                    f"{joint} delta exceeds the armed per-joint limit."
                )
        step_total = sum(abs(delta) for delta in moving.values())
        if step_total > session.limits.max_total_delta + 1e-12:
            raise PlanInputRejected("Action exceeds the armed per-step total limit.")
        proposed = session.cumulative_absolute_delta + step_total
        if proposed > session.limits.max_cumulative_delta + 1e-12:
            raise PlanInputRejected("Action would exceed the session cumulative limit.")
        return moving, step_total

    def _validate_verified_execution(
        self,
        execution: Any,
        plan: Mapping[str, Any],
        session: ActiveSession,
        requested_deltas: Mapping[str, float],
    ) -> dict[str, dict[str, float | str]]:
        if not isinstance(execution, dict):
            raise RuntimeError("Execution result must be an object.")
        if execution.get("status") != "VERIFIED_SUCCESS":
            raise RuntimeError("Execution did not report VERIFIED_SUCCESS.")
        verification = execution.get("verification")
        if not isinstance(verification, dict) or verification.get("status") != "PASSED":
            raise RuntimeError("Execution receipt is missing a passed verification report.")

        commanded = verification.get("commanded_joints")
        if not isinstance(commanded, list) or not commanded:
            raise RuntimeError("Verification must contain commanded joints.")
        results: dict[str, Mapping[str, Any]] = {}
        for result in commanded:
            if not isinstance(result, dict):
                raise RuntimeError("Commanded-joint verification must be an object.")
            joint = result.get("joint")
            if not isinstance(joint, str) or joint in results:
                raise RuntimeError("Verification contains an invalid or duplicate joint.")
            results[joint] = result
        if set(results) != set(requested_deltas):
            raise RuntimeError(
                "Verified joint set does not exactly match the requested moving joints."
            )
        if verification.get("non_commanded_drift_ok") is not True:
            raise RuntimeError("Non-commanded joint drift verification did not pass.")
        if verification.get("settled_velocity_ok") is not True:
            raise RuntimeError("Settled-velocity verification did not pass.")

        before = plan.get("current_joint_positions")
        target = plan.get("target_joint_positions")
        after = verification.get("measured_joint_positions")
        velocities = verification.get("settled_joint_velocities")
        for label, values in (
            ("current_joint_positions", before),
            ("target_joint_positions", target),
            ("measured_joint_positions", after),
            ("settled_joint_velocities", velocities),
        ):
            if not isinstance(values, dict):
                raise RuntimeError(f"{label} must be an object.")
            for expected_joint in JOINTS:
                self._parse_finite_float(values.get(expected_joint), label)

        measured_motion: dict[str, dict[str, float | str]] = {}
        for joint, requested_delta in requested_deltas.items():
            result = results[joint]
            for field in ("direction_ok", "movement_ok", "target_ok"):
                if result.get(field) is not True:
                    raise RuntimeError(
                        f"Verification field {field} did not pass for {joint}."
                    )
            reported_requested = self._parse_finite_float(
                result.get("requested_delta"), f"{joint}.requested_delta"
            )
            measured_delta = self._parse_finite_float(
                result.get("measured_delta"), f"{joint}.measured_delta"
            )
            reported_error = self._parse_finite_float(
                result.get("target_error"), f"{joint}.target_error"
            )
            if reported_error < 0.0:
                raise RuntimeError("target_error cannot be negative.")
            if abs(reported_requested - requested_delta) > 1e-9:
                raise RuntimeError(
                    f"Verified requested delta does not match the action for {joint}."
                )

            measured_from_positions = float(after[joint]) - float(before[joint])
            target_error_from_positions = abs(float(after[joint]) - float(target[joint]))
            requested_from_positions = float(target[joint]) - float(before[joint])
            if abs(measured_from_positions - measured_delta) > 1e-6:
                raise RuntimeError("Measured delta is inconsistent with measured positions.")
            if abs(target_error_from_positions - reported_error) > 1e-6:
                raise RuntimeError("Target error is inconsistent with measured positions.")
            if abs(requested_from_positions - requested_delta) > 1e-9:
                raise RuntimeError("Planned target is inconsistent with the session action.")
            measured_motion[joint] = {
                "joint": joint,
                "requested_delta": requested_delta,
                "measured_delta": measured_delta,
                "target_error": reported_error,
            }
        return measured_motion

    def _authenticate_step(self, body: bytes, signature: str) -> dict[str, Any]:
        try:
            verify_signature(body, signature, self.secret)
            return validate_session_step_envelope(
                parse_strict_json(body),
                now_unix_ms=int(time.time() * 1000),
                max_age_ms=self.max_request_age_ms,
                max_future_skew_ms=self.max_future_skew_ms,
            )
        except ProtocolError as error:
            raise self._protocol_failure(error) from error

    def _authenticate_arm(self, body: bytes, signature: str) -> dict[str, Any]:
        try:
            verify_signature(body, signature, self.secret)
            return validate_session_arm_envelope(
                parse_strict_json(body),
                now_unix_ms=int(time.time() * 1000),
                max_age_ms=self.max_request_age_ms,
                max_future_skew_ms=self.max_future_skew_ms,
            )
        except ProtocolError as error:
            raise self._protocol_failure(error) from error

    def _authenticate_stop(self, body: bytes, signature: str) -> dict[str, Any]:
        try:
            verify_signature(body, signature, self.secret)
            return validate_session_stop_envelope(
                parse_strict_json(body),
                now_unix_ms=int(time.time() * 1000),
                max_age_ms=self.max_request_age_ms,
                max_future_skew_ms=self.max_future_skew_ms,
            )
        except ProtocolError as error:
            raise self._protocol_failure(error) from error

    @staticmethod
    def _protocol_failure(error: ProtocolError) -> RequestFailure:
        status = 401 if error.code == "INVALID_SIGNATURE" else 400
        if error.code == "STALE_REQUEST":
            status = 408
        return RequestFailure(status, error.code, str(error))

    def _step_fault(
        self,
        session: ActiveSession,
        http_status: int,
        code: str,
        message: str,
    ) -> tuple[int, dict[str, Any]]:
        with self.session_lock:
            self._fault_locked(session, code, message)
            return http_status, {
                "schema_version": SCHEMA_VERSION,
                "status": "FAULT",
                "code": code,
                "message": message,
                "session": self._session_payload_locked(session),
            }

    def _fault_locked(self, session: ActiveSession, code: str, message: str) -> None:
        session.status = "FAULT"
        session.stop_requested = True
        session.terminal_code = code
        session.current_request_id = None
        session.last_result = {"status": "FAULT", "code": code, "message": message}
        self._write_event(
            {
                "event": "SESSION_FAULT",
                "session_id": session.session_id,
                "code": code,
                "message": message,
            }
        )

    def _expire_if_needed_locked(self, session: ActiveSession) -> None:
        if session.status not in ACTIVE_STATUSES:
            return
        if time.monotonic() < session.expires_monotonic:
            return
        session.stop_requested = True
        if session.status == "ARMED":
            session.status = "EXPIRED"
            session.terminal_code = "SESSION_DURATION_EXCEEDED"
            self._write_event(
                {
                    "event": "SESSION_EXPIRED",
                    "session_id": session.session_id,
                    "completed_steps": session.completed_steps,
                }
            )
        else:
            session.terminal_code = "SESSION_DURATION_EXCEEDED"

    def _get_session_locked(self, session_id: str) -> ActiveSession:
        session = self.active_session
        if session is None or session.session_id != session_id:
            raise RequestFailure(404, "SESSION_NOT_FOUND", "No matching session is armed.")
        return session

    def _get_active_session_locked(self) -> ActiveSession:
        session = self.active_session
        if session is None:
            raise RequestFailure(404, "SESSION_NOT_FOUND", "No session exists.")
        return session

    def _session_payload_locked(self, session: ActiveSession) -> dict[str, Any]:
        return {
            "session_id": session.session_id,
            "status": session.status,
            "next_sequence": session.next_sequence,
            "completed_steps": session.completed_steps,
            "cumulative_absolute_delta": session.cumulative_absolute_delta,
            "created_unix_ms": session.created_unix_ms,
            "expires_unix_ms": session.expires_unix_ms,
            "remaining_seconds": max(0.0, session.expires_monotonic - time.monotonic()),
            "current_request_id": session.current_request_id,
            "stop_requested": session.stop_requested,
            "terminal_code": session.terminal_code,
            "limits": self._limits_payload(session.limits),
            "last_result": session.last_result,
        }

    @staticmethod
    def _limits_payload(limits: SessionLimits) -> dict[str, Any]:
        return {
            "allowed_joints": list(limits.allowed_joints),
            "max_moving_joints": limits.max_moving_joints,
            "max_steps": limits.max_steps,
            "max_duration_seconds": limits.max_duration_seconds,
            "max_delta": limits.max_delta,
            "max_total_delta": limits.max_total_delta,
            "max_cumulative_delta": limits.max_cumulative_delta,
        }

    @staticmethod
    def _parse_uuid4(value: Any, field: str) -> str:
        if not isinstance(value, str):
            raise RequestFailure(400, "INVALID_SESSION_ID", f"{field} must be a UUID.")
        try:
            parsed = uuid.UUID(value)
        except ValueError as error:
            raise RequestFailure(400, "INVALID_SESSION_ID", f"{field} must be a UUID.") from error
        if parsed.version != 4 or str(parsed) != value.lower():
            raise RequestFailure(400, "INVALID_SESSION_ID", f"{field} must be UUIDv4.")
        return value

    @staticmethod
    def _parse_positive_int(value: Any, field: str) -> int:
        if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
            raise RequestFailure(422, "INVALID_SESSION_LIMIT", f"{field} must be positive.")
        return value

    @staticmethod
    def _parse_positive_float(value: Any, field: str) -> float:
        if isinstance(value, bool):
            raise RequestFailure(422, "INVALID_SESSION_LIMIT", f"{field} must be numeric.")
        try:
            parsed = float(value)
        except (TypeError, ValueError) as error:
            raise RequestFailure(
                422, "INVALID_SESSION_LIMIT", f"{field} must be numeric."
            ) from error
        if not math.isfinite(parsed) or parsed <= 0.0:
            raise RequestFailure(422, "INVALID_SESSION_LIMIT", f"{field} must be positive.")
        return parsed

    @staticmethod
    def _parse_finite_float(value: Any, field: str) -> float:
        if isinstance(value, bool):
            raise RuntimeError(f"{field} must be numeric.")
        try:
            parsed = float(value)
        except (TypeError, ValueError) as error:
            raise RuntimeError(f"{field} must be numeric.") from error
        if not math.isfinite(parsed):
            raise RuntimeError(f"{field} must be finite.")
        return parsed

    def _write_event(self, event: Mapping[str, Any]) -> None:
        self.event_sink(event)


class BoundedSessionHttpServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(
        self,
        server_address: tuple[str, int],
        core: BoundedSessionCore,
        allowed_clients: set[str],
    ) -> None:
        super().__init__(server_address, BoundedSessionRequestHandler)
        self.core = core
        self.allowed_clients = allowed_clients


class BoundedSessionRequestHandler(BaseHTTPRequestHandler):
    server: BoundedSessionHttpServer

    def do_GET(self) -> None:
        if self.path != HEALTH_PATH:
            self._send_json(404, {"status": "error", "code": "NOT_FOUND"})
            return
        if not self._client_allowed():
            return
        self._send_json(200, self.server.core.health_payload())

    def do_POST(self) -> None:
        if self.path not in {SESSION_ARM_PATH, SESSION_STEP_PATH, SESSION_STOP_PATH}:
            self._send_json(
                404,
                {
                    "status": "error",
                    "code": "NOT_FOUND",
                    "message": "No direct remote execution endpoint exists.",
                },
            )
            return
        if not self._client_allowed():
            return
        if not self.headers.get("Content-Type", "").lower().startswith("application/json"):
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
            if self.path == SESSION_ARM_PATH:
                status, response = self.server.core.handle_remote_arm(body, signature)
            elif self.path == SESSION_STEP_PATH:
                status, response = self.server.core.handle_step(body, signature)
            else:
                status, response = self.server.core.handle_stop(body, signature)
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

    def _client_allowed(self) -> bool:
        if self.client_address[0] not in self.server.allowed_clients:
            self._send_json(403, {"status": "error", "code": "CLIENT_NOT_ALLOWED"})
            return False
        return True

    def _send_json(self, status: int, payload: Mapping[str, Any]) -> None:
        body = json.dumps(payload, allow_nan=False, indent=2).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, message_format: str, *args: Any) -> None:
        print(f"[SESSION HTTP] {self.client_address[0]} {message_format % args}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run a bounded FR3 execution-session supervisor."
    )
    parser.add_argument("--enable-bounded-session-execution", action="store_true")
    parser.add_argument(
        "--enable-remote-session-arm",
        action="store_true",
        help="Permit authenticated Windows requests to create bounded sessions.",
    )
    parser.add_argument(
        "--remote-arm-joint",
        choices=JOINTS,
        action="append",
        dest="remote_arm_joints",
        help="Joint Windows may select; repeat to permit a bounded allowlist.",
    )
    parser.add_argument("--bind-host", default="10.10.10.1")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--allowed-client", action="append", default=[])
    parser.add_argument("--shared-secret-env", default=DEFAULT_SECRET_ENV)
    parser.add_argument("--max-request-age", type=float, default=10.0)
    parser.add_argument("--max-future-skew", type=float, default=2.0)
    parser.add_argument("--replay-retention", type=float, default=600.0)
    parser.add_argument(
        "--approval-socket",
        type=Path,
        default=Path("/tmp/fr3_bounded_session.sock"),
    )
    parser.add_argument(
        "--session-log-dir",
        type=Path,
        default=WORKSPACE_ROOT / "session_logs" / "ubuntu",
    )
    parser.add_argument("--session-max-steps", type=int, default=4)
    parser.add_argument("--session-max-duration", type=float, default=60.0)
    parser.add_argument("--session-max-cumulative-delta", type=float, default=0.12)

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
    parser.add_argument("--start-state-tolerance", type=float, default=0.01)
    parser.add_argument("--max-start-velocity", type=float, default=0.01)
    parser.add_argument("--settle-time", type=float, default=1.0)
    parser.add_argument("--verification-timeout", type=float, default=2.0)
    parser.add_argument("--verification-tolerance", type=float, default=0.01)
    parser.add_argument(
        "--joint7-verification-tolerance", type=float, default=0.01
    )
    parser.add_argument("--non-commanded-drift-limit", type=float, default=0.01)
    parser.add_argument("--max-settled-velocity", type=float, default=0.01)
    parser.add_argument("--minimum-measured-delta", type=float, default=0.001)
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> set[str]:
    if not args.enable_bounded_session_execution:
        raise RuntimeError("Explicit --enable-bounded-session-execution is required.")
    if args.enable_remote_session_arm and not args.remote_arm_joints:
        raise RuntimeError("--remote-arm-joint is required with --enable-remote-session-arm.")
    if not args.enable_remote_session_arm and args.remote_arm_joints:
        raise RuntimeError("--remote-arm-joint requires --enable-remote-session-arm.")
    if args.remote_arm_joints:
        if len(set(args.remote_arm_joints)) != len(args.remote_arm_joints):
            raise RuntimeError("--remote-arm-joint cannot contain duplicates.")
        args.remote_arm_joints = [
            joint for joint in JOINTS if joint in set(args.remote_arm_joints)
        ]
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
            raise RuntimeError(f"Invalid --allowed-client: {client}") from error

    positive = {
        "max_request_age": args.max_request_age,
        "max_future_skew": args.max_future_skew,
        "replay_retention": args.replay_retention,
        "session_max_duration": args.session_max_duration,
        "session_max_cumulative_delta": args.session_max_cumulative_delta,
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
        "start_state_tolerance": args.start_state_tolerance,
        "max_start_velocity": args.max_start_velocity,
        "verification_timeout": args.verification_timeout,
        "verification_tolerance": args.verification_tolerance,
        "joint7_verification_tolerance": args.joint7_verification_tolerance,
        "non_commanded_drift_limit": args.non_commanded_drift_limit,
        "max_settled_velocity": args.max_settled_velocity,
        "minimum_measured_delta": args.minimum_measured_delta,
    }
    for name, value in positive.items():
        if not math.isfinite(value) or value <= 0.0:
            raise RuntimeError(f"--{name.replace('_', '-')} must be finite and positive.")
    if not math.isfinite(args.settle_time) or args.settle_time < 0.0:
        raise RuntimeError("--settle-time must be finite and non-negative.")
    if not 1 <= args.session_max_steps <= ABSOLUTE_MAX_STEPS:
        raise RuntimeError(f"--session-max-steps cannot exceed {ABSOLUTE_MAX_STEPS}.")
    if args.session_max_duration > ABSOLUTE_MAX_DURATION_SECONDS:
        raise RuntimeError(
            f"--session-max-duration cannot exceed {ABSOLUTE_MAX_DURATION_SECONDS:.0f}."
        )
    if args.session_max_cumulative_delta > ABSOLUTE_MAX_CUMULATIVE_DELTA:
        raise RuntimeError(
            "--session-max-cumulative-delta exceeds the absolute supervisor cap."
        )
    if args.max_delta > ABSOLUTE_MAX_ACTION_DELTA:
        raise RuntimeError("--max-delta exceeds the absolute supervisor cap.")
    if not 1 <= args.max_moving_joints <= len(JOINTS):
        raise RuntimeError("--max-moving-joints must be between 1 and 7.")
    if args.max_total_delta > args.max_delta * args.max_moving_joints + 1e-12:
        raise RuntimeError(
            "--max-total-delta exceeds --max-delta times --max-moving-joints."
        )
    if args.session_max_cumulative_delta + 1e-12 < args.max_total_delta:
        raise RuntimeError(
            "--session-max-cumulative-delta cannot be smaller than --max-total-delta."
        )
    if args.remote_arm_joints and args.max_moving_joints > len(args.remote_arm_joints):
        raise RuntimeError(
            "--max-moving-joints exceeds the remote joint allowlist size."
        )
    if not 0.0 < args.min_confidence <= 1.0:
        raise RuntimeError("--min-confidence must be in (0, 1].")
    if not 0.0 < args.velocity_scaling <= 1.0:
        raise RuntimeError("--velocity-scaling must be in (0, 1].")
    if not 0.0 < args.acceleration_scaling <= 1.0:
        raise RuntimeError("--acceleration-scaling must be in (0, 1].")
    if args.planning_attempts <= 0:
        raise RuntimeError("--planning-attempts must be positive.")
    return clients


def main() -> int:
    args = parse_args()
    try:
        allowed_clients = validate_args(args)
        secret = load_shared_secret(args.shared_secret_env)
        audit = SessionAuditLog(args.session_log_dir)
    except (RuntimeError, ProtocolError, OSError) as error:
        print(f"Configuration error: {error}", file=sys.stderr)
        return 2

    args.enable_local_execution = True
    planner = None
    http_server = None
    local_server = None
    local_thread = None
    try:
        planner = RosMoveItPlanner(
            args, action_validator=validate_joint_delta_action_contract
        )
        server_limits = SessionLimits(
            allowed_joints=JOINTS,
            max_moving_joints=args.max_moving_joints,
            max_steps=args.session_max_steps,
            max_duration_seconds=args.session_max_duration,
            max_delta=args.max_delta,
            max_total_delta=args.max_total_delta,
            max_cumulative_delta=args.session_max_cumulative_delta,
        )
        core = BoundedSessionCore(
            planner,
            secret,
            max_request_age_ms=int(args.max_request_age * 1000),
            max_future_skew_ms=int(args.max_future_skew * 1000),
            replay_retention_seconds=args.replay_retention,
            server_limits=server_limits,
            minimum_confidence=args.min_confidence,
            remote_arm_joints=(
                tuple(args.remote_arm_joints) if args.enable_remote_session_arm else None
            ),
            event_sink=audit.write,
        )
        local_server = create_approval_server(args.approval_socket, core)
        local_thread = threading.Thread(
            target=local_server.serve_forever,
            kwargs={"poll_interval": 0.2},
            daemon=True,
        )
        local_thread.start()
        http_server = BoundedSessionHttpServer(
            (args.bind_host, args.port), core, allowed_clients
        )

        print("FR3 bounded-session supervisor started.")
        print("NO SESSION IS ARMED. THE ROBOT CANNOT MOVE FROM REMOTE REQUESTS YET.")
        print(f"Health: {args.bind_host}:{args.port}{HEALTH_PATH}")
        if args.enable_remote_session_arm:
            print(f"Remote arm:  {SESSION_ARM_PATH} ({args.remote_arm_joints})")
            print("REMOTE SESSION ARM IS ENABLED UNTIL THIS SUPERVISOR STOPS.")
        else:
            print("Remote arm:  DISABLED")
        print(f"Session step: {SESSION_STEP_PATH}")
        print(f"Safe stop:    {SESSION_STOP_PATH}")
        print("Direct remote execution endpoint: NONE")
        print(f"Local arm socket: {args.approval_socket} (mode 0600)")
        print(f"Allowed Windows clients: {sorted(allowed_clients)}")
        print(
            "Server caps: "
            f"steps={server_limits.max_steps}, "
            f"duration={server_limits.max_duration_seconds:.1f}s, "
            f"moving_joints={server_limits.max_moving_joints}, "
            f"per_joint={server_limits.max_delta:.3f}rad, "
            f"per_step_total={server_limits.max_total_delta:.3f}rad, "
            f"cumulative={server_limits.max_cumulative_delta:.3f}rad"
        )
        http_server.serve_forever(poll_interval=0.2)
        return 0
    except KeyboardInterrupt:
        print("\nBounded-session supervisor stopped.")
        return 130
    except Exception as error:
        print(f"Supervisor error: {type(error).__name__}: {error}", file=sys.stderr)
        return 1
    finally:
        if local_server is not None:
            local_server.shutdown()
            local_server.server_close()
        if local_thread is not None:
            local_thread.join(timeout=2.0)
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
