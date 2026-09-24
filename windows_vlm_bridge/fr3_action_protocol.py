"""Shared authenticated message protocol for Windows-to-Ubuntu FR3 proposals."""

from __future__ import annotations

import hashlib
import hmac
import json
import math
import os
import re
import time
import uuid
from collections import OrderedDict
from threading import Lock
from typing import Any, Mapping


SCHEMA_VERSION = 2
EXPECTED_SOURCE = "windows-isaac-qwen"
SIGNATURE_HEADER = "X-FR3-Signature"
DEFAULT_SECRET_ENV = "FR3_BRIDGE_SECRET"
MAX_BODY_BYTES = 64 * 1024
MIN_SECRET_LENGTH = 32
FR3_JOINTS = tuple(f"fr3_joint{index}" for index in range(1, 8))

_SHA256_PATTERN = re.compile(r"^[0-9a-f]{64}$")


class ProtocolError(ValueError):
    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code


def canonical_json_bytes(value: Mapping[str, Any]) -> bytes:
    return json.dumps(
        value,
        allow_nan=False,
        ensure_ascii=True,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")


def load_shared_secret(environment_name: str = DEFAULT_SECRET_ENV) -> bytes:
    value = os.environ.get(environment_name, "")
    if len(value) < MIN_SECRET_LENGTH:
        raise ProtocolError(
            "SECRET_NOT_CONFIGURED",
            f"{environment_name} must contain at least {MIN_SECRET_LENGTH} characters.",
        )
    return value.encode("utf-8")


def make_signature(body: bytes, secret: bytes) -> str:
    digest = hmac.new(secret, body, hashlib.sha256).hexdigest()
    return f"sha256={digest}"


def verify_signature(body: bytes, supplied_signature: str, secret: bytes) -> None:
    expected = make_signature(body, secret)
    if not supplied_signature or not hmac.compare_digest(expected, supplied_signature):
        raise ProtocolError("INVALID_SIGNATURE", "Request signature is missing or invalid.")


def create_action_envelope(
    action: Mapping[str, Any],
    *,
    frame_source: str,
    frame_id: str,
    frame_sha256: str,
    now_unix_ms: int | None = None,
    request_id: str | None = None,
) -> dict[str, Any]:
    return {
        "schema_version": SCHEMA_VERSION,
        "request_id": request_id or str(uuid.uuid4()),
        "created_at_unix_ms": (
            int(time.time() * 1000) if now_unix_ms is None else now_unix_ms
        ),
        "source": EXPECTED_SOURCE,
        "frame": {
            "source": frame_source,
            "frame_id": frame_id,
            "sha256": frame_sha256,
        },
        "action": dict(action),
    }


def validate_action_envelope(
    envelope: Any,
    *,
    now_unix_ms: int,
    max_age_ms: int,
    max_future_skew_ms: int,
) -> dict[str, Any]:
    if not isinstance(envelope, dict):
        raise ProtocolError("INVALID_ENVELOPE", "Request body must be a JSON object.")

    expected_keys = {
        "schema_version",
        "request_id",
        "created_at_unix_ms",
        "source",
        "frame",
        "action",
    }
    if set(envelope) != expected_keys:
        raise ProtocolError(
            "INVALID_ENVELOPE",
            f"Envelope keys must be exactly: {sorted(expected_keys)}.",
        )

    if (
        isinstance(envelope["schema_version"], bool)
        or envelope["schema_version"] != SCHEMA_VERSION
    ):
        raise ProtocolError(
            "UNSUPPORTED_SCHEMA",
            f"schema_version must equal {SCHEMA_VERSION}.",
        )

    request_id = envelope["request_id"]
    if not isinstance(request_id, str):
        raise ProtocolError("INVALID_REQUEST_ID", "request_id must be a UUID string.")
    try:
        parsed_request_id = uuid.UUID(request_id)
    except ValueError as error:
        raise ProtocolError("INVALID_REQUEST_ID", "request_id must be a valid UUID.") from error
    if parsed_request_id.version != 4 or str(parsed_request_id) != request_id.lower():
        raise ProtocolError("INVALID_REQUEST_ID", "request_id must be a canonical UUIDv4.")

    created_at = envelope["created_at_unix_ms"]
    if isinstance(created_at, bool) or not isinstance(created_at, int):
        raise ProtocolError(
            "INVALID_TIMESTAMP", "created_at_unix_ms must be an integer."
        )
    age_ms = now_unix_ms - created_at
    if age_ms > max_age_ms:
        raise ProtocolError(
            "STALE_REQUEST",
            f"Request is {age_ms} ms old; maximum age is {max_age_ms} ms.",
        )
    if age_ms < -max_future_skew_ms:
        raise ProtocolError(
            "FUTURE_REQUEST",
            f"Request timestamp is {-age_ms} ms in the future.",
        )

    if envelope["source"] != EXPECTED_SOURCE:
        raise ProtocolError(
            "INVALID_SOURCE", f"source must equal {EXPECTED_SOURCE!r}."
        )

    frame = envelope["frame"]
    if not isinstance(frame, dict) or set(frame) != {"source", "frame_id", "sha256"}:
        raise ProtocolError(
            "INVALID_FRAME", "frame must contain source, frame_id, and sha256."
        )
    for field in ("source", "frame_id"):
        value = frame[field]
        if not isinstance(value, str) or not value.strip() or len(value) > 128:
            raise ProtocolError(
                "INVALID_FRAME", f"frame.{field} must be a non-empty short string."
            )
    frame_digest = frame["sha256"]
    if not isinstance(frame_digest, str) or not _SHA256_PATTERN.fullmatch(frame_digest):
        raise ProtocolError(
            "INVALID_FRAME", "frame.sha256 must be a lowercase SHA-256 hex digest."
        )

    if not isinstance(envelope["action"], dict):
        raise ProtocolError("INVALID_ACTION", "action must be a JSON object.")
    return envelope


def action_sha256(action: Mapping[str, Any]) -> str:
    return hashlib.sha256(canonical_json_bytes(action)).hexdigest()


def create_session_step_envelope(
    action: Mapping[str, Any],
    *,
    session_id: str,
    sequence: int,
    frame_source: str,
    frame_id: str,
    frame_sha256: str,
    now_unix_ms: int | None = None,
    request_id: str | None = None,
) -> dict[str, Any]:
    return {
        "schema_version": SCHEMA_VERSION,
        "request_id": request_id or str(uuid.uuid4()),
        "created_at_unix_ms": (
            int(time.time() * 1000) if now_unix_ms is None else now_unix_ms
        ),
        "source": EXPECTED_SOURCE,
        "session_id": session_id,
        "sequence": sequence,
        "frame": {
            "source": frame_source,
            "frame_id": frame_id,
            "sha256": frame_sha256,
        },
        "action": dict(action),
    }


def create_session_arm_envelope(
    *,
    session_id: str,
    allowed_joints: list[str] | tuple[str, ...],
    max_moving_joints: int,
    max_steps: int,
    max_duration_seconds: float,
    max_delta: float,
    max_total_delta: float,
    max_cumulative_delta: float,
    now_unix_ms: int | None = None,
    request_id: str | None = None,
) -> dict[str, Any]:
    return {
        "schema_version": SCHEMA_VERSION,
        "request_id": request_id or str(uuid.uuid4()),
        "created_at_unix_ms": (
            int(time.time() * 1000) if now_unix_ms is None else now_unix_ms
        ),
        "source": EXPECTED_SOURCE,
        "session_id": session_id,
        "operation": "arm",
        "limits": {
            "allowed_joints": list(allowed_joints),
            "max_moving_joints": max_moving_joints,
            "max_steps": max_steps,
            "max_duration_seconds": max_duration_seconds,
            "max_delta": max_delta,
            "max_total_delta": max_total_delta,
            "max_cumulative_delta": max_cumulative_delta,
        },
    }


def validate_session_arm_envelope(
    envelope: Any,
    *,
    now_unix_ms: int,
    max_age_ms: int,
    max_future_skew_ms: int,
) -> dict[str, Any]:
    if not isinstance(envelope, dict):
        raise ProtocolError("INVALID_ENVELOPE", "Request body must be a JSON object.")
    expected_keys = {
        "schema_version",
        "request_id",
        "created_at_unix_ms",
        "source",
        "session_id",
        "operation",
        "limits",
    }
    if set(envelope) != expected_keys:
        raise ProtocolError(
            "INVALID_ENVELOPE",
            f"Session-arm keys must be exactly: {sorted(expected_keys)}.",
        )
    if (
        isinstance(envelope["schema_version"], bool)
        or envelope["schema_version"] != SCHEMA_VERSION
    ):
        raise ProtocolError(
            "UNSUPPORTED_SCHEMA", f"schema_version must equal {SCHEMA_VERSION}."
        )
    _validate_canonical_uuid4(envelope["request_id"], "INVALID_REQUEST_ID", "request_id")
    _validate_fresh_timestamp(
        envelope["created_at_unix_ms"],
        now_unix_ms=now_unix_ms,
        max_age_ms=max_age_ms,
        max_future_skew_ms=max_future_skew_ms,
    )
    if envelope["source"] != EXPECTED_SOURCE:
        raise ProtocolError(
            "INVALID_SOURCE", f"source must equal {EXPECTED_SOURCE!r}."
        )
    _validate_canonical_uuid4(envelope["session_id"], "INVALID_SESSION_ID", "session_id")
    if envelope["operation"] != "arm":
        raise ProtocolError("INVALID_OPERATION", "operation must equal 'arm'.")

    limits = envelope["limits"]
    expected_limits = {
        "allowed_joints",
        "max_moving_joints",
        "max_steps",
        "max_duration_seconds",
        "max_delta",
        "max_total_delta",
        "max_cumulative_delta",
    }
    if not isinstance(limits, dict) or set(limits) != expected_limits:
        raise ProtocolError(
            "INVALID_SESSION_LIMITS",
            f"limits keys must be exactly: {sorted(expected_limits)}.",
        )
    allowed_joints = limits["allowed_joints"]
    if not isinstance(allowed_joints, list) or not allowed_joints:
        raise ProtocolError(
            "INVALID_SESSION_LIMITS",
            "limits.allowed_joints must be a non-empty list.",
        )
    if any(not isinstance(joint, str) or joint not in FR3_JOINTS for joint in allowed_joints):
        raise ProtocolError(
            "INVALID_SESSION_LIMITS",
            "limits.allowed_joints contains an unknown FR3 joint.",
        )
    if len(set(allowed_joints)) != len(allowed_joints):
        raise ProtocolError(
            "INVALID_SESSION_LIMITS",
            "limits.allowed_joints cannot contain duplicates.",
        )
    expected_order = [joint for joint in FR3_JOINTS if joint in set(allowed_joints)]
    if allowed_joints != expected_order:
        raise ProtocolError(
            "INVALID_SESSION_LIMITS",
            "limits.allowed_joints must use canonical FR3 joint order.",
        )

    moving_limit = limits["max_moving_joints"]
    if (
        isinstance(moving_limit, bool)
        or not isinstance(moving_limit, int)
        or not 1 <= moving_limit <= len(allowed_joints)
    ):
        raise ProtocolError(
            "INVALID_SESSION_LIMITS",
            "limits.max_moving_joints must be between 1 and the allowlist size.",
        )
    steps = limits["max_steps"]
    if isinstance(steps, bool) or not isinstance(steps, int) or steps <= 0:
        raise ProtocolError(
            "INVALID_SESSION_LIMITS", "limits.max_steps must be a positive integer."
        )
    for field in (
        "max_duration_seconds",
        "max_delta",
        "max_total_delta",
        "max_cumulative_delta",
    ):
        value = limits[field]
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ProtocolError(
                "INVALID_SESSION_LIMITS", f"limits.{field} must be numeric."
            )
        if not math.isfinite(float(value)) or float(value) <= 0.0:
            raise ProtocolError(
                "INVALID_SESSION_LIMITS", f"limits.{field} must be finite and positive."
            )
    max_delta = float(limits["max_delta"])
    max_total_delta = float(limits["max_total_delta"])
    max_cumulative_delta = float(limits["max_cumulative_delta"])
    if max_total_delta > max_delta * moving_limit + 1e-12:
        raise ProtocolError(
            "INVALID_SESSION_LIMITS",
            "limits.max_total_delta exceeds max_delta times max_moving_joints.",
        )
    if max_cumulative_delta + 1e-12 < max_total_delta:
        raise ProtocolError(
            "INVALID_SESSION_LIMITS",
            "limits.max_cumulative_delta cannot be smaller than max_total_delta.",
        )
    return envelope


def validate_session_step_envelope(
    envelope: Any,
    *,
    now_unix_ms: int,
    max_age_ms: int,
    max_future_skew_ms: int,
) -> dict[str, Any]:
    if not isinstance(envelope, dict):
        raise ProtocolError("INVALID_ENVELOPE", "Request body must be a JSON object.")
    expected_keys = {
        "schema_version",
        "request_id",
        "created_at_unix_ms",
        "source",
        "session_id",
        "sequence",
        "frame",
        "action",
    }
    if set(envelope) != expected_keys:
        raise ProtocolError(
            "INVALID_ENVELOPE",
            f"Session-step keys must be exactly: {sorted(expected_keys)}.",
        )

    base_envelope = {
        key: envelope[key]
        for key in (
            "schema_version",
            "request_id",
            "created_at_unix_ms",
            "source",
            "frame",
            "action",
        )
    }
    validate_action_envelope(
        base_envelope,
        now_unix_ms=now_unix_ms,
        max_age_ms=max_age_ms,
        max_future_skew_ms=max_future_skew_ms,
    )
    _validate_canonical_uuid4(envelope["session_id"], "INVALID_SESSION_ID", "session_id")
    sequence = envelope["sequence"]
    if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence <= 0:
        raise ProtocolError("INVALID_SEQUENCE", "sequence must be a positive integer.")
    return envelope


def create_session_stop_envelope(
    session_id: str,
    *,
    now_unix_ms: int | None = None,
    request_id: str | None = None,
) -> dict[str, Any]:
    return {
        "schema_version": SCHEMA_VERSION,
        "request_id": request_id or str(uuid.uuid4()),
        "created_at_unix_ms": (
            int(time.time() * 1000) if now_unix_ms is None else now_unix_ms
        ),
        "source": EXPECTED_SOURCE,
        "session_id": session_id,
        "operation": "stop",
    }


def validate_session_stop_envelope(
    envelope: Any,
    *,
    now_unix_ms: int,
    max_age_ms: int,
    max_future_skew_ms: int,
) -> dict[str, Any]:
    if not isinstance(envelope, dict):
        raise ProtocolError("INVALID_ENVELOPE", "Request body must be a JSON object.")
    expected_keys = {
        "schema_version",
        "request_id",
        "created_at_unix_ms",
        "source",
        "session_id",
        "operation",
    }
    if set(envelope) != expected_keys:
        raise ProtocolError(
            "INVALID_ENVELOPE",
            f"Session-stop keys must be exactly: {sorted(expected_keys)}.",
        )
    if (
        isinstance(envelope["schema_version"], bool)
        or envelope["schema_version"] != SCHEMA_VERSION
    ):
        raise ProtocolError(
            "UNSUPPORTED_SCHEMA", f"schema_version must equal {SCHEMA_VERSION}."
        )
    _validate_canonical_uuid4(envelope["request_id"], "INVALID_REQUEST_ID", "request_id")
    _validate_fresh_timestamp(
        envelope["created_at_unix_ms"],
        now_unix_ms=now_unix_ms,
        max_age_ms=max_age_ms,
        max_future_skew_ms=max_future_skew_ms,
    )
    if envelope["source"] != EXPECTED_SOURCE:
        raise ProtocolError(
            "INVALID_SOURCE", f"source must equal {EXPECTED_SOURCE!r}."
        )
    _validate_canonical_uuid4(envelope["session_id"], "INVALID_SESSION_ID", "session_id")
    if envelope["operation"] != "stop":
        raise ProtocolError("INVALID_OPERATION", "operation must equal 'stop'.")
    return envelope


def _validate_canonical_uuid4(value: Any, code: str, field: str) -> None:
    if not isinstance(value, str):
        raise ProtocolError(code, f"{field} must be a UUID string.")
    try:
        parsed = uuid.UUID(value)
    except ValueError as error:
        raise ProtocolError(code, f"{field} must be a valid UUID.") from error
    if parsed.version != 4 or str(parsed) != value.lower():
        raise ProtocolError(code, f"{field} must be a canonical UUIDv4.")


def _validate_fresh_timestamp(
    value: Any,
    *,
    now_unix_ms: int,
    max_age_ms: int,
    max_future_skew_ms: int,
) -> None:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ProtocolError("INVALID_TIMESTAMP", "created_at_unix_ms must be an integer.")
    age_ms = now_unix_ms - value
    if age_ms > max_age_ms:
        raise ProtocolError(
            "STALE_REQUEST",
            f"Request is {age_ms} ms old; maximum age is {max_age_ms} ms.",
        )
    if age_ms < -max_future_skew_ms:
        raise ProtocolError(
            "FUTURE_REQUEST",
            f"Request timestamp is {-age_ms} ms in the future.",
        )


class ReplayCache:
    def __init__(self, retention_seconds: float = 300.0, maximum_entries: int = 2048) -> None:
        if retention_seconds <= 0.0 or maximum_entries <= 0:
            raise ValueError("Replay-cache limits must be positive.")
        self._retention_seconds = retention_seconds
        self._maximum_entries = maximum_entries
        self._entries: OrderedDict[str, float] = OrderedDict()
        self._lock = Lock()

    def record_if_new(self, request_id: str, now_monotonic: float | None = None) -> bool:
        now = time.monotonic() if now_monotonic is None else now_monotonic
        with self._lock:
            cutoff = now - self._retention_seconds
            while self._entries:
                _, oldest_time = next(iter(self._entries.items()))
                if oldest_time >= cutoff:
                    break
                self._entries.popitem(last=False)

            if request_id in self._entries:
                return False

            self._entries[request_id] = now
            while len(self._entries) > self._maximum_entries:
                self._entries.popitem(last=False)
            return True
