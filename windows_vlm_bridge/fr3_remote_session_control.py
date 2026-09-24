"""Request, inspect, or stop an Ubuntu-bounded FR3 session from Windows."""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from typing import Any, Mapping
import urllib.error
import urllib.parse
import urllib.request
import uuid

from fr3_action_protocol import (
    DEFAULT_SECRET_ENV,
    SIGNATURE_HEADER,
    canonical_json_bytes,
    create_session_arm_envelope,
    create_session_stop_envelope,
    load_shared_secret,
    make_signature,
)


DEFAULT_UBUNTU_URL = "http://10.10.10.1:8080"
JOINTS = tuple(f"fr3_joint{index}" for index in range(1, 8))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Control the session authority exposed by an Ubuntu supervisor that "
            "was explicitly started with remote session arm enabled."
        )
    )
    parser.add_argument("--ubuntu-url", default=DEFAULT_UBUNTU_URL)
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--shared-secret-env", default=DEFAULT_SECRET_ENV)
    subparsers = parser.add_subparsers(dest="command", required=True)

    arm = subparsers.add_parser("arm", help="Request one new bounded session.")
    arm.add_argument("--joint", choices=JOINTS, action="append", required=True)
    arm.add_argument("--max-moving-joints", type=int, default=1)
    arm.add_argument("--max-steps", type=int, default=2)
    arm.add_argument("--max-duration", type=float, default=60.0)
    arm.add_argument("--max-delta", type=float, default=0.01)
    arm.add_argument("--max-total-delta", type=float, default=0.01)
    arm.add_argument("--max-cumulative-delta", type=float, default=0.02)
    arm.add_argument("--session-id", default="", help=argparse.SUPPRESS)

    subparsers.add_parser("status", help="Read the current bounded-session state.")

    stop = subparsers.add_parser("stop", help="Remove authority for future session steps.")
    stop.add_argument(
        "--session-id",
        default="",
        help="Optional exact session UUID; otherwise discover the current session.",
    )
    return parser.parse_args()


def validate_base_url(value: str) -> str:
    parsed = urllib.parse.urlsplit(value)
    if parsed.scheme not in {"http", "https"} or not parsed.hostname:
        raise RuntimeError("--ubuntu-url must be an absolute HTTP(S) server URL.")
    if parsed.username or parsed.password or parsed.query or parsed.fragment:
        raise RuntimeError("--ubuntu-url cannot contain credentials, query, or fragment.")
    if parsed.path not in {"", "/"}:
        raise RuntimeError("--ubuntu-url must not contain an endpoint path.")
    return value.rstrip("/")


def validate_common(args: argparse.Namespace) -> str:
    if not math.isfinite(args.timeout) or args.timeout <= 0.0:
        raise RuntimeError("--timeout must be finite and positive.")
    return validate_base_url(args.ubuntu_url)


def validate_uuid4(value: str) -> str:
    try:
        parsed = uuid.UUID(value)
    except ValueError as error:
        raise RuntimeError("session ID must be a canonical UUIDv4.") from error
    if parsed.version != 4 or str(parsed) != value.lower():
        raise RuntimeError("session ID must be a canonical UUIDv4.")
    return value


def validate_arm_args(args: argparse.Namespace) -> None:
    if args.max_steps <= 0:
        raise RuntimeError("--max-steps must be positive.")
    if len(set(args.joint)) != len(args.joint):
        raise RuntimeError("--joint cannot contain duplicates.")
    args.joint = [joint for joint in JOINTS if joint in set(args.joint)]
    if not 1 <= args.max_moving_joints <= len(args.joint):
        raise RuntimeError("--max-moving-joints exceeds the allowed joint count.")
    for name in ("max_duration", "max_delta", "max_total_delta", "max_cumulative_delta"):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0.0:
            raise RuntimeError(f"--{name.replace('_', '-')} must be finite and positive.")
    if args.max_total_delta > args.max_delta * args.max_moving_joints + 1e-12:
        raise RuntimeError("--max-total-delta exceeds the per-joint action budget.")
    if args.max_cumulative_delta + 1e-12 < args.max_total_delta:
        raise RuntimeError("--max-cumulative-delta cannot be smaller than --max-total-delta.")


def request_json(
    url: str,
    *,
    timeout: float,
    body: bytes | None = None,
    signature: str = "",
) -> tuple[int, Any]:
    headers = {"Accept": "application/json"}
    method = "GET"
    if body is not None:
        method = "POST"
        headers["Content-Type"] = "application/json"
        headers[SIGNATURE_HEADER] = signature
    request = urllib.request.Request(url, data=body, headers=headers, method=method)
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            raw = response.read()
            status = response.status
    except urllib.error.HTTPError as error:
        raw = error.read()
        status = error.code
    except (urllib.error.URLError, TimeoutError, OSError) as error:
        raise RuntimeError(f"Could not reach {url}: {error}") from error
    try:
        return status, json.loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise RuntimeError(f"Ubuntu returned non-JSON data with HTTP {status}.") from error


def get_health(base_url: str, timeout: float) -> dict[str, Any]:
    status, response = request_json(f"{base_url}/health", timeout=timeout)
    if status != 200 or not isinstance(response, dict):
        raise RuntimeError(f"Ubuntu health failed with HTTP {status}: {response}")
    if response.get("mode") != "BOUNDED_SESSION_EXECUTION":
        raise RuntimeError("Ubuntu is not running the bounded-session supervisor.")
    return response


def print_session(session: Mapping[str, Any]) -> None:
    limits = session.get("limits") if isinstance(session.get("limits"), dict) else {}
    print(f"Session ID:       {session.get('session_id')}")
    print(f"Status:           {session.get('status')}")
    print(f"Allowed joints:   {limits.get('allowed_joints')}")
    print(f"Moving-joint cap: {limits.get('max_moving_joints')}")
    print(f"Next sequence:    {session.get('next_sequence')}")
    print(f"Completed steps:  {session.get('completed_steps')}/{limits.get('max_steps')}")
    print(
        "Cumulative:      "
        f"{float(session.get('cumulative_absolute_delta', 0.0)):.6f}/"
        f"{float(limits.get('max_cumulative_delta', 0.0)):.6f} rad"
    )
    print(f"Per-step limit:   {float(limits.get('max_delta', 0.0)):.6f} rad")
    print(f"Step-total limit: {float(limits.get('max_total_delta', 0.0)):.6f} rad")
    print(f"Remaining:        {float(session.get('remaining_seconds', 0.0)):.1f} s")
    if session.get("terminal_code"):
        print(f"Terminal code:    {session.get('terminal_code')}")


def arm_session(args: argparse.Namespace, base_url: str) -> int:
    validate_arm_args(args)
    health = get_health(base_url, args.timeout)
    if health.get("remote_session_arm_available") is not True:
        raise RuntimeError("Ubuntu has not enabled authenticated remote session arm.")

    session_id = validate_uuid4(args.session_id) if args.session_id else str(uuid.uuid4())
    envelope = create_session_arm_envelope(
        session_id=session_id,
        allowed_joints=args.joint,
        max_moving_joints=args.max_moving_joints,
        max_steps=args.max_steps,
        max_duration_seconds=args.max_duration,
        max_delta=args.max_delta,
        max_total_delta=args.max_total_delta,
        max_cumulative_delta=args.max_cumulative_delta,
    )
    body = canonical_json_bytes(envelope)
    secret = load_shared_secret(args.shared_secret_env)
    status, response = request_json(
        f"{base_url}/v1/sessions/arm",
        body=body,
        signature=make_signature(body, secret),
        timeout=args.timeout,
    )
    if status != 201 or not isinstance(response, dict):
        raise RuntimeError(f"Ubuntu arm request failed with HTTP {status}: {response}")
    if response.get("status") != "SESSION_ARMED":
        raise RuntimeError("Ubuntu did not confirm SESSION_ARMED.")
    if response.get("request_id") != envelope["request_id"]:
        raise RuntimeError("Ubuntu arm receipt request ID mismatch.")
    session = response.get("session")
    if not isinstance(session, dict) or session.get("session_id") != session_id:
        raise RuntimeError("Ubuntu arm receipt session ID mismatch.")
    if session.get("status") != "ARMED":
        raise RuntimeError("Ubuntu arm receipt does not contain an ARMED session.")

    print("REMOTE BOUNDED SESSION ARMED")
    print_session(session)
    print("\nThis created bounded authority; it did not move the robot.")
    return 0


def status_session(args: argparse.Namespace, base_url: str) -> int:
    health = get_health(base_url, args.timeout)
    session = health.get("session")
    if not isinstance(session, dict):
        print("No bounded session exists on Ubuntu.")
        return 1
    print_session(session)
    return 0


def stop_session(args: argparse.Namespace, base_url: str) -> int:
    if args.session_id:
        session_id = validate_uuid4(args.session_id)
    else:
        health = get_health(base_url, args.timeout)
        session = health.get("session")
        if not isinstance(session, dict):
            raise RuntimeError("No bounded session exists on Ubuntu.")
        session_id = validate_uuid4(str(session.get("session_id", "")))

    envelope = create_session_stop_envelope(session_id)
    body = canonical_json_bytes(envelope)
    secret = load_shared_secret(args.shared_secret_env)
    status, response = request_json(
        f"{base_url}/v1/sessions/stop",
        body=body,
        signature=make_signature(body, secret),
        timeout=args.timeout,
    )
    if status != 200 or not isinstance(response, dict):
        raise RuntimeError(f"Ubuntu stop request failed with HTTP {status}: {response}")
    if response.get("request_id") != envelope["request_id"]:
        raise RuntimeError("Ubuntu stop receipt request ID mismatch.")
    session = response.get("session")
    if not isinstance(session, dict) or session.get("session_id") != session_id:
        raise RuntimeError("Ubuntu stop receipt session ID mismatch.")
    print("REMOTE SESSION STOP REQUESTED")
    print_session(session)
    return 0


def main() -> int:
    args = parse_args()
    try:
        base_url = validate_common(args)
        if args.command == "arm":
            return arm_session(args, base_url)
        if args.command == "status":
            return status_session(args, base_url)
        return stop_session(args, base_url)
    except (KeyboardInterrupt, EOFError):
        print("\nRemote session control cancelled.")
        return 130
    except Exception as error:
        print(f"Remote session control error: {type(error).__name__}: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
