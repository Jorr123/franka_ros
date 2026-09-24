#!/usr/bin/env python3
"""Locally arm, inspect, or disarm one bounded FR3 execution session."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import uuid

from fr3_supervisor_approve import send_local_request


DEFAULT_SOCKET = Path("/tmp/fr3_bounded_session.sock")
JOINTS = tuple(f"fr3_joint{index}" for index in range(1, 8))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Control a bounded FR3 session through the private Ubuntu socket."
    )
    parser.add_argument("--socket", type=Path, default=DEFAULT_SOCKET)
    parser.add_argument("--timeout", type=float, default=5.0)
    subparsers = parser.add_subparsers(dest="command", required=True)

    arm = subparsers.add_parser("arm", help="Locally arm one new bounded session.")
    arm.add_argument("--joint", choices=JOINTS, action="append", required=True)
    arm.add_argument("--max-moving-joints", type=int, default=1)
    arm.add_argument("--max-steps", type=int, default=4)
    arm.add_argument("--max-duration", type=float, default=60.0)
    arm.add_argument("--max-delta", type=float, default=0.03)
    arm.add_argument("--max-total-delta", type=float, default=0.03)
    arm.add_argument("--max-cumulative-delta", type=float, default=0.12)
    arm.add_argument("--session-id", default="", help=argparse.SUPPRESS)

    status = subparsers.add_parser("status", help="Inspect the current session.")
    status.add_argument(
        "--session-id",
        default="",
        help="Optional exact session UUID; defaults to the current session.",
    )

    disarm = subparsers.add_parser("disarm", help="Remove current session authority.")
    disarm.add_argument(
        "--session-id",
        default="",
        help="Optional exact session UUID; defaults to the current session.",
    )
    return parser.parse_args()


def validate_common(args: argparse.Namespace) -> None:
    if not math.isfinite(args.timeout) or args.timeout <= 0.0:
        raise RuntimeError("--timeout must be finite and positive.")


def parse_uuid4(value: str) -> str:
    try:
        parsed = uuid.UUID(value)
    except ValueError as error:
        raise RuntimeError("session ID must be a valid UUID.") from error
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
        raise RuntimeError("--max-cumulative-delta is smaller than --max-total-delta.")


def print_session(session: dict) -> None:
    limits = session.get("limits", {})
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
    print(f"Stop requested:   {session.get('stop_requested')}")
    if session.get("terminal_code"):
        print(f"Terminal code:    {session.get('terminal_code')}")
    if session.get("last_result"):
        print(f"Last result:      {json.dumps(session['last_result'], sort_keys=True)}")


def arm_session(args: argparse.Namespace) -> int:
    validate_arm_args(args)
    session_id = parse_uuid4(args.session_id) if args.session_id else str(uuid.uuid4())
    print("Bounded FR3 session request")
    print("")
    print(f"Session ID:              {session_id}")
    print(f"Allowed joints:          {args.joint}")
    print(f"Maximum moving joints:   {args.max_moving_joints}")
    print(f"Maximum actions:         {args.max_steps}")
    print(f"Maximum duration:        {args.max_duration:.1f} s")
    print(f"Maximum action delta:    {args.max_delta:.6f} rad")
    print(f"Maximum action total:    {args.max_total_delta:.6f} rad")
    print(f"Maximum cumulative delta:{args.max_cumulative_delta:>9.6f} rad")
    print("Actions in flight:       1")
    print("Automatic retries:       0")
    print("Any failure:             TERMINATE SESSION")

    expected = "ARM"
    confirmation = input(f"\nType exactly '{expected}' to arm: ").strip()
    if confirmation != expected:
        print("Session arm cancelled. No remote motion authority was created.")
        return 1

    response = send_local_request(
        args.socket,
        {
            "operation": "arm_session",
            "session_id": session_id,
            "allowed_joints": args.joint,
            "max_moving_joints": args.max_moving_joints,
            "max_steps": args.max_steps,
            "max_duration_seconds": args.max_duration,
            "max_delta": args.max_delta,
            "max_total_delta": args.max_total_delta,
            "max_cumulative_delta": args.max_cumulative_delta,
        },
        timeout=args.timeout,
    )
    if response.get("local_status") != 201:
        print(json.dumps(response, indent=2, sort_keys=True), file=sys.stderr)
        return 1

    print("\nSESSION ARMED")
    print_session(response)
    print("\nWindows can discover this fresh session automatically.")
    print("Run the coordinator with --run-armed-session; no UUID copy is required.")
    return 0


def session_status(args: argparse.Namespace) -> int:
    request = {"operation": "session_status"}
    if args.session_id:
        request["session_id"] = parse_uuid4(args.session_id)
    response = send_local_request(
        args.socket,
        request,
        timeout=args.timeout,
    )
    if response.get("local_status") != 200:
        print(json.dumps(response, indent=2, sort_keys=True), file=sys.stderr)
        return 1
    print_session(response)
    return 0


def disarm_session(args: argparse.Namespace) -> int:
    request = {"operation": "disarm_session"}
    if args.session_id:
        request["session_id"] = parse_uuid4(args.session_id)
    response = send_local_request(
        args.socket,
        request,
        timeout=args.timeout,
    )
    if response.get("local_status") != 200:
        print(json.dumps(response, indent=2, sort_keys=True), file=sys.stderr)
        return 1
    print("SESSION DISARM REQUESTED")
    session = response.get("session")
    if isinstance(session, dict):
        print_session(session)
    else:
        print(json.dumps(response, indent=2, sort_keys=True))
    return 0


def main() -> int:
    args = parse_args()
    try:
        validate_common(args)
        if args.command == "arm":
            return arm_session(args)
        if args.command == "status":
            return session_status(args)
        return disarm_session(args)
    except (EOFError, KeyboardInterrupt):
        print("\nSession control cancelled.")
        return 130
    except Exception as error:
        print(f"Session control error: {type(error).__name__}: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
