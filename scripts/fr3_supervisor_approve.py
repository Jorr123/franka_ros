#!/usr/bin/env python3
"""Inspect and locally approve one frozen FR3 supervisor plan."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import socket
import sys
import time
from typing import Any, Mapping


DEFAULT_SOCKET = Path("/tmp/fr3_ai_supervisor_approval.sock")
MAX_RESPONSE_BYTES = 64 * 1024
READY_STATUS = "READY_FOR_LOCAL_APPROVAL"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Inspect a frozen plan and optionally approve exactly one local "
            "MoveIt execution. This command must run on the Ubuntu robot PC."
        )
    )
    parser.add_argument("--request-id", required=True)
    parser.add_argument("--socket", type=Path, default=DEFAULT_SOCKET)
    parser.add_argument(
        "--action-sha256",
        default="",
        help="Action hash printed by the Windows plan response; required with --execute.",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Ask for local execution after displaying and confirming the frozen plan.",
    )
    parser.add_argument("--timeout", type=float, default=90.0)
    return parser.parse_args()


def send_local_request(
    socket_path: Path,
    request: Mapping[str, Any],
    *,
    timeout: float,
) -> dict[str, Any]:
    body = json.dumps(request, allow_nan=False, separators=(",", ":")).encode("utf-8")
    if len(body) > 4096:
        raise RuntimeError("Local request is unexpectedly large.")

    chunks: list[bytes] = []
    total = 0
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
            client.settimeout(timeout)
            client.connect(str(socket_path))
            client.sendall(body + b"\n")
            client.shutdown(socket.SHUT_WR)
            while True:
                chunk = client.recv(4096)
                if not chunk:
                    break
                total += len(chunk)
                if total > MAX_RESPONSE_BYTES:
                    raise RuntimeError("Supervisor response exceeded the size limit.")
                chunks.append(chunk)
                if b"\n" in chunk:
                    break
    except FileNotFoundError as error:
        raise RuntimeError(
            f"Approval socket does not exist: {socket_path}. Start the supervisor "
            "with --enable-local-execution."
        ) from error
    except (ConnectionError, OSError) as error:
        raise RuntimeError(f"Could not contact the local supervisor: {error}") from error

    response_bytes = b"".join(chunks).split(b"\n", 1)[0]
    try:
        response = json.loads(response_bytes.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise RuntimeError("Supervisor returned invalid JSON.") from error
    if not isinstance(response, dict):
        raise RuntimeError("Supervisor response must be a JSON object.")
    return response


def format_plan(status: Mapping[str, Any]) -> str:
    lines = [
        "Frozen plan",
        "",
        f"Status:        {status.get('status', 'UNKNOWN')}",
        f"Request ID:    {status.get('request_id', 'UNKNOWN')}",
        f"Action SHA256: {status.get('action_sha256', 'UNKNOWN')}",
    ]

    expires_ms = status.get("plan_expires_at_unix_ms")
    if isinstance(expires_ms, int):
        remaining = max(0.0, expires_ms / 1000.0 - time.time())
        local_expiry = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(expires_ms / 1000.0))
        lines.append(f"Expires:       {local_expiry} ({remaining:.1f} s remaining)")

    deltas = status.get("validated_deltas")
    if isinstance(deltas, dict):
        lines.extend(["", "Validated joint deltas:"])
        for joint, value in sorted(deltas.items()):
            lines.append(f"  {joint}: {float(value):+.6f} rad")

    current = status.get("current_joint_positions")
    target = status.get("target_joint_positions")
    if isinstance(current, dict) and isinstance(target, dict):
        lines.extend(["", "joint        planned start   frozen target   delta"])
        for joint in sorted(set(current) & set(target)):
            start = float(current[joint])
            goal = float(target[joint])
            lines.append(
                f"{joint:<11} {start:>13.6f}  {goal:>13.6f}  {goal - start:>+9.6f}"
            )

    planning = status.get("planning")
    if isinstance(planning, dict):
        lines.extend(
            [
                "",
                f"MoveIt result: {planning.get('result', 'UNKNOWN')}",
                f"Points:        {planning.get('point_count', 'UNKNOWN')}",
                f"Duration:      {planning.get('duration_seconds', 'UNKNOWN')} s",
            ]
        )

    result = status.get("result")
    if isinstance(result, dict):
        lines.extend(["", f"Recorded result: {json.dumps(result, sort_keys=True)}"])
    return "\n".join(lines)


def validate_args(args: argparse.Namespace) -> None:
    if not args.request_id.strip():
        raise RuntimeError("--request-id cannot be empty.")
    if not math.isfinite(args.timeout) or args.timeout <= 0.0:
        raise RuntimeError("--timeout must be finite and positive.")
    if args.execute:
        digest = args.action_sha256.strip().lower()
        if len(digest) != 64 or any(character not in "0123456789abcdef" for character in digest):
            raise RuntimeError("--action-sha256 must be a 64-character hexadecimal hash.")
        args.action_sha256 = digest


def main() -> int:
    args = parse_args()
    try:
        validate_args(args)
        status = send_local_request(
            args.socket,
            {"operation": "status", "request_id": args.request_id},
            timeout=min(args.timeout, 5.0),
        )
        if status.get("local_status") != 200:
            print(json.dumps(status, indent=2, sort_keys=True), file=sys.stderr)
            return 1

        print(format_plan(status))
        if not args.execute:
            print("\nInspection complete. The robot was not commanded.")
            return 0

        if status.get("status") != READY_STATUS:
            raise RuntimeError(
                f"Plan status is {status.get('status')!r}; expected {READY_STATUS}."
            )
        if status.get("action_sha256") != args.action_sha256:
            raise RuntimeError(
                "--action-sha256 does not match the hash stored with the frozen plan."
            )

        expected_confirmation = f"EXECUTE {args.request_id}"
        print("\nThis will command the real FR3 once through MoveIt.")
        confirmation = input(f"Type exactly '{expected_confirmation}' to continue: ").strip()
        if confirmation != expected_confirmation:
            print("Approval cancelled. The frozen plan was not executed.")
            return 1

        result = send_local_request(
            args.socket,
            {
                "operation": "execute",
                "request_id": args.request_id,
                "action_sha256": args.action_sha256,
            },
            timeout=args.timeout,
        )
        print("\nSupervisor execution result:")
        print(json.dumps(result, indent=2, sort_keys=True))
        return 0 if result.get("status") == "VERIFIED_SUCCESS" else 1
    except (EOFError, KeyboardInterrupt):
        print("\nApproval cancelled. The frozen plan was not executed.")
        return 130
    except Exception as error:
        print(f"Approval error: {type(error).__name__}: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
