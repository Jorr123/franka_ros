"""Validate bounded FR3 joint-action files and optionally submit an armed session."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import time
from typing import Any, Mapping

from fr3_action_protocol import DEFAULT_SECRET_ENV, load_shared_secret
from static_vla_execution_session import (
    DEFAULT_HEALTH_URL,
    DEFAULT_STEP_URL,
    DEFAULT_STOP_URL,
    FR3_JOINTS,
    TERMINAL_SESSION_STATUSES,
    get_json,
    make_step_function,
    send_stop,
    validate_armed_health,
    validate_endpoints,
    validate_session_id,
)
from static_vla_shadow_session import JsonlEventLog


HARD_MAX_ACTION_FILES = 20


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate named relative-joint action files for a bounded FR3 session."
    )
    parser.add_argument("--action-json-file", type=Path, action="append", required=True)
    parser.add_argument("--session-id", default="")
    parser.add_argument("--ubuntu-health-url", default=DEFAULT_HEALTH_URL)
    parser.add_argument("--ubuntu-step-url", default=DEFAULT_STEP_URL)
    parser.add_argument("--ubuntu-stop-url", default=DEFAULT_STOP_URL)
    parser.add_argument("--ubuntu-timeout", type=float, default=90.0)
    parser.add_argument("--shared-secret-env", default=DEFAULT_SECRET_ENV)
    parser.add_argument("--min-confidence", type=float, default=0.70)
    parser.add_argument("--max-moving-joints", type=int, default=2)
    parser.add_argument("--max-delta", type=float, default=0.01)
    parser.add_argument("--max-total-delta", type=float, default=0.02)
    parser.add_argument("--max-cumulative-delta", type=float, default=0.04)
    parser.add_argument("--max-session-seconds", type=float, default=50.0)
    parser.add_argument("--log-file", type=Path)
    parser.add_argument(
        "--run-armed-session",
        action="store_true",
        help="Submit actions to one fresh Ubuntu session. Without this flag no network is used.",
    )
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    if not 1 <= len(args.action_json_file) <= HARD_MAX_ACTION_FILES:
        raise RuntimeError(
            f"--action-json-file count must be between 1 and {HARD_MAX_ACTION_FILES}."
        )
    if not 1 <= args.max_moving_joints <= len(FR3_JOINTS):
        raise RuntimeError("--max-moving-joints must be between 1 and 7.")
    for name in (
        "ubuntu_timeout",
        "max_delta",
        "max_total_delta",
        "max_cumulative_delta",
        "max_session_seconds",
    ):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0.0:
            raise RuntimeError(f"--{name.replace('_', '-')} must be finite and positive.")
    if not 0.0 < args.min_confidence <= 1.0:
        raise RuntimeError("--min-confidence must be in (0, 1].")
    if args.max_total_delta > args.max_delta * args.max_moving_joints + 1e-12:
        raise RuntimeError("--max-total-delta exceeds the per-joint action budget.")
    if args.max_cumulative_delta + 1e-12 < args.max_total_delta:
        raise RuntimeError("--max-cumulative-delta is smaller than --max-total-delta.")
    if args.session_id:
        validate_session_id(args.session_id)
    validate_endpoints(
        args.ubuntu_health_url,
        args.ubuntu_step_url,
        args.ubuntu_stop_url,
    )


def _reject_constant(value: str) -> None:
    raise ValueError(f"Non-standard JSON constant {value} is not allowed.")


def load_action_file(path: Path) -> tuple[dict[str, Any], bytes, str]:
    try:
        raw = path.read_bytes()
    except OSError as error:
        raise RuntimeError(f"Could not read {path}: {error}") from error
    try:
        action = json.loads(raw.decode("utf-8"), parse_constant=_reject_constant)
    except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as error:
        raise RuntimeError(f"Invalid action JSON in {path}: {error}") from error
    if not isinstance(action, dict):
        raise RuntimeError(f"{path} must contain exactly one JSON object.")
    return action, raw, hashlib.sha256(raw).hexdigest()


def validate_action(
    action: Mapping[str, Any], args: argparse.Namespace
) -> tuple[dict[str, float], float]:
    if set(action) != {"action", "deltas", "confidence"}:
        raise RuntimeError("Action keys must be exactly action, deltas, and confidence.")
    if action.get("action") != "joint_deltas":
        raise RuntimeError("Only action='joint_deltas' is accepted.")
    confidence = action.get("confidence")
    if isinstance(confidence, bool):
        raise RuntimeError("confidence must be numeric.")
    try:
        parsed_confidence = float(confidence)
    except (TypeError, ValueError) as error:
        raise RuntimeError("confidence must be numeric.") from error
    if not math.isfinite(parsed_confidence) or parsed_confidence < args.min_confidence:
        raise RuntimeError("Action confidence is below the configured threshold.")
    deltas = action.get("deltas")
    if not isinstance(deltas, dict):
        raise RuntimeError("deltas must be an object.")
    moving: dict[str, float] = {}
    for joint, value in deltas.items():
        if joint not in FR3_JOINTS:
            raise RuntimeError(f"Unknown FR3 joint: {joint}")
        if isinstance(value, bool):
            raise RuntimeError(f"{joint} delta must be numeric.")
        try:
            delta = float(value)
        except (TypeError, ValueError) as error:
            raise RuntimeError(f"{joint} delta must be numeric.") from error
        if not math.isfinite(delta):
            raise RuntimeError(f"{joint} delta must be finite.")
        if abs(delta) > 1e-9:
            moving[joint] = delta
    if not moving:
        raise RuntimeError("Action must contain at least one non-zero delta.")
    if len(moving) > args.max_moving_joints:
        raise RuntimeError("Action exceeds --max-moving-joints.")
    for joint, delta in moving.items():
        if abs(delta) > args.max_delta + 1e-12:
            raise RuntimeError(f"{joint} exceeds --max-delta.")
    step_total = sum(abs(delta) for delta in moving.values())
    if step_total > args.max_total_delta + 1e-12:
        raise RuntimeError("Action exceeds --max-total-delta.")
    return moving, step_total


def default_log_path() -> Path:
    stamp = time.strftime("%Y%m%d_%H%M%S")
    return Path("session_logs") / f"bounded_joint_actions_{stamp}.jsonl"


def _run_armed_session(
    args: argparse.Namespace,
    validated: list[tuple[dict[str, Any], str, dict[str, float], float, Path]],
    log_path: Path,
    event_log: JsonlEventLog,
) -> tuple[int, dict[str, Any]]:
    secret = load_shared_secret(args.shared_secret_env)
    status, health = get_json(
        args.ubuntu_health_url, timeout=min(args.ubuntu_timeout, 10.0)
    )
    session = validate_armed_health(
        status,
        health,
        expected_session_id=args.session_id or None,
        args=args,
    )
    allowed = set(session["limits"]["allowed_joints"])
    for _action, _digest, moving, _total, _path in validated:
        if not set(moving).issubset(allowed):
            raise RuntimeError("An action uses a joint outside the armed allowlist.")
    if len(validated) > int(session["limits"]["max_steps"]):
        raise RuntimeError("Action count exceeds the armed max_steps limit.")

    session_id = str(session["session_id"])
    submit = make_step_function(
        args,
        secret,
        session_id,
        frame_source="windows-manual-joint-action-file",
    )
    start = time.monotonic()
    verified_cumulative = 0.0
    verified_steps = 0
    last_status = "ARMED"
    try:
        for sequence, (action, digest, moving, step_total, path) in enumerate(
            validated, start=1
        ):
            remaining = args.max_session_seconds - (time.monotonic() - start)
            if remaining <= 0.0:
                raise RuntimeError("Local bounded-session deadline elapsed.")
            receipt = submit(
                action,
                digest,
                f"manual-action-{path.name}-seq-{sequence:04d}",
                sequence,
                remaining,
            )
            expected_cumulative = verified_cumulative + step_total
            reported_cumulative = float(
                receipt["session"].get("cumulative_absolute_delta", float("nan"))
            )
            if (
                not math.isfinite(reported_cumulative)
                or abs(reported_cumulative - expected_cumulative) > 1e-9
            ):
                raise RuntimeError("Ubuntu cumulative receipt does not match the action.")
            verified_cumulative = expected_cumulative
            verified_steps += 1
            last_status = str(receipt["session"]["status"])
            event_log.write(
                {
                    "event": "STEP_VERIFIED",
                    "session_id": session_id,
                    "sequence": sequence,
                    "moving_deltas": moving,
                    "step_total_delta": step_total,
                    "cumulative_delta": verified_cumulative,
                    "session_status": last_status,
                }
            )
            if last_status in TERMINAL_SESSION_STATUSES:
                break
    except Exception:
        if last_status not in TERMINAL_SESSION_STATUSES:
            try:
                send_stop(args, secret, session_id, timeout=min(args.ubuntu_timeout, 10.0))
            except Exception:
                pass
        raise

    if last_status not in TERMINAL_SESSION_STATUSES:
        send_stop(args, secret, session_id, timeout=min(args.ubuntu_timeout, 10.0))
    return 0, {
        "status": "SESSION_COMPLETED" if verified_steps == len(validated) else last_status,
        "actions_validated": len(validated),
        "verified_steps": verified_steps,
        "cumulative_delta": verified_cumulative,
        "ubuntu_contacted": True,
        "robot_execution": True,
        "log_file": str(log_path),
    }


def run(args: argparse.Namespace) -> tuple[int, dict[str, Any]]:
    validate_args(args)
    loaded = [load_action_file(path) for path in args.action_json_file]
    validated: list[tuple[dict[str, Any], str, dict[str, float], float, Path]] = []
    cumulative = 0.0
    for path, (action, _raw, digest) in zip(args.action_json_file, loaded):
        moving, step_total = validate_action(action, args)
        cumulative += step_total
        if cumulative > args.max_cumulative_delta + 1e-12:
            raise RuntimeError("Actions exceed --max-cumulative-delta.")
        validated.append((action, digest, moving, step_total, path))

    log_path = args.log_file or default_log_path()
    event_log = JsonlEventLog(log_path)
    try:
        event_log.write(
            {
                "event": "ACTION_FILES_VALIDATED",
                "action_count": len(validated),
                "proposed_cumulative_delta": cumulative,
                "run_armed_session": args.run_armed_session,
            }
        )
        if not args.run_armed_session:
            return 0, {
                "status": "DRY_RUN_COMPLETE",
                "actions_validated": len(validated),
                "cumulative_delta": cumulative,
                "ubuntu_contacted": False,
                "robot_execution": False,
                "log_file": str(log_path),
            }
        return _run_armed_session(args, validated, log_path, event_log)
    finally:
        event_log.close()


def print_summary(summary: Mapping[str, Any], args: argparse.Namespace) -> None:
    print("\nBounded joint-action session finished")
    print(f"Status:                    {summary['status']}")
    print(f"Actions validated:         {summary['actions_validated']}")
    print(f"Maximum moving joints:     {args.max_moving_joints}")
    print(f"Maximum per-joint delta:   {args.max_delta:.6f} rad")
    print(f"Maximum step total:        {args.max_total_delta:.6f} rad")
    print(f"Cumulative proposed/done:  {summary['cumulative_delta']:.6f} rad")
    print(f"Ubuntu contacted:          {summary['ubuntu_contacted']}")
    print(f"Robot execution:           {summary['robot_execution']}")
    print(f"Log:                       {summary['log_file']}")


def main() -> int:
    args = parse_args()
    try:
        code, summary = run(args)
        print_summary(summary, args)
        return code
    except (KeyboardInterrupt, EOFError):
        print("\nBounded joint-action session cancelled.")
        return 130
    except Exception as error:
        print(
            f"Bounded joint-action session error: {type(error).__name__}: {error}",
            file=sys.stderr,
        )
        return 2


if __name__ == "__main__":
    sys.exit(main())
