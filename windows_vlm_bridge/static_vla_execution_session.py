"""Run a locally armed, bounded static-image FR3 execution session."""

from __future__ import annotations

import argparse
import copy
from dataclasses import dataclass
import math
from pathlib import Path
import sys
import time
from typing import Any, Callable, Mapping, Sequence
import urllib.parse
import uuid

from fr3_action_protocol import (
    DEFAULT_SECRET_ENV,
    SIGNATURE_HEADER,
    action_sha256,
    canonical_json_bytes,
    create_session_step_envelope,
    create_session_stop_envelope,
    load_shared_secret,
    make_signature,
)
from isaac_qwen_coordinator import (
    DEFAULT_MODEL_ID,
    DEFAULT_QWEN_URL,
    NoAction,
    call_qwen,
    load_image,
    post_json,
    validate_action,
)
from static_vla_shadow_session import JsonlEventLog, collect_images, get_json


DEFAULT_HEALTH_URL = "http://10.10.10.1:8080/health"
DEFAULT_STEP_URL = "http://10.10.10.1:8080/v1/sessions/step"
DEFAULT_STOP_URL = "http://10.10.10.1:8080/v1/sessions/stop"
DEFAULT_TASK = (
    "Inspect the image and report whether the red marker center is left or "
    "right of the vertical dashed center line. Report unclear if it cannot "
    "be determined."
)
TERMINAL_SESSION_STATUSES = {"STOPPED", "DISARMED", "COMPLETED", "EXPIRED", "FAULT"}
HARD_MAX_IMAGES = 20
FR3_JOINTS = tuple(f"fr3_joint{index}" for index in range(1, 8))


@dataclass(frozen=True)
class InferenceResult:
    action: dict[str, Any]
    image_sha256: str
    mime_type: str


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Process static images one at a time inside a session that was "
            "explicitly armed on Ubuntu. There are no automatic retries."
        )
    )
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--image", type=Path, action="append")
    source.add_argument("--image-dir", type=Path)
    parser.add_argument(
        "--session-id",
        default="",
        help=(
            "Optional exact Ubuntu session UUID. When omitted, armed mode "
            "automatically discovers the single fresh armed session."
        ),
    )
    parser.add_argument("--task", default=DEFAULT_TASK)
    parser.add_argument("--model-id", default=DEFAULT_MODEL_ID)
    parser.add_argument("--qwen-url", default=DEFAULT_QWEN_URL)
    parser.add_argument("--ubuntu-health-url", default=DEFAULT_HEALTH_URL)
    parser.add_argument("--ubuntu-step-url", default=DEFAULT_STEP_URL)
    parser.add_argument("--ubuntu-stop-url", default=DEFAULT_STOP_URL)
    parser.add_argument("--qwen-timeout", type=float, default=180.0)
    parser.add_argument("--ubuntu-timeout", type=float, default=90.0)
    parser.add_argument("--shared-secret-env", default=DEFAULT_SECRET_ENV)

    parser.add_argument("--min-confidence", type=float, default=0.70)
    parser.add_argument("--max-delta", type=float, default=0.03)
    parser.add_argument("--max-total-delta", type=float, default=0.03)
    parser.add_argument("--max-moving-joints", type=int, default=1)
    parser.add_argument(
        "--expected-joint",
        choices=FR3_JOINTS,
        default="fr3_joint3",
        help="Expected joint for local-inference mode; armed mode uses Ubuntu's limit.",
    )
    parser.add_argument("--max-images", type=int, default=5)
    parser.add_argument("--max-session-seconds", type=float, default=60.0)
    parser.add_argument("--interval", type=float, default=0.5)
    parser.add_argument("--log-file", type=Path)
    parser.add_argument(
        "--run-armed-session",
        action="store_true",
        help=(
            "Send actions to the locally armed Ubuntu session. Without this "
            "flag, images are inferred and validated but the robot cannot move."
        ),
    )
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    if not args.task.strip():
        raise RuntimeError("--task cannot be empty.")
    for name in (
        "qwen_timeout",
        "ubuntu_timeout",
        "max_delta",
        "max_total_delta",
        "max_session_seconds",
    ):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0.0:
            raise RuntimeError(f"--{name.replace('_', '-')} must be finite and positive.")
    if not math.isfinite(args.interval) or args.interval < 0.0:
        raise RuntimeError("--interval must be finite and non-negative.")
    if not 0.0 < args.min_confidence <= 1.0:
        raise RuntimeError("--min-confidence must be in (0, 1].")
    if not 1 <= args.max_moving_joints <= len(FR3_JOINTS):
        raise RuntimeError("--max-moving-joints must be between 1 and 7.")
    if args.max_total_delta > args.max_delta * args.max_moving_joints + 1e-12:
        raise RuntimeError("--max-total-delta exceeds the per-joint action budget.")
    if not 1 <= args.max_images <= HARD_MAX_IMAGES:
        raise RuntimeError(f"--max-images must be between 1 and {HARD_MAX_IMAGES}.")
    validate_endpoints(args.ubuntu_health_url, args.ubuntu_step_url, args.ubuntu_stop_url)
    if args.session_id:
        validate_session_id(args.session_id)


def validate_session_id(value: Any) -> str:
    if not isinstance(value, str):
        raise RuntimeError("Session ID must be a canonical UUIDv4.")
    try:
        parsed = uuid.UUID(value)
    except ValueError as error:
        raise RuntimeError("Session ID must be a canonical UUIDv4.") from error
    if parsed.version != 4 or str(parsed) != value.lower():
        raise RuntimeError("Session ID must be a canonical UUIDv4.")
    return value


def validate_endpoints(health_url: str, step_url: str, stop_url: str) -> None:
    parsed = [urllib.parse.urlsplit(url) for url in (health_url, step_url, stop_url)]
    expected_paths = ("/health", "/v1/sessions/step", "/v1/sessions/stop")
    for value, expected_path in zip(parsed, expected_paths):
        if value.scheme not in {"http", "https"} or not value.hostname:
            raise RuntimeError("Ubuntu session URLs must be absolute HTTP(S) URLs.")
        if value.username or value.password or value.query or value.fragment:
            raise RuntimeError("Ubuntu session URLs cannot contain credentials or parameters.")
        if value.path != expected_path:
            raise RuntimeError(f"Ubuntu URL path must be exactly {expected_path}.")
    authorities = {(value.scheme, value.hostname, value.port) for value in parsed}
    if len(authorities) != 1:
        raise RuntimeError("Ubuntu health, step, and stop URLs must use the same server.")


def validate_armed_health(
    status: int,
    health: Any,
    *,
    expected_session_id: str | None,
    args: argparse.Namespace,
) -> dict[str, Any]:
    if status != 200 or not isinstance(health, dict):
        raise RuntimeError(f"Ubuntu health failed with HTTP {status}: {health}")
    if health.get("mode") != "BOUNDED_SESSION_EXECUTION":
        raise RuntimeError("Ubuntu is not running the bounded-session supervisor.")
    if health.get("remote_direct_execution_available") is not False:
        raise RuntimeError("Ubuntu did not explicitly disable direct remote execution.")
    if health.get("session_step_available") is not True:
        raise RuntimeError("Ubuntu did not expose the bounded session-step interface.")
    if health.get("multi_joint_bounded_sessions") is not True:
        raise RuntimeError("Ubuntu does not advertise multi-joint bounded sessions.")
    if health.get("schema_version") != 2:
        raise RuntimeError("Ubuntu does not advertise bounded-session schema version 2.")
    if health.get("session_armed") is not True:
        raise RuntimeError("No bounded session is armed on Ubuntu.")
    session = health.get("session")
    if not isinstance(session, dict):
        raise RuntimeError("Ubuntu armed session details are missing.")
    discovered_session_id = validate_session_id(session.get("session_id"))
    if expected_session_id is not None and discovered_session_id != expected_session_id:
        raise RuntimeError("Ubuntu armed session ID does not match --session-id.")
    if session.get("status") != "ARMED":
        raise RuntimeError(f"Ubuntu session status is {session.get('status')!r}, not ARMED.")
    if session.get("next_sequence") != 1 or session.get("completed_steps") != 0:
        raise RuntimeError("Execution coordinator only starts a fresh session at sequence 1.")
    limits = session.get("limits")
    if not isinstance(limits, dict):
        raise RuntimeError("Ubuntu session limits are missing.")
    if args.max_delta > float(limits.get("max_delta", 0.0)) + 1e-12:
        raise RuntimeError("Windows --max-delta exceeds the armed Ubuntu limit.")
    if args.max_total_delta > float(limits.get("max_total_delta", 0.0)) + 1e-12:
        raise RuntimeError("Windows --max-total-delta exceeds the armed Ubuntu limit.")
    if args.max_moving_joints > int(limits.get("max_moving_joints", 0)):
        raise RuntimeError("Windows --max-moving-joints exceeds the armed Ubuntu limit.")
    requested_cumulative = getattr(args, "max_cumulative_delta", None)
    if (
        requested_cumulative is not None
        and requested_cumulative
        > float(limits.get("max_cumulative_delta", 0.0)) + 1e-12
    ):
        raise RuntimeError(
            "Windows --max-cumulative-delta exceeds the armed Ubuntu limit."
        )
    return session


def validate_action_for_session(
    action: Mapping[str, Any],
    args: argparse.Namespace,
    session: Mapping[str, Any],
    cumulative_delta: float,
) -> tuple[dict[str, float], float]:
    validate_action(action, args)
    limits = session["limits"]
    allowed_joints = limits["allowed_joints"]
    moving = {
        str(joint): float(value)
        for joint, value in action["deltas"].items()
        if abs(float(value)) > 1e-9
    }
    if not set(moving).issubset(allowed_joints):
        raise RuntimeError(
            f"Armed session permits {allowed_joints}; action uses {sorted(moving)}."
        )
    if len(moving) > int(limits["max_moving_joints"]):
        raise RuntimeError("Action exceeds the armed moving-joint limit.")
    for joint, delta in moving.items():
        if abs(delta) > float(limits["max_delta"]) + 1e-12:
            raise RuntimeError(f"{joint} exceeds the armed per-joint limit.")
    step_total = sum(abs(delta) for delta in moving.values())
    if step_total > float(limits["max_total_delta"]) + 1e-12:
        raise RuntimeError("Action exceeds the armed per-step total limit.")
    if cumulative_delta + step_total > float(limits["max_cumulative_delta"]) + 1e-12:
        raise RuntimeError("Action would exceed the armed cumulative limit.")
    return moving, step_total


def validate_step_receipt(
    status: int,
    response: Any,
    *,
    session_id: str,
    sequence: int,
    request_id: str,
    action_digest: str,
    requested_deltas: Mapping[str, float],
) -> dict[str, Any]:
    if status != 200 or not isinstance(response, dict):
        raise RuntimeError(f"Ubuntu session step failed with HTTP {status}: {response}")
    expected = {
        "status": "VERIFIED_SUCCESS",
        "session_id": session_id,
        "sequence": sequence,
        "request_id": request_id,
        "action_sha256": action_digest,
    }
    for key, value in expected.items():
        if response.get(key) != value:
            raise RuntimeError(f"Ubuntu receipt mismatch for {key}.")
    session = response.get("session")
    if not isinstance(session, dict):
        raise RuntimeError("Ubuntu receipt is missing session state.")
    if session.get("next_sequence") != sequence + 1:
        raise RuntimeError("Ubuntu next sequence does not follow the verified step.")
    if session.get("completed_steps") != sequence:
        raise RuntimeError("Ubuntu completed-step count does not match the sequence.")
    if session.get("status") not in {"ARMED", "COMPLETED", "STOPPED", "EXPIRED"}:
        raise RuntimeError(f"Unexpected post-step session status: {session.get('status')}")
    execution = response.get("execution")
    if not isinstance(execution, dict) or execution.get("status") != "VERIFIED_SUCCESS":
        raise RuntimeError("Ubuntu receipt is missing VERIFIED_SUCCESS execution.")
    verification = execution.get("verification")
    if not isinstance(verification, dict) or verification.get("status") != "PASSED":
        raise RuntimeError("Ubuntu receipt is missing passed measured verification.")
    commanded = verification.get("commanded_joints")
    if not isinstance(commanded, list):
        raise RuntimeError("Ubuntu receipt is missing commanded-joint verification.")
    results: dict[str, Mapping[str, Any]] = {}
    for result in commanded:
        if not isinstance(result, dict) or not isinstance(result.get("joint"), str):
            raise RuntimeError("Ubuntu receipt contains an invalid commanded joint.")
        joint = result["joint"]
        if joint in results:
            raise RuntimeError("Ubuntu receipt contains duplicate commanded joints.")
        results[joint] = result
    if set(results) != set(requested_deltas):
        raise RuntimeError("Ubuntu verified joint set does not match the submitted action.")
    for joint, requested_delta in requested_deltas.items():
        result = results[joint]
        for field in ("direction_ok", "movement_ok", "target_ok"):
            if result.get(field) is not True:
                raise RuntimeError(f"Ubuntu verification {field} failed for {joint}.")
        try:
            reported_delta = float(result.get("requested_delta"))
        except (TypeError, ValueError) as error:
            raise RuntimeError("Ubuntu receipt contains an invalid requested delta.") from error
        if not math.isfinite(reported_delta) or abs(reported_delta - requested_delta) > 1e-9:
            raise RuntimeError(f"Ubuntu requested delta does not match for {joint}.")
    if verification.get("non_commanded_drift_ok") is not True:
        raise RuntimeError("Ubuntu non-commanded drift verification failed.")
    if verification.get("settled_velocity_ok") is not True:
        raise RuntimeError("Ubuntu settled-velocity verification failed.")
    return response


def make_inference_function(
    args: argparse.Namespace,
) -> Callable[[Path, float], InferenceResult]:
    def infer(image: Path, remaining_seconds: float) -> InferenceResult:
        image_data_url, digest, mime_type = load_image(image)
        step_args = copy.copy(args)
        step_args.qwen_timeout = min(args.qwen_timeout, remaining_seconds)
        action = call_qwen(step_args, image_data_url)
        return InferenceResult(action=action, image_sha256=digest, mime_type=mime_type)

    return infer


def make_step_function(
    args: argparse.Namespace,
    secret: bytes,
    session_id: str,
    *,
    frame_source: str = "windows-static-execution-session",
) -> Callable[[Mapping[str, Any], str, str, int, float], dict[str, Any]]:
    def submit(
        action: Mapping[str, Any],
        image_digest: str,
        frame_id: str,
        sequence: int,
        remaining_seconds: float,
    ) -> dict[str, Any]:
        envelope = create_session_step_envelope(
            action,
            session_id=session_id,
            sequence=sequence,
            frame_source=frame_source,
            frame_id=frame_id,
            frame_sha256=image_digest,
        )
        body = canonical_json_bytes(envelope)
        status, response = post_json(
            args.ubuntu_step_url,
            body,
            timeout=min(args.ubuntu_timeout, remaining_seconds),
            headers={SIGNATURE_HEADER: make_signature(body, secret)},
        )
        return validate_step_receipt(
            status,
            response,
            session_id=session_id,
            sequence=sequence,
            request_id=envelope["request_id"],
            action_digest=action_sha256(action),
            requested_deltas={
                str(joint): float(delta)
                for joint, delta in action["deltas"].items()
                if abs(float(delta)) > 1e-9
            },
        )

    return submit


def send_stop(
    args: argparse.Namespace,
    secret: bytes,
    session_id: str,
    *,
    timeout: float,
) -> dict[str, Any]:
    envelope = create_session_stop_envelope(session_id)
    body = canonical_json_bytes(envelope)
    status, response = post_json(
        args.ubuntu_stop_url,
        body,
        timeout=min(args.ubuntu_timeout, timeout),
        headers={SIGNATURE_HEADER: make_signature(body, secret)},
    )
    if status != 200 or not isinstance(response, dict):
        raise RuntimeError(f"Ubuntu stop request failed with HTTP {status}: {response}")
    if response.get("request_id") != envelope["request_id"]:
        raise RuntimeError("Ubuntu stop receipt request ID mismatch.")
    session = response.get("session")
    if not isinstance(session, dict) or session.get("session_id") != session_id:
        raise RuntimeError("Ubuntu stop receipt session ID mismatch.")
    return response


def run_execution_session(
    args: argparse.Namespace,
    images: Sequence[Path],
    session: Mapping[str, Any],
    *,
    event_sink: Callable[[Mapping[str, Any]], None],
    infer: Callable[[Path, float], InferenceResult],
    submit_step: Callable[[Mapping[str, Any], str, str, int, float], dict[str, Any]]
    | None,
    stop_session: Callable[[float], dict[str, Any]] | None,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> tuple[int, dict[str, Any]]:
    session_id = str(session["session_id"])
    selected_images = list(images[: args.max_images])
    start = monotonic()
    armed_remaining = float(session.get("remaining_seconds", args.max_session_seconds))
    deadline_seconds = min(args.max_session_seconds, armed_remaining)
    sequence = 1
    verified_steps = 0
    no_action_steps = 0
    cumulative_delta = 0.0
    final_status = "COMPLETED_LOCAL_INFERENCE"
    failure = ""
    last_server_status = "ARMED"

    event_sink(
        {
            "event": "EXECUTION_SESSION_STARTED",
            "session_id": session_id,
            "run_armed_session": submit_step is not None,
            "selected_image_count": len(selected_images),
            "deadline_seconds": deadline_seconds,
            "limits": session["limits"],
        }
    )

    for image_index, image in enumerate(selected_images, start=1):
        remaining = deadline_seconds - (monotonic() - start)
        if remaining <= 0.0:
            final_status = "SESSION_TIMEOUT"
            failure = "Local session deadline elapsed."
            break
        frame_id = f"execute-{session_id}-seq-{sequence:04d}"
        event_sink(
            {
                "event": "IMAGE_STARTED",
                "session_id": session_id,
                "image_index": image_index,
                "sequence": sequence,
                "image": str(image),
            }
        )

        try:
            inferred = infer(image, remaining)
            moving, step_total = validate_action_for_session(
                inferred.action,
                args,
                session,
                cumulative_delta,
            )
        except NoAction as error:
            no_action_steps += 1
            final_status = "TASK_COMPLETE_NO_ACTION"
            event_sink(
                {
                    "event": "NO_ACTION",
                    "session_id": session_id,
                    "image_index": image_index,
                    "reason": str(error),
                }
            )
            break
        except Exception as error:
            final_status = "FAILED"
            failure = f"{type(error).__name__}: {error}"
            event_sink(
                {
                    "event": "INFERENCE_OR_VALIDATION_FAILED",
                    "session_id": session_id,
                    "image_index": image_index,
                    "failure": failure,
                }
            )
            break

        event_sink(
            {
                "event": "ACTION_VALIDATED",
                "session_id": session_id,
                "image_index": image_index,
                "sequence": sequence,
                "image_sha256": inferred.image_sha256,
                "action": inferred.action,
                "moving_deltas": moving,
                "step_total_delta": step_total,
                "cumulative_if_verified": cumulative_delta + step_total,
            }
        )

        if submit_step is not None:
            try:
                receipt = submit_step(
                    inferred.action,
                    inferred.image_sha256,
                    frame_id,
                    sequence,
                    remaining,
                )
            except Exception as error:
                final_status = "FAILED"
                failure = f"{type(error).__name__}: {error}"
                event_sink(
                    {
                        "event": "STEP_FAILED",
                        "session_id": session_id,
                        "sequence": sequence,
                        "failure": failure,
                    }
                )
                break
            expected_cumulative = cumulative_delta + step_total
            try:
                reported_cumulative = float(
                    receipt["session"]["cumulative_absolute_delta"]
                )
            except (KeyError, TypeError, ValueError) as error:
                final_status = "FAILED"
                failure = "RuntimeError: receipt cumulative delta is missing or invalid"
                event_sink(
                    {
                        "event": "STEP_FAILED",
                        "session_id": session_id,
                        "sequence": sequence,
                        "failure": failure,
                    }
                )
                break
            if (
                not math.isfinite(reported_cumulative)
                or abs(reported_cumulative - expected_cumulative) > 1e-9
            ):
                final_status = "FAILED"
                failure = "RuntimeError: receipt cumulative delta does not match the action"
                event_sink(
                    {
                        "event": "STEP_FAILED",
                        "session_id": session_id,
                        "sequence": sequence,
                        "failure": failure,
                    }
                )
                break
            verified_steps += 1
            cumulative_delta = expected_cumulative
            last_server_status = receipt["session"]["status"]
            event_sink(
                {
                    "event": "STEP_VERIFIED",
                    "session_id": session_id,
                    "sequence": sequence,
                    "request_id": receipt["request_id"],
                    "action_sha256": receipt["action_sha256"],
                    "session_status": last_server_status,
                    "planning": receipt.get("planning"),
                    "execution": receipt.get("execution"),
                }
            )
            sequence += 1
            if last_server_status in TERMINAL_SESSION_STATUSES:
                final_status = f"SESSION_{last_server_status}"
                break
        else:
            cumulative_delta += step_total
            verified_steps += 1
            sequence += 1

        if image_index < len(selected_images) and args.interval > 0.0:
            remaining = deadline_seconds - (monotonic() - start)
            if remaining <= 0.0:
                final_status = "SESSION_TIMEOUT"
                failure = "Local session deadline elapsed."
                break
            sleep(min(args.interval, remaining))

    if submit_step is not None and last_server_status not in TERMINAL_SESSION_STATUSES:
        stop_reason = "normal completion" if final_status != "FAILED" else "failure"
        try:
            remaining = max(1.0, deadline_seconds - (monotonic() - start))
            stop_receipt = stop_session(remaining) if stop_session is not None else None
            event_sink(
                {
                    "event": "STOP_CONFIRMED",
                    "session_id": session_id,
                    "reason": stop_reason,
                    "receipt": stop_receipt,
                }
            )
            if final_status == "COMPLETED_LOCAL_INFERENCE":
                final_status = "INPUT_EXHAUSTED_STOPPED"
        except Exception as error:
            stop_failure = f"{type(error).__name__}: {error}"
            event_sink(
                {
                    "event": "STOP_UNCONFIRMED",
                    "session_id": session_id,
                    "failure": stop_failure,
                }
            )
            failure = f"{failure}; stop unconfirmed: {stop_failure}".strip("; ")
            final_status = "FAILED_STOP_UNCONFIRMED"

    summary = {
        "event": "EXECUTION_SESSION_FINISHED",
        "session_id": session_id,
        "status": final_status,
        "verified_steps": verified_steps,
        "no_action_steps": no_action_steps,
        "cumulative_absolute_delta": cumulative_delta,
        "elapsed_seconds": monotonic() - start,
        "failure": failure or None,
        "robot_execution_enabled": submit_step is not None,
    }
    event_sink(summary)
    exit_code = 0 if final_status in {
        "COMPLETED_LOCAL_INFERENCE",
        "TASK_COMPLETE_NO_ACTION",
        "SESSION_COMPLETED",
        "SESSION_STOPPED",
        "INPUT_EXHAUSTED_STOPPED",
    } else 1
    return exit_code, summary


def default_log_path() -> Path:
    timestamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())
    return Path("session_logs") / f"static_execution_{timestamp}.jsonl"


def print_summary(summary: Mapping[str, Any], path: Path) -> None:
    print("\nStatic VLA execution session finished")
    print(f"Session ID:          {summary['session_id']}")
    print(f"Status:              {summary['status']}")
    print(f"Verified steps:      {summary['verified_steps']}")
    print(f"No-action decisions: {summary['no_action_steps']}")
    print(f"Cumulative delta:    {summary['cumulative_absolute_delta']:.6f} rad")
    print(f"Elapsed:             {summary['elapsed_seconds']:.3f} s")
    print(f"Robot execution:     {summary['robot_execution_enabled']}")
    print(f"Log:                 {path}")
    if summary.get("failure"):
        print(f"Failure:             {summary['failure']}")


def main() -> int:
    args = parse_args()
    event_log = None
    try:
        validate_args(args)
        images = collect_images(args)
        log_path = args.log_file or default_log_path()
        event_log = JsonlEventLog(log_path)
        infer = make_inference_function(args)

        if args.run_armed_session:
            secret = load_shared_secret(args.shared_secret_env)
            health_status, health = get_json(
                args.ubuntu_health_url,
                timeout=min(args.ubuntu_timeout, 10.0),
            )
            session = validate_armed_health(
                health_status,
                health,
                expected_session_id=args.session_id or None,
                args=args,
            )
            session_id = session["session_id"]
            submit_step = make_step_function(args, secret, session_id)
            stop_session = lambda timeout: send_stop(
                args,
                secret,
                session_id,
                timeout=timeout,
            )
            if args.session_id:
                print("Ubuntu explicit locally armed session confirmed.")
            else:
                print(f"Ubuntu armed session discovered: {session_id}")
            print("ROBOT EXECUTION IS ENABLED FOR THIS BOUNDED SESSION.")
        else:
            session = {
                "session_id": args.session_id or "00000000-0000-4000-8000-000000000000",
                "remaining_seconds": args.max_session_seconds,
                "limits": {
                    "allowed_joints": [args.expected_joint],
                    "max_moving_joints": 1,
                    "max_steps": args.max_images,
                    "max_duration_seconds": args.max_session_seconds,
                    "max_delta": args.max_delta,
                    "max_total_delta": args.max_total_delta,
                    "max_cumulative_delta": args.max_total_delta * args.max_images,
                },
            }
            submit_step = None
            stop_session = None
            print("LOCAL INFERENCE ONLY. Ubuntu will not be contacted and the robot cannot move.")

        exit_code, summary = run_execution_session(
            args,
            images,
            session,
            event_sink=event_log.write,
            infer=infer,
            submit_step=submit_step,
            stop_session=stop_session,
        )
        print_summary(summary, log_path)
        return exit_code
    except Exception as error:
        print(f"Execution session error: {type(error).__name__}: {error}", file=sys.stderr)
        return 2
    finally:
        if event_log is not None:
            event_log.close()


if __name__ == "__main__":
    sys.exit(main())
