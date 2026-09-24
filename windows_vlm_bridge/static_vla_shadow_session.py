"""Run a bounded, plan-only static-image VLA session from Windows."""

from __future__ import annotations

import argparse
import copy
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys
import time
from typing import Any, Callable, Mapping, Sequence
import urllib.error
import urllib.parse
import urllib.request
import uuid

from fr3_action_protocol import (
    DEFAULT_SECRET_ENV,
    SIGNATURE_HEADER,
    action_sha256,
    canonical_json_bytes,
    create_action_envelope,
    load_shared_secret,
    make_signature,
)
from isaac_qwen_coordinator import (
    ALLOWED_IMAGE_TYPES,
    DEFAULT_MODEL_ID,
    DEFAULT_QWEN_URL,
    DEFAULT_UBUNTU_PLAN_URL,
    NoAction,
    call_qwen,
    load_image,
    post_json,
    validate_action,
)


DEFAULT_UBUNTU_HEALTH_URL = "http://10.10.10.1:8080/health"
DEFAULT_TASK = (
    "Inspect the image and report whether the red marker center is left or "
    "right of the vertical dashed center line. Report unclear if it cannot "
    "be determined."
)
HARD_MAX_STEPS = 50


@dataclass(frozen=True)
class InferenceResult:
    action: dict[str, Any]
    image_sha256: str
    mime_type: str


class JsonlEventLog:
    def __init__(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        try:
            self._stream = path.open("x", encoding="utf-8", newline="\n")
        except FileExistsError as error:
            raise RuntimeError(f"Refusing to overwrite existing log: {path}") from error
        except OSError as error:
            raise RuntimeError(f"Could not create session log {path}: {error}") from error
        self.path = path

    def write(self, event: Mapping[str, Any]) -> None:
        payload = {"timestamp_unix_ms": int(time.time() * 1000), **event}
        self._stream.write(
            json.dumps(payload, allow_nan=False, ensure_ascii=True, sort_keys=True) + "\n"
        )
        self._stream.flush()

    def close(self) -> None:
        self._stream.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run an ordered, bounded static-image VLA shadow session. This "
            "program can request MoveIt plans but has no robot execution mode."
        )
    )
    image_source = parser.add_mutually_exclusive_group(required=True)
    image_source.add_argument(
        "--image",
        type=Path,
        action="append",
        help="Image to process. Repeat this option to define an ordered sequence.",
    )
    image_source.add_argument(
        "--image-dir",
        type=Path,
        help="Directory of images, processed in case-insensitive filename order.",
    )
    parser.add_argument("--task", default=DEFAULT_TASK)
    parser.add_argument("--model-id", default=DEFAULT_MODEL_ID)
    parser.add_argument("--qwen-url", default=DEFAULT_QWEN_URL)
    parser.add_argument("--ubuntu-health-url", default=DEFAULT_UBUNTU_HEALTH_URL)
    parser.add_argument("--ubuntu-plan-url", default=DEFAULT_UBUNTU_PLAN_URL)
    parser.add_argument("--qwen-timeout", type=float, default=180.0)
    parser.add_argument("--ubuntu-timeout", type=float, default=90.0)
    parser.add_argument("--shared-secret-env", default=DEFAULT_SECRET_ENV)

    parser.add_argument("--min-confidence", type=float, default=0.70)
    parser.add_argument("--max-delta", type=float, default=0.03)
    parser.add_argument("--max-total-delta", type=float, default=0.03)
    parser.add_argument("--max-moving-joints", type=int, default=1)
    parser.add_argument("--max-cumulative-delta", type=float, default=0.15)
    parser.add_argument("--max-steps", type=int, default=10)
    parser.add_argument("--max-session-seconds", type=float, default=120.0)
    parser.add_argument("--interval", type=float, default=0.5)
    parser.add_argument(
        "--no-action-policy",
        choices=("stop", "continue"),
        default="stop",
    )
    parser.add_argument("--log-file", type=Path)
    parser.add_argument(
        "--send-plans",
        action="store_true",
        help=(
            "Send one synchronous plan request per accepted action. Ubuntu "
            "must report PLAN_ONLY. Without this flag, only Qwen is called."
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
        "max_cumulative_delta",
        "max_session_seconds",
    ):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0.0:
            raise RuntimeError(f"--{name.replace('_', '-')} must be finite and positive.")
    if not math.isfinite(args.interval) or args.interval < 0.0:
        raise RuntimeError("--interval must be finite and non-negative.")
    if not 0.0 < args.min_confidence <= 1.0:
        raise RuntimeError("--min-confidence must be in (0, 1].")
    if not 1 <= args.max_steps <= HARD_MAX_STEPS:
        raise RuntimeError(f"--max-steps must be between 1 and {HARD_MAX_STEPS}.")
    if args.max_moving_joints <= 0:
        raise RuntimeError("--max-moving-joints must be positive.")
    validate_ubuntu_endpoints(args.ubuntu_health_url, args.ubuntu_plan_url)


def validate_ubuntu_endpoints(health_url: str, plan_url: str) -> None:
    health = urllib.parse.urlsplit(health_url)
    plan = urllib.parse.urlsplit(plan_url)
    for label, parsed, expected_path in (
        ("health", health, "/health"),
        ("plan", plan, "/v1/actions/plan"),
    ):
        if parsed.scheme not in {"http", "https"} or not parsed.hostname:
            raise RuntimeError(f"Ubuntu {label} URL must be an absolute HTTP(S) URL.")
        if parsed.username or parsed.password or parsed.query or parsed.fragment:
            raise RuntimeError(f"Ubuntu {label} URL cannot contain credentials or parameters.")
        if parsed.path != expected_path:
            raise RuntimeError(f"Ubuntu {label} URL path must be exactly {expected_path}.")
    if (health.scheme, health.hostname, health.port) != (
        plan.scheme,
        plan.hostname,
        plan.port,
    ):
        raise RuntimeError("Ubuntu health and plan URLs must use the same server.")


def collect_images(args: argparse.Namespace) -> list[Path]:
    if args.image:
        images = list(args.image)
    else:
        if not args.image_dir.is_dir():
            raise RuntimeError(f"Image directory does not exist: {args.image_dir}")
        allowed_suffixes = {f".{mime.split('/', 1)[1]}" for mime in ALLOWED_IMAGE_TYPES}
        allowed_suffixes.add(".jpg")
        images = sorted(
            (
                path
                for path in args.image_dir.iterdir()
                if path.is_file() and path.suffix.lower() in allowed_suffixes
            ),
            key=lambda path: path.name.casefold(),
        )

    if not images:
        raise RuntimeError("No JPEG, PNG, or WebP images were selected.")
    missing = [str(path) for path in images if not path.is_file()]
    if missing:
        raise RuntimeError(f"Images do not exist or are not files: {missing}")
    return images


def get_json(url: str, *, timeout: float) -> tuple[int, Any]:
    request = urllib.request.Request(url, method="GET")
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            body = response.read()
            status = response.status
    except urllib.error.HTTPError as error:
        body = error.read()
        status = error.code
    except urllib.error.URLError as error:
        raise RuntimeError(f"Could not reach {url}: {error.reason}") from error
    try:
        return status, json.loads(body.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise RuntimeError(f"Server at {url} returned invalid JSON.") from error


def validate_shadow_health(status: int, health: Any) -> None:
    if status != 200 or not isinstance(health, dict):
        raise RuntimeError(f"Ubuntu health check failed with HTTP {status}: {health}")
    if health.get("mode") != "PLAN_ONLY":
        raise RuntimeError(
            "Shadow sessions require Ubuntu mode PLAN_ONLY. Stop the supervisor "
            "and restart it without --enable-local-execution."
        )
    if health.get("execution_available") is not False:
        raise RuntimeError("Ubuntu did not explicitly report execution_available=false.")
    if health.get("remote_execution_available") is not False:
        raise RuntimeError("Ubuntu did not explicitly disable remote execution.")
    if health.get("local_approval_available") is not False:
        raise RuntimeError("Ubuntu local execution must be disabled during a shadow session.")


def action_motion(action: Mapping[str, Any]) -> float:
    deltas = action["deltas"]
    return sum(abs(float(value)) for value in deltas.values())


def validate_plan_receipt(
    http_status: int,
    response: Any,
    *,
    request_id: str,
    expected_action_digest: str,
) -> dict[str, Any]:
    if http_status != 200 or not isinstance(response, dict):
        raise RuntimeError(f"Ubuntu plan request failed with HTTP {http_status}: {response}")
    if response.get("status") != "PLAN_READY":
        raise RuntimeError(f"Ubuntu did not return PLAN_READY: {response}")
    if response.get("request_id") != request_id:
        raise RuntimeError("Ubuntu plan response request_id does not match the request.")
    if response.get("action_sha256") != expected_action_digest:
        raise RuntimeError("Ubuntu plan response action hash does not match the request.")
    if response.get("plan_only") is not True:
        raise RuntimeError("Ubuntu did not explicitly mark the response plan_only=true.")
    if response.get("execution_available") is not False:
        raise RuntimeError("Ubuntu did not explicitly mark execution_available=false.")
    if response.get("remote_execution_available") is not False:
        raise RuntimeError("Ubuntu did not explicitly disable remote execution.")
    if response.get("local_approval_available") is not False:
        raise RuntimeError("Ubuntu unexpectedly offered local execution in shadow mode.")
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


def make_plan_function(
    args: argparse.Namespace,
    secret: bytes,
) -> Callable[[Mapping[str, Any], str, str, float], dict[str, Any]]:
    def submit(
        action: Mapping[str, Any],
        image_digest: str,
        frame_id: str,
        remaining_seconds: float,
    ) -> dict[str, Any]:
        envelope = create_action_envelope(
            action,
            frame_source="windows-static-shadow-session",
            frame_id=frame_id,
            frame_sha256=image_digest,
        )
        body = canonical_json_bytes(envelope)
        status, response = post_json(
            args.ubuntu_plan_url,
            body,
            timeout=min(args.ubuntu_timeout, remaining_seconds),
            headers={SIGNATURE_HEADER: make_signature(body, secret)},
        )
        return validate_plan_receipt(
            status,
            response,
            request_id=envelope["request_id"],
            expected_action_digest=action_sha256(action),
        )

    return submit


def run_shadow_session(
    args: argparse.Namespace,
    images: Sequence[Path],
    *,
    event_sink: Callable[[Mapping[str, Any]], None],
    infer: Callable[[Path, float], InferenceResult],
    submit_plan: Callable[[Mapping[str, Any], str, str, float], dict[str, Any]] | None,
    session_id: str | None = None,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> tuple[int, dict[str, Any]]:
    selected_session_id = session_id or str(uuid.uuid4())
    selected_images = list(images[: args.max_steps])
    input_truncated = len(images) > len(selected_images)
    started = monotonic()
    cumulative_delta = 0.0
    accepted_steps = 0
    no_action_steps = 0
    plan_ready_steps = 0

    event_sink(
        {
            "event": "SESSION_STARTED",
            "session_id": selected_session_id,
            "image_count": len(images),
            "selected_image_count": len(selected_images),
            "send_plans": submit_plan is not None,
            "limits": {
                "max_steps": args.max_steps,
                "max_session_seconds": args.max_session_seconds,
                "max_cumulative_delta": args.max_cumulative_delta,
                "max_delta": args.max_delta,
                "max_total_delta": args.max_total_delta,
                "max_moving_joints": args.max_moving_joints,
            },
        }
    )

    final_status = "COMPLETED"
    failure = ""
    for index, image in enumerate(selected_images, start=1):
        elapsed = monotonic() - started
        remaining = args.max_session_seconds - elapsed
        if remaining <= 0.0:
            final_status = "SESSION_TIMEOUT"
            break

        frame_id = f"shadow-{selected_session_id}-step-{index:04d}"
        event_sink(
            {
                "event": "STEP_STARTED",
                "session_id": selected_session_id,
                "step": index,
                "frame_id": frame_id,
                "image": str(image),
            }
        )

        try:
            inferred = infer(image, remaining)
            validate_action(inferred.action, args)
        except NoAction as error:
            no_action_steps += 1
            event_sink(
                {
                    "event": "NO_ACTION",
                    "session_id": selected_session_id,
                    "step": index,
                    "reason": str(error),
                }
            )
            if args.no_action_policy == "stop":
                final_status = "TASK_COMPLETE_NO_ACTION"
                break
            continue
        except Exception as error:
            final_status = "FAILED"
            failure = f"{type(error).__name__}: {error}"
            event_sink(
                {
                    "event": "STEP_FAILED",
                    "session_id": selected_session_id,
                    "step": index,
                    "failure": failure,
                }
            )
            break

        elapsed = monotonic() - started
        remaining = args.max_session_seconds - elapsed
        if remaining <= 0.0:
            final_status = "SESSION_TIMEOUT"
            break

        step_delta = action_motion(inferred.action)
        proposed_cumulative = cumulative_delta + step_delta
        if proposed_cumulative > args.max_cumulative_delta + 1e-12:
            final_status = "CUMULATIVE_LIMIT_REACHED"
            failure = (
                f"Proposed cumulative delta {proposed_cumulative:.6f} exceeds "
                f"limit {args.max_cumulative_delta:.6f}."
            )
            event_sink(
                {
                    "event": "CUMULATIVE_LIMIT_REACHED",
                    "session_id": selected_session_id,
                    "step": index,
                    "proposed_cumulative_delta": proposed_cumulative,
                    "limit": args.max_cumulative_delta,
                }
            )
            break

        accepted_steps += 1
        cumulative_delta = proposed_cumulative
        event_sink(
            {
                "event": "ACTION_VALIDATED",
                "session_id": selected_session_id,
                "step": index,
                "image_sha256": inferred.image_sha256,
                "mime_type": inferred.mime_type,
                "action": inferred.action,
                "step_absolute_delta": step_delta,
                "cumulative_absolute_delta": cumulative_delta,
            }
        )

        if submit_plan is not None:
            try:
                receipt = submit_plan(
                    inferred.action,
                    inferred.image_sha256,
                    frame_id,
                    remaining,
                )
            except Exception as error:
                final_status = "FAILED"
                failure = f"{type(error).__name__}: {error}"
                event_sink(
                    {
                        "event": "PLAN_FAILED",
                        "session_id": selected_session_id,
                        "step": index,
                        "failure": failure,
                    }
                )
                break
            plan_ready_steps += 1
            event_sink(
                {
                    "event": "PLAN_READY",
                    "session_id": selected_session_id,
                    "step": index,
                    "request_id": receipt["request_id"],
                    "action_sha256": receipt["action_sha256"],
                    "planning": receipt.get("planning"),
                }
            )

        if index < len(selected_images) and args.interval > 0.0:
            remaining = args.max_session_seconds - (monotonic() - started)
            if remaining <= 0.0:
                final_status = "SESSION_TIMEOUT"
                break
            sleep(min(args.interval, remaining))

    if final_status == "COMPLETED" and input_truncated:
        final_status = "STEP_LIMIT_REACHED"

    summary = {
        "event": "SESSION_FINISHED",
        "session_id": selected_session_id,
        "status": final_status,
        "accepted_steps": accepted_steps,
        "plan_ready_steps": plan_ready_steps,
        "no_action_steps": no_action_steps,
        "cumulative_absolute_delta": cumulative_delta,
        "elapsed_seconds": monotonic() - started,
        "failure": failure or None,
    }
    event_sink(summary)
    exit_code = 1 if final_status in {"FAILED", "CUMULATIVE_LIMIT_REACHED"} else 0
    return exit_code, summary


def default_log_path() -> Path:
    timestamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())
    return Path("session_logs") / f"static_shadow_{timestamp}.jsonl"


def print_summary(summary: Mapping[str, Any], log_path: Path) -> None:
    print("\nStatic VLA shadow session finished")
    print(f"Session ID:           {summary['session_id']}")
    print(f"Status:               {summary['status']}")
    print(f"Accepted actions:     {summary['accepted_steps']}")
    print(f"MoveIt plans ready:   {summary['plan_ready_steps']}")
    print(f"No-action decisions:  {summary['no_action_steps']}")
    print(f"Cumulative delta:     {summary['cumulative_absolute_delta']:.6f} rad")
    print(f"Elapsed:              {summary['elapsed_seconds']:.3f} s")
    print(f"Log:                  {log_path}")
    if summary.get("failure"):
        print(f"Failure:              {summary['failure']}")
    print("Robot execution:      DISABLED")


def main() -> int:
    args = parse_args()
    event_log = None
    try:
        validate_args(args)
        images = collect_images(args)
        log_path = args.log_file or default_log_path()
        event_log = JsonlEventLog(log_path)

        submit_plan = None
        if args.send_plans:
            health_status, health = get_json(
                args.ubuntu_health_url,
                timeout=min(args.ubuntu_timeout, 10.0),
            )
            validate_shadow_health(health_status, health)
            secret = load_shared_secret(args.shared_secret_env)
            submit_plan = make_plan_function(args, secret)
            print("Ubuntu safety gate: PLAN_ONLY confirmed")
        else:
            print("LOCAL INFERENCE ONLY: Ubuntu will not be contacted")

        print(f"Selected images: {min(len(images), args.max_steps)}")
        print(f"Maximum steps: {args.max_steps}")
        print(f"Maximum session time: {args.max_session_seconds:.1f} s")
        print(f"Maximum cumulative delta: {args.max_cumulative_delta:.3f} rad")
        print("Robot execution: DISABLED")

        exit_code, summary = run_shadow_session(
            args,
            images,
            event_sink=event_log.write,
            infer=make_inference_function(args),
            submit_plan=submit_plan,
        )
        print_summary(summary, log_path)
        return exit_code
    except Exception as error:
        print(f"Shadow session error: {type(error).__name__}: {error}", file=sys.stderr)
        return 2
    finally:
        if event_log is not None:
            event_log.close()


if __name__ == "__main__":
    sys.exit(main())
