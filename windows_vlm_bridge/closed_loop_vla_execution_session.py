"""Run a locally armed synthetic visual loop through measured FR3 motion."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import sys
import time
from typing import Any, Callable, Mapping

from closed_loop_vla_shadow_session import (
    FR3_JOINTS,
    MockVisualEnvironment,
    moving_delta,
    render_frame,
)
from fr3_action_protocol import DEFAULT_SECRET_ENV, load_shared_secret
from isaac_qwen_coordinator import DEFAULT_MODEL_ID, DEFAULT_QWEN_URL, NoAction
from static_vla_execution_session import (
    DEFAULT_HEALTH_URL,
    DEFAULT_STEP_URL,
    DEFAULT_STOP_URL,
    TERMINAL_SESSION_STATUSES,
    InferenceResult,
    make_inference_function,
    make_step_function,
    send_stop,
    validate_action_for_session,
    validate_armed_health,
    validate_endpoints,
)
from static_vla_shadow_session import JsonlEventLog, get_json


HARD_MAX_ACTIONS = 4
HARD_MAX_DELTA = 0.03
HARD_MAX_CUMULATIVE_DELTA = 0.12


@dataclass(frozen=True)
class VerifiedMotion:
    joint: str
    requested_delta: float
    measured_delta: float
    target_error: float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run a synthetic Qwen loop through a fresh, locally armed Ubuntu "
            "bounded session. Each image update requires measured VERIFIED_SUCCESS."
        )
    )
    parser.add_argument("--start-side", choices=("left", "right"), default="right")
    parser.add_argument("--start-offset-px", type=float, default=60.0)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--marker-radius", type=int, default=12)
    parser.add_argument("--deadband-px", type=float, default=15.0)
    parser.add_argument("--pixels-per-rad", type=float, default=1000.0)
    parser.add_argument("--joint", choices=FR3_JOINTS, default="fr3_joint3")

    parser.add_argument("--session-id", default="")
    parser.add_argument("--model-id", default=DEFAULT_MODEL_ID)
    parser.add_argument("--qwen-url", default=DEFAULT_QWEN_URL)
    parser.add_argument("--ubuntu-health-url", default=DEFAULT_HEALTH_URL)
    parser.add_argument("--ubuntu-step-url", default=DEFAULT_STEP_URL)
    parser.add_argument("--ubuntu-stop-url", default=DEFAULT_STOP_URL)
    parser.add_argument("--qwen-timeout", type=float, default=180.0)
    parser.add_argument("--ubuntu-timeout", type=float, default=90.0)
    parser.add_argument("--shared-secret-env", default=DEFAULT_SECRET_ENV)
    parser.add_argument("--task", default="")

    parser.add_argument("--min-confidence", type=float, default=0.70)
    parser.add_argument("--max-delta", type=float, default=0.03)
    parser.add_argument("--max-total-delta", type=float, default=0.03)
    parser.add_argument("--max-moving-joints", type=int, default=1)
    parser.add_argument("--max-actions", type=int, default=2)
    parser.add_argument("--max-cumulative-delta", type=float, default=0.06)
    parser.add_argument("--max-measured-delta-error", type=float, default=0.005)
    parser.add_argument("--max-session-seconds", type=float, default=60.0)
    parser.add_argument("--interval", type=float, default=0.5)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument(
        "--run-armed-session",
        action="store_true",
        help=(
            "Enable requests to one fresh session that was locally armed on Ubuntu. "
            "This flag allows real robot motion."
        ),
    )
    return parser.parse_args()


def default_task(args: argparse.Namespace) -> str:
    return (
        "Inspect the synthetic image and compare the red marker center with the "
        "vertical dashed center line. If the marker is right of the line, report "
        f"right and propose exactly +{args.max_delta:.4f} radians for {args.joint}. "
        "If it is left, report left and propose exactly "
        f"-{args.max_delta:.4f} radians for {args.joint}. Treat the marker as "
        f"centered when its center is within about {args.deadband_px:.0f} pixels "
        "of the line. If centered or unclear, report unclear and return zero motion."
    )


def validate_args(args: argparse.Namespace) -> None:
    if not args.run_armed_session:
        raise RuntimeError(
            "Explicit --run-armed-session is required. Use the shadow script "
            "for tests that must not move the robot."
        )
    if args.width < 200 or args.height < 200:
        raise RuntimeError("--width and --height must each be at least 200 pixels.")
    if args.marker_radius <= 0 or args.marker_radius * 2 >= min(args.width, args.height):
        raise RuntimeError("--marker-radius does not fit inside the image.")
    for name in (
        "start_offset_px",
        "deadband_px",
        "pixels_per_rad",
        "qwen_timeout",
        "ubuntu_timeout",
        "max_delta",
        "max_total_delta",
        "max_cumulative_delta",
        "max_measured_delta_error",
        "max_session_seconds",
    ):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0.0:
            raise RuntimeError(f"--{name.replace('_', '-')} must be finite and positive.")
    if not math.isfinite(args.interval) or args.interval < 0.0:
        raise RuntimeError("--interval must be finite and non-negative.")
    if not 0.0 < args.min_confidence <= 1.0:
        raise RuntimeError("--min-confidence must be in (0, 1].")
    if args.max_moving_joints != 1:
        raise RuntimeError("Hardware-loop sessions require --max-moving-joints 1.")
    if not 1 <= args.max_actions <= HARD_MAX_ACTIONS:
        raise RuntimeError(f"--max-actions cannot exceed {HARD_MAX_ACTIONS}.")
    if args.max_delta > HARD_MAX_DELTA + 1e-12:
        raise RuntimeError(f"--max-delta cannot exceed {HARD_MAX_DELTA:.3f} rad.")
    if abs(args.max_total_delta - args.max_delta) > 1e-12:
        raise RuntimeError(
            "For this one-joint loop, --max-total-delta must equal --max-delta."
        )
    if args.max_cumulative_delta > HARD_MAX_CUMULATIVE_DELTA + 1e-12:
        raise RuntimeError("--max-cumulative-delta exceeds the hardware-loop cap.")
    expected_cumulative = args.max_delta * args.max_actions
    if abs(args.max_cumulative_delta - expected_cumulative) > 1e-12:
        raise RuntimeError(
            "--max-cumulative-delta must equal max-delta times max-actions."
        )
    if args.max_measured_delta_error > 0.01:
        raise RuntimeError("--max-measured-delta-error cannot exceed 0.01 rad.")
    if args.max_measured_delta_error >= args.max_delta:
        raise RuntimeError("--max-measured-delta-error must be smaller than --max-delta.")
    if args.start_offset_px <= args.deadband_px:
        raise RuntimeError("--start-offset-px must begin outside the center deadband.")
    maximum_offset = args.width / 2.0 - args.marker_radius
    if args.start_offset_px > maximum_offset:
        raise RuntimeError("--start-offset-px places the marker outside the image.")
    reachable = (
        args.max_actions * args.max_delta * args.pixels_per_rad + args.deadband_px
    )
    if args.start_offset_px > reachable + 1e-9:
        raise RuntimeError("Configured action budget cannot reach the center deadband.")
    validate_endpoints(
        args.ubuntu_health_url,
        args.ubuntu_step_url,
        args.ubuntu_stop_url,
    )
    if args.session_id:
        from static_vla_execution_session import validate_session_id

        validate_session_id(args.session_id)
    if not args.task:
        args.task = default_task(args)


def make_environment(args: argparse.Namespace) -> MockVisualEnvironment:
    direction = -1.0 if args.start_side == "left" else 1.0
    return MockVisualEnvironment(
        width=args.width,
        height=args.height,
        marker_x=args.width / 2.0 + direction * args.start_offset_px,
        marker_radius=args.marker_radius,
        deadband_px=args.deadband_px,
        pixels_per_rad=args.pixels_per_rad,
    )


def validate_exact_session_limits(
    session: Mapping[str, Any],
    args: argparse.Namespace,
) -> None:
    limits = session["limits"]
    expected = {
        "allowed_joints": [args.joint],
        "max_moving_joints": 1,
        "max_steps": args.max_actions,
        "max_delta": args.max_delta,
        "max_total_delta": args.max_total_delta,
        "max_cumulative_delta": args.max_cumulative_delta,
    }
    if limits.get("allowed_joints") != expected["allowed_joints"]:
        raise RuntimeError("Armed joint does not match --joint.")
    if limits.get("max_moving_joints") != 1:
        raise RuntimeError("Armed max_moving_joints must equal 1 for this visual loop.")
    if limits.get("max_steps") != expected["max_steps"]:
        raise RuntimeError("Armed max_steps must exactly match --max-actions.")
    for field in ("max_delta", "max_total_delta", "max_cumulative_delta"):
        try:
            value = float(limits.get(field))
        except (TypeError, ValueError) as error:
            raise RuntimeError(f"Armed {field} is invalid.") from error
        if not math.isfinite(value) or abs(value - float(expected[field])) > 1e-12:
            raise RuntimeError(f"Armed {field} must exactly match the Windows limit.")


def _finite(value: Any, field: str) -> float:
    if isinstance(value, bool):
        raise RuntimeError(f"{field} must be numeric.")
    try:
        parsed = float(value)
    except (TypeError, ValueError) as error:
        raise RuntimeError(f"{field} must be numeric.") from error
    if not math.isfinite(parsed):
        raise RuntimeError(f"{field} must be finite.")
    return parsed


def extract_verified_motion(
    receipt: Mapping[str, Any],
    *,
    joint: str,
    requested_delta: float,
    maximum_error: float,
) -> VerifiedMotion:
    execution = receipt.get("execution")
    if not isinstance(execution, dict) or execution.get("status") != "VERIFIED_SUCCESS":
        raise RuntimeError("Receipt does not contain VERIFIED_SUCCESS execution.")
    verification = execution.get("verification")
    if not isinstance(verification, dict) or verification.get("status") != "PASSED":
        raise RuntimeError("Receipt does not contain a passed measured verification.")
    commanded = verification.get("commanded_joints")
    if not isinstance(commanded, list) or len(commanded) != 1:
        raise RuntimeError("Receipt must verify exactly one commanded joint.")
    result = commanded[0]
    if not isinstance(result, dict) or result.get("joint") != joint:
        raise RuntimeError("Receipt verified the wrong joint.")
    for field in ("direction_ok", "movement_ok", "target_ok"):
        if result.get(field) is not True:
            raise RuntimeError(f"Measured verification field {field} did not pass.")
    if verification.get("non_commanded_drift_ok") is not True:
        raise RuntimeError("Measured non-commanded drift did not pass.")
    if verification.get("settled_velocity_ok") is not True:
        raise RuntimeError("Measured settled velocity did not pass.")

    reported_request = _finite(result.get("requested_delta"), "requested_delta")
    measured_delta = _finite(result.get("measured_delta"), "measured_delta")
    target_error = _finite(result.get("target_error"), "target_error")
    if target_error < 0.0 or target_error > maximum_error + 1e-12:
        raise RuntimeError("Measured target error exceeds the Windows limit.")
    if abs(reported_request - requested_delta) > 1e-9:
        raise RuntimeError("Verified request does not match the Qwen action.")
    if requested_delta * measured_delta <= 0.0:
        raise RuntimeError("Measured joint motion has the wrong direction.")
    if abs(measured_delta - requested_delta) > maximum_error + 1e-12:
        raise RuntimeError("Measured joint delta differs too far from the request.")

    before = receipt.get("current_joint_positions")
    target = receipt.get("target_joint_positions")
    after = verification.get("measured_joint_positions")
    velocities = verification.get("settled_joint_velocities")
    for label, values in (
        ("current_joint_positions", before),
        ("target_joint_positions", target),
        ("measured_joint_positions", after),
        ("settled_joint_velocities", velocities),
    ):
        if not isinstance(values, dict):
            raise RuntimeError(f"Receipt {label} is missing.")
        for expected_joint in FR3_JOINTS:
            _finite(values.get(expected_joint), f"{label}.{expected_joint}")

    measured_from_positions = float(after[joint]) - float(before[joint])
    requested_from_positions = float(target[joint]) - float(before[joint])
    error_from_positions = abs(float(after[joint]) - float(target[joint]))
    if abs(measured_from_positions - measured_delta) > 1e-6:
        raise RuntimeError("Measured delta conflicts with the returned joint positions.")
    if abs(requested_from_positions - requested_delta) > 1e-9:
        raise RuntimeError("Target position conflicts with the requested delta.")
    if abs(error_from_positions - target_error) > 1e-6:
        raise RuntimeError("Target error conflicts with the returned joint positions.")
    return VerifiedMotion(joint, requested_delta, measured_delta, target_error)


def run_hardware_loop(
    args: argparse.Namespace,
    environment: MockVisualEnvironment,
    output_dir: Path,
    session: Mapping[str, Any],
    *,
    event_sink: Callable[[Mapping[str, Any]], None],
    infer: Callable[[Path, float], InferenceResult],
    submit_step: Callable[
        [Mapping[str, Any], str, str, int, float], dict[str, Any]
    ],
    stop_session: Callable[[float], dict[str, Any]],
    renderer: Callable[[MockVisualEnvironment, int, Path], Path] = render_frame,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> tuple[int, dict[str, Any]]:
    session_id = str(session["session_id"])
    start = monotonic()
    deadline = min(
        args.max_session_seconds,
        float(session.get("remaining_seconds", args.max_session_seconds)),
    )
    observations = 0
    verified_steps = 0
    sequence = 1
    requested_cumulative = 0.0
    measured_cumulative = 0.0
    last_server_status = "ARMED"
    final_status = "FAILED"
    failure = ""

    event_sink(
        {
            "event": "HARDWARE_LOOP_STARTED",
            "session_id": session_id,
            "joint": args.joint,
            "start_marker_x": environment.marker_x,
            "start_side": environment.side,
            "limits": session["limits"],
            "robot_execution_enabled": True,
        }
    )

    while True:
        remaining = deadline - (monotonic() - start)
        if remaining <= 0.0:
            final_status = "SESSION_TIMEOUT"
            failure = "Hardware-loop deadline elapsed."
            break
        if verified_steps >= args.max_actions and environment.side != "centered":
            final_status = "ACTION_LIMIT_REACHED"
            failure = "Measured robot motion did not reach the synthetic deadband."
            break

        observations += 1
        frame = renderer(environment, observations, output_dir)
        true_side = environment.side
        event_sink(
            {
                "event": "OBSERVATION_RENDERED",
                "session_id": session_id,
                "observation": observations,
                "frame": str(frame),
                "ground_truth_side": true_side,
                "marker_x": environment.marker_x,
                "error_px": environment.error_px,
            }
        )

        try:
            inferred = infer(frame, remaining)
            moving_deltas, _step_total = validate_action_for_session(
                inferred.action,
                args,
                session,
                requested_cumulative,
            )
            requested_delta = moving_deltas[args.joint]
        except NoAction as error:
            event_sink(
                {
                    "event": "NO_ACTION",
                    "session_id": session_id,
                    "observation": observations,
                    "ground_truth_side": true_side,
                    "reason": str(error),
                }
            )
            if true_side == "centered":
                final_status = "GOAL_REACHED"
            else:
                final_status = "PREMATURE_NO_ACTION"
                failure = "Qwen stopped while the marker was outside the deadband."
            break
        except Exception as error:
            final_status = "INFERENCE_OR_VALIDATION_FAILED"
            failure = f"{type(error).__name__}: {error}"
            break

        observed_side = inferred.action["observation"]["red_marker_side"]
        if true_side == "centered" or observed_side != true_side:
            final_status = "PERCEPTION_MISMATCH"
            failure = (
                f"Ground truth is {true_side!r}, but Qwen reported {observed_side!r}."
            )
            break
        if abs(moving_delta(inferred.action, args.joint) - requested_delta) > 1e-12:
            final_status = "ACTION_REJECTED"
            failure = "Validated action delta changed unexpectedly."
            break
        if last_server_status != "ARMED":
            final_status = "SESSION_NOT_ARMED"
            failure = f"Ubuntu session status is {last_server_status}."
            break

        frame_id = f"synthetic-hardware-{session_id}-seq-{sequence:04d}"
        event_sink(
            {
                "event": "ACTION_VALIDATED",
                "session_id": session_id,
                "sequence": sequence,
                "observation": observations,
                "action": inferred.action,
                "image_sha256": inferred.image_sha256,
            }
        )
        try:
            receipt = submit_step(
                inferred.action,
                inferred.image_sha256,
                frame_id,
                sequence,
                remaining,
            )
            motion = extract_verified_motion(
                receipt,
                joint=args.joint,
                requested_delta=requested_delta,
                maximum_error=args.max_measured_delta_error,
            )
        except Exception as error:
            final_status = "STEP_FAILED"
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

        verified_steps += 1
        sequence += 1
        requested_cumulative += abs(motion.requested_delta)
        measured_cumulative += abs(motion.measured_delta)
        last_server_status = str(receipt["session"]["status"])
        event_sink(
            {
                "event": "STEP_VERIFIED",
                "session_id": session_id,
                "sequence": sequence - 1,
                "request_id": receipt["request_id"],
                "action_sha256": receipt["action_sha256"],
                "requested_delta": motion.requested_delta,
                "measured_delta": motion.measured_delta,
                "target_error": motion.target_error,
                "session_status": last_server_status,
                "planning": receipt.get("planning"),
            }
        )

        try:
            before_error, after_error = environment.apply_joint_delta(
                motion.measured_delta
            )
        except Exception as error:
            final_status = "MEASURED_MOTION_DID_NOT_IMPROVE"
            failure = f"{type(error).__name__}: {error}"
            break
        event_sink(
            {
                "event": "MEASURED_MOCK_UPDATE",
                "session_id": session_id,
                "sequence": sequence - 1,
                "measured_delta": motion.measured_delta,
                "before_error_px": before_error,
                "after_error_px": after_error,
                "new_marker_x": environment.marker_x,
                "new_ground_truth_side": environment.side,
            }
        )

        if last_server_status in TERMINAL_SESSION_STATUSES - {"COMPLETED"}:
            final_status = f"SESSION_{last_server_status}"
            failure = "Ubuntu terminated the session after the verified step."
            break
        if args.interval > 0.0:
            remaining = deadline - (monotonic() - start)
            if remaining <= 0.0:
                final_status = "SESSION_TIMEOUT"
                failure = "Hardware-loop deadline elapsed."
                break
            sleep(min(args.interval, remaining))

    stop_confirmed = last_server_status in TERMINAL_SESSION_STATUSES
    if not stop_confirmed:
        try:
            remaining = max(1.0, deadline - (monotonic() - start))
            stop_receipt = stop_session(remaining)
            stop_confirmed = True
            last_server_status = str(stop_receipt["session"]["status"])
            event_sink(
                {
                    "event": "STOP_CONFIRMED",
                    "session_id": session_id,
                    "session_status": last_server_status,
                }
            )
        except Exception as error:
            stop_failure = f"{type(error).__name__}: {error}"
            failure = f"{failure}; stop unconfirmed: {stop_failure}".strip("; ")
            final_status = "FAILED_STOP_UNCONFIRMED"
            event_sink(
                {
                    "event": "STOP_UNCONFIRMED",
                    "session_id": session_id,
                    "failure": stop_failure,
                }
            )

    summary = {
        "event": "HARDWARE_LOOP_FINISHED",
        "session_id": session_id,
        "status": final_status,
        "observations": observations,
        "verified_steps": verified_steps,
        "requested_cumulative_delta": requested_cumulative,
        "measured_cumulative_delta": measured_cumulative,
        "final_marker_x": environment.marker_x,
        "final_error_px": environment.error_px,
        "final_ground_truth_side": environment.side,
        "last_server_status": last_server_status,
        "stop_confirmed": stop_confirmed,
        "elapsed_seconds": monotonic() - start,
        "failure": failure or None,
        "robot_execution_enabled": True,
    }
    event_sink(summary)
    return (0 if final_status == "GOAL_REACHED" and stop_confirmed else 1), summary


def default_output_dir() -> Path:
    timestamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())
    return Path("mock_runs") / f"hardware_loop_{timestamp}"


def print_event(event: Mapping[str, Any]) -> None:
    if event.get("event") == "OBSERVATION_RENDERED":
        print(
            f"Observation {event['observation']}: "
            f"ground_truth={event['ground_truth_side']}, "
            f"error={event['error_px']:+.1f}px"
        )
    elif event.get("event") == "STEP_VERIFIED":
        print(
            f"  VERIFIED step {event['sequence']}: requested "
            f"{event['requested_delta']:+.6f}, measured "
            f"{event['measured_delta']:+.6f} rad"
        )
    elif event.get("event") == "MEASURED_MOCK_UPDATE":
        print(
            f"  Measured feedback updated error "
            f"{event['before_error_px']:.1f}px -> {event['after_error_px']:.1f}px"
        )
    elif event.get("event") == "NO_ACTION":
        print(f"  Qwen returned NO_ACTION ({event['reason']})")


def print_summary(summary: Mapping[str, Any], output_dir: Path) -> None:
    print("\nSynthetic hardware-in-the-loop session finished")
    print(f"Session ID:          {summary['session_id']}")
    print(f"Status:              {summary['status']}")
    print(f"Observations:        {summary['observations']}")
    print(f"Verified steps:      {summary['verified_steps']}")
    print(f"Requested cumulative:{summary['requested_cumulative_delta']:>10.6f} rad")
    print(f"Measured cumulative: {summary['measured_cumulative_delta']:.6f} rad")
    print(f"Final visual error:  {summary['final_error_px']:+.3f} px")
    print(f"Ubuntu status:       {summary['last_server_status']}")
    print(f"Stop confirmed:      {summary['stop_confirmed']}")
    print("Robot execution:     ENABLED FOR THIS LOCALLY ARMED SESSION")
    print(f"Frames and log:      {output_dir}")
    if summary.get("failure"):
        print(f"Failure:             {summary['failure']}")


def main() -> int:
    args = parse_args()
    event_log = None
    try:
        validate_args(args)
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
        validate_exact_session_limits(session, args)

        output_dir = args.output_dir or default_output_dir()
        output_dir.mkdir(parents=True, exist_ok=False)
        event_log = JsonlEventLog(output_dir / "events.jsonl")
        environment = make_environment(args)
        session_id = str(session["session_id"])
        submit_step = make_step_function(
            args,
            secret,
            session_id,
            frame_source="windows-synthetic-hardware-loop",
        )
        stop_session = lambda timeout: send_stop(
            args,
            secret,
            session_id,
            timeout=timeout,
        )

        print("REAL ROBOT EXECUTION IS ENABLED FOR THIS LOCALLY ARMED SESSION.")
        print(f"Session ID:          {session_id}")
        print(f"Allowed joint:       {args.joint}")
        print(f"Maximum steps:       {args.max_actions}")
        print(f"Cumulative limit:    {args.max_cumulative_delta:.6f} rad")
        print(f"Start visual error:  {environment.error_px:+.1f} px")
        print(f"Output:              {output_dir}")

        def record(event: Mapping[str, Any]) -> None:
            event_log.write(event)
            print_event(event)

        exit_code, summary = run_hardware_loop(
            args,
            environment,
            output_dir,
            session,
            event_sink=record,
            infer=make_inference_function(args),
            submit_step=submit_step,
            stop_session=stop_session,
        )
        print_summary(summary, output_dir)
        return exit_code
    except FileExistsError:
        print(
            "Hardware-loop error: --output-dir already exists; choose a new directory.",
            file=sys.stderr,
        )
        return 2
    except Exception as error:
        print(f"Hardware-loop error: {type(error).__name__}: {error}", file=sys.stderr)
        return 2
    finally:
        if event_log is not None:
            event_log.close()


if __name__ == "__main__":
    sys.exit(main())
