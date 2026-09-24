"""Run a synthetic visual closed loop through Qwen without robot access."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys
import time
from typing import Any, Callable, Mapping
import uuid

from PIL import Image, ImageDraw

from fr3_action_protocol import DEFAULT_SECRET_ENV, load_shared_secret
from isaac_qwen_coordinator import (
    DEFAULT_MODEL_ID,
    DEFAULT_QWEN_URL,
    DEFAULT_UBUNTU_PLAN_URL,
    NoAction,
    validate_action,
)
from static_vla_shadow_session import (
    DEFAULT_UBUNTU_HEALTH_URL,
    InferenceResult,
    JsonlEventLog,
    get_json,
    make_inference_function,
    make_plan_function,
    validate_shadow_health,
    validate_ubuntu_endpoints,
)


FR3_JOINTS = tuple(f"fr3_joint{index}" for index in range(1, 8))
HARD_MAX_ACTIONS = 20


@dataclass
class MockVisualEnvironment:
    width: int
    height: int
    marker_x: float
    marker_radius: int
    deadband_px: float
    pixels_per_rad: float

    @property
    def center_x(self) -> float:
        return self.width / 2.0

    @property
    def error_px(self) -> float:
        return self.marker_x - self.center_x

    @property
    def side(self) -> str:
        if self.error_px < -self.deadband_px:
            return "left"
        if self.error_px > self.deadband_px:
            return "right"
        return "centered"

    def apply_joint_delta(self, delta: float) -> tuple[float, float]:
        before_error = abs(self.error_px)
        proposed_x = self.marker_x - delta * self.pixels_per_rad
        proposed_x = min(
            float(self.width - self.marker_radius),
            max(float(self.marker_radius), proposed_x),
        )
        after_error = abs(proposed_x - self.center_x)
        if after_error >= before_error - 1e-9:
            raise RuntimeError(
                "Mock action did not reduce visual error; check the action sign mapping."
            )
        self.marker_x = proposed_x
        return before_error, after_error


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Render a synthetic red marker, call Qwen, and update the mock "
            "image state from each validated action. Optional Ubuntu requests "
            "are strictly MoveIt plan-only; robot execution is unavailable."
        )
    )
    parser.add_argument("--start-side", choices=("left", "right"), default="right")
    parser.add_argument("--start-offset-px", type=float, default=120.0)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--marker-radius", type=int, default=24)
    parser.add_argument("--deadband-px", type=float, default=30.0)
    parser.add_argument("--pixels-per-rad", type=float, default=1000.0)
    parser.add_argument("--joint", choices=FR3_JOINTS, default="fr3_joint3")

    parser.add_argument("--model-id", default=DEFAULT_MODEL_ID)
    parser.add_argument("--qwen-url", default=DEFAULT_QWEN_URL)
    parser.add_argument("--qwen-timeout", type=float, default=180.0)
    parser.add_argument("--ubuntu-health-url", default=DEFAULT_UBUNTU_HEALTH_URL)
    parser.add_argument("--ubuntu-plan-url", default=DEFAULT_UBUNTU_PLAN_URL)
    parser.add_argument("--ubuntu-timeout", type=float, default=90.0)
    parser.add_argument("--shared-secret-env", default=DEFAULT_SECRET_ENV)
    parser.add_argument("--task", default="")
    parser.add_argument("--min-confidence", type=float, default=0.70)
    parser.add_argument("--max-delta", type=float, default=0.03)
    parser.add_argument("--max-total-delta", type=float, default=0.03)
    parser.add_argument("--max-moving-joints", type=int, default=1)
    parser.add_argument("--max-actions", type=int, default=6)
    parser.add_argument("--max-cumulative-delta", type=float, default=0.18)
    parser.add_argument("--max-session-seconds", type=float, default=180.0)
    parser.add_argument("--interval", type=float, default=0.5)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument(
        "--send-plans",
        action="store_true",
        help=(
            "Require one authenticated Ubuntu PLAN_READY response before each "
            "mock state update. Ubuntu must report strict PLAN_ONLY mode."
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
    if args.width < 200 or args.height < 200:
        raise RuntimeError("--width and --height must each be at least 200 pixels.")
    if args.marker_radius <= 0 or args.marker_radius * 2 >= min(args.width, args.height):
        raise RuntimeError("--marker-radius does not fit inside the image.")
    positive_values = (
        "start_offset_px",
        "deadband_px",
        "pixels_per_rad",
        "qwen_timeout",
        "ubuntu_timeout",
        "max_delta",
        "max_total_delta",
        "max_cumulative_delta",
        "max_session_seconds",
    )
    for name in positive_values:
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0.0:
            raise RuntimeError(f"--{name.replace('_', '-')} must be finite and positive.")
    if not math.isfinite(args.interval) or args.interval < 0.0:
        raise RuntimeError("--interval must be finite and non-negative.")
    if not 0.0 < args.min_confidence <= 1.0:
        raise RuntimeError("--min-confidence must be in (0, 1].")
    if args.max_moving_joints != 1:
        raise RuntimeError("Mock closed-loop tests require --max-moving-joints 1.")
    if args.max_total_delta > args.max_delta + 1e-12:
        raise RuntimeError("For one joint, --max-total-delta cannot exceed --max-delta.")
    if not 1 <= args.max_actions <= HARD_MAX_ACTIONS:
        raise RuntimeError(f"--max-actions must be between 1 and {HARD_MAX_ACTIONS}.")
    if args.start_offset_px <= args.deadband_px:
        raise RuntimeError("--start-offset-px must begin outside the center deadband.")
    maximum_offset = args.width / 2.0 - args.marker_radius
    if args.start_offset_px > maximum_offset:
        raise RuntimeError("--start-offset-px places the marker outside the image.")
    validate_ubuntu_endpoints(args.ubuntu_health_url, args.ubuntu_plan_url)
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


def render_frame(
    environment: MockVisualEnvironment,
    observation: int,
    output_dir: Path,
) -> Path:
    image = Image.new("RGB", (environment.width, environment.height), "white")
    draw = ImageDraw.Draw(image)
    center_x = int(round(environment.center_x))
    for y in range(16, environment.height - 16, 20):
        draw.line(
            (center_x, y, center_x, min(y + 10, environment.height - 16)),
            fill="black",
            width=3,
        )

    marker_x = int(round(environment.marker_x))
    marker_y = environment.height // 2
    radius = environment.marker_radius
    draw.ellipse(
        (marker_x - radius, marker_y - radius, marker_x + radius, marker_y + radius),
        fill=(220, 35, 45),
        outline=(120, 0, 0),
        width=3,
    )
    path = output_dir / f"frame_{observation:03d}_x_{marker_x:04d}.png"
    image.save(path, format="PNG")
    return path


def moving_delta(action: Mapping[str, Any], expected_joint: str) -> float:
    moving = {
        str(joint): float(value)
        for joint, value in action["deltas"].items()
        if abs(float(value)) > 1e-9
    }
    if set(moving) != {expected_joint}:
        raise RuntimeError(
            f"Mock loop permits only {expected_joint}; action uses {sorted(moving)}."
        )
    return moving[expected_joint]


def run_closed_loop(
    args: argparse.Namespace,
    environment: MockVisualEnvironment,
    output_dir: Path,
    *,
    event_sink: Callable[[Mapping[str, Any]], None],
    infer: Callable[[Path, float], InferenceResult],
    submit_plan: Callable[[Mapping[str, Any], str, str, float], dict[str, Any]]
    | None = None,
    session_id: str | None = None,
    renderer: Callable[[MockVisualEnvironment, int, Path], Path] = render_frame,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> tuple[int, dict[str, Any]]:
    selected_session_id = session_id or str(uuid.uuid4())
    started = monotonic()
    observations = 0
    actions_applied = 0
    plans_ready = 0
    cumulative_delta = 0.0
    final_status = "FAILED"
    failure = ""

    event_sink(
        {
            "event": "MOCK_LOOP_STARTED",
            "session_id": selected_session_id,
            "start_side": environment.side,
            "start_marker_x": environment.marker_x,
            "center_x": environment.center_x,
            "deadband_px": environment.deadband_px,
            "pixels_per_rad": environment.pixels_per_rad,
            "joint": args.joint,
            "limits": {
                "max_actions": args.max_actions,
                "max_delta": args.max_delta,
                "max_cumulative_delta": args.max_cumulative_delta,
                "max_session_seconds": args.max_session_seconds,
            },
            "robot_execution_enabled": False,
            "send_plans": submit_plan is not None,
        }
    )

    while True:
        remaining = args.max_session_seconds - (monotonic() - started)
        if remaining <= 0.0:
            final_status = "SESSION_TIMEOUT"
            failure = "Mock closed-loop deadline elapsed."
            break
        if actions_applied >= args.max_actions and environment.side != "centered":
            final_status = "ACTION_LIMIT_REACHED"
            failure = "Marker remained outside the deadband after the action limit."
            break

        observations += 1
        frame = renderer(environment, observations, output_dir)
        true_side = environment.side
        event_sink(
            {
                "event": "OBSERVATION_RENDERED",
                "observation": observations,
                "frame": str(frame),
                "ground_truth_side": true_side,
                "marker_x": environment.marker_x,
                "error_px": environment.error_px,
            }
        )

        try:
            inferred = infer(frame, remaining)
            validate_action(inferred.action, args)
        except NoAction as error:
            event_sink(
                {
                    "event": "NO_ACTION",
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

        try:
            delta = moving_delta(inferred.action, args.joint)
        except Exception as error:
            final_status = "ACTION_REJECTED"
            failure = f"{type(error).__name__}: {error}"
            break

        proposed_cumulative = cumulative_delta + abs(delta)
        if proposed_cumulative > args.max_cumulative_delta + 1e-12:
            final_status = "CUMULATIVE_LIMIT_REACHED"
            failure = (
                f"Proposed cumulative delta {proposed_cumulative:.6f} exceeds "
                f"{args.max_cumulative_delta:.6f}."
            )
            break

        if submit_plan is not None:
            remaining = args.max_session_seconds - (monotonic() - started)
            if remaining <= 0.0:
                final_status = "SESSION_TIMEOUT"
                failure = "Mock closed-loop deadline elapsed before planning."
                break
            frame_id = (
                f"mock-shadow-{selected_session_id}-observation-{observations:04d}"
            )
            try:
                receipt = submit_plan(
                    inferred.action,
                    inferred.image_sha256,
                    frame_id,
                    remaining,
                )
            except Exception as error:
                final_status = "PLAN_FAILED"
                failure = f"{type(error).__name__}: {error}"
                event_sink(
                    {
                        "event": "PLAN_FAILED",
                        "session_id": selected_session_id,
                        "observation": observations,
                        "frame_id": frame_id,
                        "failure": failure,
                    }
                )
                break
            plans_ready += 1
            event_sink(
                {
                    "event": "PLAN_READY",
                    "session_id": selected_session_id,
                    "observation": observations,
                    "frame_id": frame_id,
                    "request_id": receipt["request_id"],
                    "action_sha256": receipt["action_sha256"],
                    "planning": receipt.get("planning"),
                }
            )

        try:
            before_error, after_error = environment.apply_joint_delta(delta)
        except Exception as error:
            final_status = "ACTION_DID_NOT_IMPROVE"
            failure = f"{type(error).__name__}: {error}"
            break

        actions_applied += 1
        cumulative_delta = proposed_cumulative
        event_sink(
            {
                "event": "MOCK_ACTION_APPLIED",
                "observation": observations,
                "action_number": actions_applied,
                "image_sha256": inferred.image_sha256,
                "action": inferred.action,
                "before_error_px": before_error,
                "after_error_px": after_error,
                "new_marker_x": environment.marker_x,
                "new_ground_truth_side": environment.side,
                "cumulative_absolute_delta": cumulative_delta,
            }
        )

        if args.interval > 0.0:
            remaining = args.max_session_seconds - (monotonic() - started)
            if remaining <= 0.0:
                final_status = "SESSION_TIMEOUT"
                failure = "Mock closed-loop deadline elapsed."
                break
            sleep(min(args.interval, remaining))

    summary = {
        "event": "MOCK_LOOP_FINISHED",
        "status": final_status,
        "session_id": selected_session_id,
        "observations": observations,
        "actions_applied": actions_applied,
        "plans_ready": plans_ready,
        "cumulative_absolute_delta": cumulative_delta,
        "final_marker_x": environment.marker_x,
        "final_error_px": environment.error_px,
        "final_ground_truth_side": environment.side,
        "elapsed_seconds": monotonic() - started,
        "failure": failure or None,
        "robot_execution_enabled": False,
    }
    event_sink(summary)
    return (0 if final_status == "GOAL_REACHED" else 1), summary


def default_output_dir() -> Path:
    timestamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())
    return Path("mock_closed_loop_runs") / f"run_{timestamp}"


def print_progress(event: Mapping[str, Any]) -> None:
    if event.get("event") == "OBSERVATION_RENDERED":
        print(
            f"Observation {event['observation']}: "
            f"ground_truth={event['ground_truth_side']}, "
            f"x={event['marker_x']:.1f}, error={event['error_px']:+.1f}px"
        )
    elif event.get("event") == "MOCK_ACTION_APPLIED":
        delta = event["action"]["deltas"]
        print(
            f"  Qwen action={json.dumps(delta, sort_keys=True)}, "
            f"error {event['before_error_px']:.1f}px -> {event['after_error_px']:.1f}px"
        )
    elif event.get("event") == "NO_ACTION":
        print(f"  Qwen returned NO_ACTION ({event['reason']})")
    elif event.get("event") == "PLAN_READY":
        planning = event.get("planning") or {}
        print(
            "  Ubuntu PLAN_READY "
            f"({planning.get('point_count', '?')} trajectory points)"
        )


def print_summary(summary: Mapping[str, Any], output_dir: Path) -> None:
    print("\nSynthetic visual closed-loop shadow session finished")
    print(f"Status:              {summary['status']}")
    print(f"Observations:        {summary['observations']}")
    print(f"Actions applied:     {summary['actions_applied']}")
    print(f"MoveIt plans ready:  {summary['plans_ready']}")
    print(f"Cumulative delta:    {summary['cumulative_absolute_delta']:.6f} rad")
    print(f"Final marker side:   {summary['final_ground_truth_side']}")
    print(f"Final visual error:  {summary['final_error_px']:+.3f} px")
    print(f"Elapsed:             {summary['elapsed_seconds']:.3f} s")
    print("Robot execution:     DISABLED (plan-only requests cannot execute)")
    print(f"Frames and log:      {output_dir}")
    if summary.get("failure"):
        print(f"Failure:             {summary['failure']}")


def main() -> int:
    args = parse_args()
    event_log = None
    try:
        validate_args(args)
        submit_plan = None
        if args.send_plans:
            health_status, health = get_json(
                args.ubuntu_health_url,
                timeout=min(args.ubuntu_timeout, 10.0),
            )
            validate_shadow_health(health_status, health)
            secret = load_shared_secret(args.shared_secret_env)
            submit_plan = make_plan_function(args, secret)
            print("Ubuntu safety gate:  PLAN_ONLY confirmed")
        else:
            print("LOCAL SYNTHETIC LOOP: Ubuntu will not be contacted")

        output_dir = args.output_dir or default_output_dir()
        output_dir.mkdir(parents=True, exist_ok=False)
        event_log = JsonlEventLog(output_dir / "events.jsonl")
        environment = make_environment(args)

        print("SYNTHETIC SHADOW MODE. THE ROBOT CANNOT MOVE.")
        print(f"Start side:          {environment.side}")
        print(f"Start visual error:  {environment.error_px:+.1f} px")
        print(f"Center deadband:     +/-{environment.deadband_px:.1f} px")
        print(f"Mock response:       positive {args.joint} moves marker left")
        print(f"Output:              {output_dir}")

        def record(event: Mapping[str, Any]) -> None:
            event_log.write(event)
            print_progress(event)

        exit_code, summary = run_closed_loop(
            args,
            environment,
            output_dir,
            event_sink=record,
            infer=make_inference_function(args),
            submit_plan=submit_plan,
        )
        print_summary(summary, output_dir)
        return exit_code
    except FileExistsError:
        print(
            "Closed-loop error: --output-dir already exists; choose a new directory.",
            file=sys.stderr,
        )
        return 2
    except Exception as error:
        print(f"Closed-loop error: {type(error).__name__}: {error}", file=sys.stderr)
        return 2
    finally:
        if event_log is not None:
            event_log.close()


if __name__ == "__main__":
    sys.exit(main())
