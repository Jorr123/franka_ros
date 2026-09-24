#!/usr/bin/env python3
"""Validate a VLA-style joint command and send it through MoveIt.

This is a first safety bridge for testing VLA/VLM inference without Isaac Sim.
It accepts either a mock primitive action or a JSON action, converts it to small
FR3 joint deltas, validates the command, asks MoveIt to plan, and only executes
when --execute is provided.

Supported JSON examples:

  {"action": "primitive", "name": "wrist_rotate_positive", "confidence": 0.9}
  {"action": "joint_delta", "joint": "fr3_joint7", "delta": 0.05, "confidence": 0.9}
  {"action": "joint_deltas", "deltas": {"fr3_joint6": 0.02, "fr3_joint7": -0.03},
   "confidence": 0.9}
"""

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Dict, Mapping, Tuple

import rclpy
from moveit_msgs.msg import MoveItErrorCodes

from moveit_joint_goal_test import (
    JOINTS,
    MoveItJointGoalClient,
    add_post_execution_verification_arguments,
    error_name,
    format_joint_table,
    make_plan_request,
    run_post_execution_verification,
    trajectory_summary,
    validate_post_execution_verification_args,
)


JOINT_LIMITS = {
    "fr3_joint1": (-2.7437, 2.7437),
    "fr3_joint2": (-1.7837, 1.7837),
    "fr3_joint3": (-2.9007, 2.9007),
    "fr3_joint4": (-3.0421, -0.1518),
    "fr3_joint5": (-2.8065, 2.8065),
    "fr3_joint6": (0.5445, 4.5169),
    "fr3_joint7": (-3.0159, 3.0159),
}


# These are intentionally tiny and boring. They are labels for early testing,
# not a claim that the robot will visually move "right" from every viewpoint.
PRIMITIVES = {
    "wrist_rotate_positive": {"fr3_joint7": 0.05},
    "wrist_rotate_negative": {"fr3_joint7": -0.05},
    "rotate_wrist_right": {"fr3_joint7": 0.05},
    "rotate_wrist_left": {"fr3_joint7": -0.05},
    "joint6_positive": {"fr3_joint6": 0.04},
    "joint6_negative": {"fr3_joint6": -0.04},
    "base_rotate_positive": {"fr3_joint1": 0.04},
    "base_rotate_negative": {"fr3_joint1": -0.04},
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate a VLA-style action and execute it through MoveIt joint goals."
    )
    input_group = parser.add_mutually_exclusive_group(required=True)
    input_group.add_argument(
        "--mock-action",
        choices=sorted(PRIMITIVES.keys()),
        help="Use a built-in tiny primitive action.",
    )
    input_group.add_argument(
        "--json",
        help="JSON VLA output string.",
    )
    input_group.add_argument(
        "--json-file",
        type=Path,
        help="Path to a JSON file containing the VLA output.",
    )

    parser.add_argument("--min-confidence", type=float, default=0.50)
    parser.add_argument("--max-delta", type=float, default=0.08)
    parser.add_argument("--max-total-delta", type=float, default=0.12)
    parser.add_argument("--max-moving-joints", type=int, default=2)
    parser.add_argument("--joint-limit-margin", type=float, default=0.02)

    parser.add_argument("--state-topic", default="/joint_states")
    parser.add_argument("--plan-service", default="/plan_kinematic_path")
    parser.add_argument("--execute-action", default="/execute_trajectory")
    parser.add_argument("--group", default="fr3_arm")
    parser.add_argument("--planning-frame", default="base")
    parser.add_argument("--pipeline", default="")
    parser.add_argument("--planner", default="")
    parser.add_argument("--planning-attempts", type=int, default=5)
    parser.add_argument("--planning-time", type=float, default=5.0)
    parser.add_argument("--velocity-scaling", type=float, default=0.05)
    parser.add_argument("--acceleration-scaling", type=float, default=0.05)
    parser.add_argument("--goal-tolerance", type=float, default=0.01)
    parser.add_argument("--workspace-size", type=float, default=2.0)
    parser.add_argument("--server-timeout", type=float, default=10.0)
    parser.add_argument("--result-timeout", type=float, default=60.0)
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Execute the planned motion. Without this flag, MoveIt only plans.",
    )
    add_post_execution_verification_arguments(parser)
    return parser.parse_args()


def load_action(args: argparse.Namespace) -> dict:
    if args.mock_action:
        return {
            "action": "primitive",
            "name": args.mock_action,
            "confidence": 1.0,
            "source": "mock",
        }

    if args.json:
        try:
            loaded = json.loads(args.json)
        except json.JSONDecodeError as exc:
            raise RuntimeError(f"Invalid --json input: {exc}") from exc
    else:
        try:
            loaded = json.loads(args.json_file.read_text(encoding="utf-8"))
        except OSError as exc:
            raise RuntimeError(f"Could not read {args.json_file}: {exc}") from exc
        except json.JSONDecodeError as exc:
            raise RuntimeError(f"Invalid JSON in {args.json_file}: {exc}") from exc

    if not isinstance(loaded, dict):
        raise RuntimeError("VLA output must be a JSON object.")
    return loaded


def decode_action(action: Mapping[str, object], args: argparse.Namespace) -> Tuple[Dict[str, float], str]:
    confidence = get_confidence(action)
    if confidence < args.min_confidence:
        raise RuntimeError(
            f"Rejected VLA action: confidence {confidence:.3f} is below "
            f"--min-confidence {args.min_confidence:.3f}."
        )

    action_type = str(action.get("action", "")).strip()
    if not action_type and "primitive" in action:
        action_type = "primitive"

    if action_type == "primitive":
        primitive_name = str(action.get("name", action.get("primitive", ""))).strip()
        if primitive_name not in PRIMITIVES:
            raise RuntimeError(
                f"Unknown primitive '{primitive_name}'. Available: {sorted(PRIMITIVES)}"
            )
        return dict(PRIMITIVES[primitive_name]), f"primitive:{primitive_name}"

    if action_type == "joint_delta":
        joint = str(action.get("joint", "")).strip()
        delta = parse_finite_float(action.get("delta"), "delta")
        return {joint: delta}, f"joint_delta:{joint}"

    if action_type == "joint_deltas":
        deltas = action.get("deltas")
        if not isinstance(deltas, dict):
            raise RuntimeError("'joint_deltas' action requires a 'deltas' object.")
        parsed = {
            str(joint).strip(): parse_finite_float(delta, f"deltas.{joint}")
            for joint, delta in deltas.items()
        }
        return parsed, "joint_deltas"

    raise RuntimeError(
        "Unsupported action. Use action='primitive', 'joint_delta', or 'joint_deltas'."
    )


def get_confidence(action: Mapping[str, object]) -> float:
    if "confidence" not in action:
        return 1.0
    return parse_finite_float(action["confidence"], "confidence")


def parse_finite_float(value: object, name: str) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError) as exc:
        raise RuntimeError(f"{name} must be a number.") from exc
    if not math.isfinite(parsed):
        raise RuntimeError(f"{name} must be finite.")
    return parsed


def validate_deltas(deltas: Mapping[str, float], args: argparse.Namespace) -> None:
    if not deltas:
        raise RuntimeError("VLA action produced no joint deltas.")

    moving_joints = [joint for joint, delta in deltas.items() if abs(delta) > 1e-9]
    if len(moving_joints) > args.max_moving_joints:
        raise RuntimeError(
            f"Rejected action: {len(moving_joints)} moving joints exceeds "
            f"--max-moving-joints {args.max_moving_joints}."
        )

    total = 0.0
    for joint, delta in deltas.items():
        if joint not in JOINTS:
            raise RuntimeError(f"Unknown joint '{joint}'. Expected one of: {JOINTS}")
        if abs(delta) > args.max_delta:
            raise RuntimeError(
                f"Rejected action: {joint} delta {delta:.4f} exceeds "
                f"--max-delta {args.max_delta:.4f}."
            )
        total += abs(delta)

    if total > args.max_total_delta:
        raise RuntimeError(
            f"Rejected action: total absolute delta {total:.4f} exceeds "
            f"--max-total-delta {args.max_total_delta:.4f}."
        )


def build_target(current: Mapping[str, float], deltas: Mapping[str, float]) -> Dict[str, float]:
    target = dict(current)
    for joint, delta in deltas.items():
        target[joint] += delta
    return target


def validate_target_limits(target: Mapping[str, float], args: argparse.Namespace) -> None:
    for joint in JOINTS:
        lower, upper = JOINT_LIMITS[joint]
        value = target[joint]
        if value < lower + args.joint_limit_margin or value > upper - args.joint_limit_margin:
            raise RuntimeError(
                f"Rejected target: {joint}={value:.4f} is too close to/outside limits "
                f"[{lower:.4f}, {upper:.4f}] with margin {args.joint_limit_margin:.4f}."
            )


def validate_common_args(args: argparse.Namespace) -> None:
    if not (0.0 < args.min_confidence <= 1.0):
        raise RuntimeError("--min-confidence must be in (0, 1].")
    if args.max_delta <= 0.0:
        raise RuntimeError("--max-delta must be positive.")
    if args.max_total_delta <= 0.0:
        raise RuntimeError("--max-total-delta must be positive.")
    if args.max_moving_joints <= 0:
        raise RuntimeError("--max-moving-joints must be positive.")
    if not (0.0 < args.velocity_scaling <= 1.0):
        raise RuntimeError("--velocity-scaling must be in (0, 1].")
    if not (0.0 < args.acceleration_scaling <= 1.0):
        raise RuntimeError("--acceleration-scaling must be in (0, 1].")
    if args.goal_tolerance <= 0.0:
        raise RuntimeError("--goal-tolerance must be positive.")
    if args.workspace_size <= 0.0:
        raise RuntimeError("--workspace-size must be positive.")
    validate_post_execution_verification_args(args)


def format_deltas(deltas: Mapping[str, float]) -> str:
    lines = ["validated VLA joint deltas:"]
    for joint in JOINTS:
        if joint in deltas:
            lines.append(f"  {joint}: {deltas[joint]:+.6f} rad")
    return "\n".join(lines)


def main() -> int:
    args = parse_args()

    try:
        validate_common_args(args)
        raw_action = load_action(args)
        deltas, action_label = decode_action(raw_action, args)
        validate_deltas(deltas, args)
    except RuntimeError as exc:
        print(f"Input rejected: {exc}", file=sys.stderr)
        return 2

    rclpy.init()
    node = MoveItJointGoalClient(args.state_topic, args.plan_service, args.execute_action)

    try:
        current = node.wait_for_joint_state(timeout=5.0)
        target = build_target(current, deltas)
        validate_target_limits(target, args)

        print(f"VLA action accepted: {action_label}")
        print(format_deltas(deltas))
        print(format_joint_table(current, target))
        print(
            f"\nMode: {'EXECUTE' if args.execute else 'PLAN ONLY'}\n"
            f"MoveIt group: {args.group}\n"
            f"Velocity scaling: {args.velocity_scaling:.3f}\n"
            f"Acceleration scaling: {args.acceleration_scaling:.3f}\n"
            f"Goal tolerance: {args.goal_tolerance:.4f} rad\n"
        )

        node.wait_for_plan_service(timeout=args.server_timeout)
        plan_request = make_plan_request(node, current, target, args)
        plan_response = node.plan(plan_request, timeout=args.result_timeout)

        plan_code = plan_response.error_code.val
        print(trajectory_summary(plan_response.trajectory))
        print(f"Planning result: {error_name(plan_code)}")

        if plan_code != MoveItErrorCodes.SUCCESS:
            return 1

        if not args.execute:
            print("Plan-only run complete. Add --execute to move the robot.")
            return 0

        node.wait_for_execute_action(timeout=args.server_timeout)
        execute_result = node.execute(plan_response.trajectory, timeout=args.result_timeout)
        execute_code = execute_result.error_code.val
        print(f"Execution result: {error_name(execute_code)}")
        if execute_code != MoveItErrorCodes.SUCCESS:
            return 1
        return 0 if run_post_execution_verification(node, current, target, args) else 1

    except KeyboardInterrupt:
        print("Interrupted.")
        return 130
    except RuntimeError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
