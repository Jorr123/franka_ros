#!/usr/bin/env python3
"""Plan one small joint-7-only FR3 trajectory without execution capability."""

import argparse
from dataclasses import dataclass
import math
import sys
from typing import Mapping, Sequence

from moveit_msgs.msg import MoveItErrorCodes
import rclpy

from moveit_joint_goal_test import (
    JOINTS,
    MoveItJointGoalClient,
    error_name,
    format_joint_table,
    make_plan_request,
    trajectory_summary,
)


@dataclass(frozen=True)
class Joint7PlanDiagnostics:
    point_count: int
    duration: float
    maximum_start_error: float
    maximum_start_error_joint: str
    joint7_planned_delta: float
    joint7_endpoint_error: float
    maximum_joint7_step: float
    maximum_joint7_velocity: float
    maximum_joint7_acceleration: float
    joint7_monotonic: bool
    maximum_holding_joint_excursion: float
    maximum_holding_joint: str
    holding_joints_within_tolerance: bool
    passed: bool


def analyze_joint7_plan(
    trajectory,
    current: Mapping[str, float],
    target: Mapping[str, float],
    *,
    holding_joint_tolerance: float,
    goal_tolerance: float,
) -> Joint7PlanDiagnostics:
    for label, value in (
        ("holding-joint tolerance", holding_joint_tolerance),
        ("goal tolerance", goal_tolerance),
    ):
        if not math.isfinite(value) or value <= 0.0:
            raise RuntimeError(f"{label} must be finite and positive.")

    names = tuple(trajectory.joint_trajectory.joint_names)
    if len(names) != len(set(names)) or set(names) != set(JOINTS):
        raise RuntimeError(
            "Planned trajectory does not contain exactly the seven FR3 "
            f"joints: {list(names)}"
        )
    points = tuple(trajectory.joint_trajectory.points)
    if len(points) < 2:
        raise RuntimeError(
            "Joint-7 diagnostic requires at least two trajectory points."
        )
    index = {joint: names.index(joint) for joint in JOINTS}
    times = []
    for point_number, point in enumerate(points):
        if len(point.positions) != len(names):
            raise RuntimeError(
                f"Trajectory point {point_number} has an invalid position "
                "vector."
            )
        values = tuple(float(value) for value in point.positions)
        if any(not math.isfinite(value) for value in values):
            raise RuntimeError(
                f"Trajectory point {point_number} contains non-finite "
                "positions."
            )
        time_value = (
            float(point.time_from_start.sec)
            + float(point.time_from_start.nanosec) * 1e-9
        )
        if not math.isfinite(time_value):
            raise RuntimeError("Trajectory contains a non-finite timestamp.")
        times.append(time_value)
    if any(
        current_time <= previous_time
        for previous_time, current_time in zip(times, times[1:])
    ):
        raise RuntimeError(
            "Trajectory timestamps are not strictly increasing."
        )

    first = points[0]
    final = points[-1]
    start_errors = {
        joint: abs(first.positions[index[joint]] - current[joint])
        for joint in JOINTS
    }
    start_joint = max(JOINTS, key=start_errors.get)

    holding_excursions = {
        joint: max(
            abs(point.positions[index[joint]] - current[joint])
            for point in points
        )
        for joint in JOINTS[:-1]
    }
    holding_joint = max(JOINTS[:-1], key=holding_excursions.get)
    maximum_holding_excursion = holding_excursions[holding_joint]

    joint7_values = [
        point.positions[index["fr3_joint7"]] for point in points
    ]
    requested_delta = target["fr3_joint7"] - current["fr3_joint7"]
    direction = 1.0 if requested_delta >= 0.0 else -1.0
    monotonic = all(
        direction * (current_value - previous_value) >= -1e-12
        for previous_value, current_value in zip(
            joint7_values,
            joint7_values[1:],
        )
    )
    maximum_step = max(
        abs(current_value - previous_value)
        for previous_value, current_value in zip(
            joint7_values,
            joint7_values[1:],
        )
    )
    maximum_velocity = _maximum_optional_field(
        points,
        index["fr3_joint7"],
        "velocities",
    )
    maximum_acceleration = _maximum_optional_field(
        points,
        index["fr3_joint7"],
        "accelerations",
    )
    endpoint_error = abs(
        final.positions[index["fr3_joint7"]]
        - target["fr3_joint7"]
    )
    holds = maximum_holding_excursion <= holding_joint_tolerance
    passed = (
        holds
        and monotonic
        and endpoint_error <= goal_tolerance
    )
    return Joint7PlanDiagnostics(
        point_count=len(points),
        duration=times[-1],
        maximum_start_error=start_errors[start_joint],
        maximum_start_error_joint=start_joint,
        joint7_planned_delta=(
            final.positions[index["fr3_joint7"]]
            - current["fr3_joint7"]
        ),
        joint7_endpoint_error=endpoint_error,
        maximum_joint7_step=maximum_step,
        maximum_joint7_velocity=maximum_velocity,
        maximum_joint7_acceleration=maximum_acceleration,
        joint7_monotonic=monotonic,
        maximum_holding_joint_excursion=maximum_holding_excursion,
        maximum_holding_joint=holding_joint,
        holding_joints_within_tolerance=holds,
        passed=passed,
    )


def _maximum_optional_field(
    points: Sequence,
    joint_index: int,
    field_name: str,
) -> float:
    maximum = 0.0
    for point_number, point in enumerate(points):
        values = getattr(point, field_name)
        if not values:
            continue
        if joint_index >= len(values):
            raise RuntimeError(
                f"Trajectory point {point_number} has an invalid "
                f"{field_name} vector."
            )
        value = abs(float(values[joint_index]))
        if not math.isfinite(value):
            raise RuntimeError(
                f"Trajectory point {point_number} contains a non-finite "
                f"{field_name} value."
            )
        maximum = max(maximum, value)
    return maximum


def format_joint7_plan_diagnostics(
    report: Joint7PlanDiagnostics,
    *,
    holding_joint_tolerance: float,
    goal_tolerance: float,
) -> str:
    return "\n".join(
        (
            "Joint-7-only plan diagnostics",
            f"  Points:                         {report.point_count}",
            f"  Duration:                       {report.duration:.6f} s",
            "  Maximum start-state error:      "
            f"{report.maximum_start_error:.6f} rad "
            f"({report.maximum_start_error_joint})",
            "  Planned joint-7 delta:           "
            f"{report.joint7_planned_delta:+.6f} rad",
            "  Joint-7 endpoint error:          "
            f"{report.joint7_endpoint_error:.6f} rad "
            f"(diagnostic limit {goal_tolerance:.6f})",
            "  Maximum joint-7 adjacent step:   "
            f"{report.maximum_joint7_step:.6f} rad",
            "  Maximum joint-7 velocity:        "
            f"{report.maximum_joint7_velocity:.6f} rad/s",
            "  Maximum joint-7 acceleration:    "
            f"{report.maximum_joint7_acceleration:.6f} rad/s^2",
            "  Joint-7 desired path monotonic:  "
            f"{'YES' if report.joint7_monotonic else 'NO'}",
            "  Maximum joints 1-6 excursion:    "
            f"{report.maximum_holding_joint_excursion:.6f} rad "
            f"({report.maximum_holding_joint}; diagnostic limit "
            f"{holding_joint_tolerance:.6f})",
            "  Joints 1-6 remain held:          "
            f"{'YES' if report.holding_joints_within_tolerance else 'NO'}",
            "  Diagnostic result:               "
            f"{'PASS' if report.passed else 'FAIL'}",
        )
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Plan one small FR3 joint-7-only trajectory. This diagnostic "
            "has no execution action client and cannot move the robot."
        )
    )
    parser.add_argument("--delta", type=float, default=0.005)
    parser.add_argument("--max-delta", type=float, default=0.01)
    parser.add_argument(
        "--holding-joint-tolerance",
        type=float,
        default=0.0001,
    )
    parser.add_argument("--state-topic", default="/joint_states")
    parser.add_argument("--plan-service", default="/plan_kinematic_path")
    parser.add_argument("--group", default="fr3_arm")
    parser.add_argument("--planning-frame", default="base")
    parser.add_argument("--pipeline", default="")
    parser.add_argument("--planner", default="")
    parser.add_argument("--planning-attempts", type=int, default=5)
    parser.add_argument("--planning-time", type=float, default=5.0)
    parser.add_argument("--velocity-scaling", type=float, default=0.03)
    parser.add_argument("--acceleration-scaling", type=float, default=0.03)
    parser.add_argument("--goal-tolerance", type=float, default=0.002)
    parser.add_argument("--workspace-size", type=float, default=2.0)
    parser.add_argument("--server-timeout", type=float, default=10.0)
    parser.add_argument("--result-timeout", type=float, default=60.0)
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    for name in (
        "max_delta",
        "holding_joint_tolerance",
        "planning_time",
        "velocity_scaling",
        "acceleration_scaling",
        "goal_tolerance",
        "workspace_size",
        "server_timeout",
        "result_timeout",
    ):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0.0:
            raise RuntimeError(
                f"--{name.replace('_', '-')} must be finite and positive."
            )
    if not math.isfinite(args.delta) or args.delta == 0.0:
        raise RuntimeError("--delta must be finite and non-zero.")
    if abs(args.delta) > args.max_delta:
        raise RuntimeError(
            f"Requested joint-7 delta {args.delta:+.6f} rad exceeds "
            f"--max-delta {args.max_delta:.6f} rad."
        )
    if args.velocity_scaling > 1.0:
        raise RuntimeError("--velocity-scaling must not exceed 1.0.")
    if args.acceleration_scaling > 1.0:
        raise RuntimeError("--acceleration-scaling must not exceed 1.0.")
    if args.planning_attempts <= 0:
        raise RuntimeError("--planning-attempts must be positive.")


def main() -> int:
    args = parse_args()
    try:
        validate_args(args)
    except RuntimeError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 2

    rclpy.init()
    node = MoveItJointGoalClient(
        args.state_topic,
        args.plan_service,
        None,
        node_name="moveit_joint7_plan_diagnostic",
    )
    try:
        current = node.wait_for_joint_state(timeout=args.server_timeout)
        target = dict(current)
        target["fr3_joint7"] += args.delta
        print(format_joint_table(current, target))
        print(
            "\nMode: PLAN ONLY BY DESIGN\n"
            "Execution action client: DISABLED\n"
            f"MoveIt group: {args.group}\n"
            f"Requested joint-7 delta: {args.delta:+.6f} rad\n"
            f"Velocity scaling: {args.velocity_scaling:.3f}\n"
            f"Acceleration scaling: {args.acceleration_scaling:.3f}\n"
        )

        node.wait_for_plan_service(timeout=args.server_timeout)
        request = make_plan_request(node, current, target, args)
        response = node.plan(request, timeout=args.result_timeout)
        code = response.error_code.val
        print(trajectory_summary(response.trajectory))
        print(f"Planning result: {error_name(code)}")
        if code != MoveItErrorCodes.SUCCESS:
            return 1

        report = analyze_joint7_plan(
            response.trajectory,
            current,
            target,
            holding_joint_tolerance=args.holding_joint_tolerance,
            goal_tolerance=args.goal_tolerance,
        )
        print(
            "\n"
            + format_joint7_plan_diagnostics(
                report,
                holding_joint_tolerance=args.holding_joint_tolerance,
                goal_tolerance=args.goal_tolerance,
            )
        )
        print(
            "\nPlan-only diagnostic complete. No execution interface exists "
            "in this process, so the robot was not commanded."
        )
        return 0 if report.passed else 1
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
