#!/usr/bin/env python3
"""Plan or execute a small FR3 joint-space goal through MoveIt.

This script uses MoveIt's planning service first:

  /plan_kinematic_path

If --execute is given, it sends the returned trajectory to:

  /execute_trajectory

By default it only plans. That means the robot does not move unless --execute
is present.
"""

import argparse
from dataclasses import dataclass
import math
import sys
import time
from typing import Dict, List, Mapping, Tuple

import rclpy
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import GetMotionPlan
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState

from joint_motion_verification import (
    DEFAULT_JOINT7_TARGET_ERROR_LIMIT,
    DEFAULT_TARGET_ERROR_LIMIT,
    MotionVerificationReport,
    format_verification_report,
    verify_joint_motion,
)


JOINTS = [
    "fr3_joint1",
    "fr3_joint2",
    "fr3_joint3",
    "fr3_joint4",
    "fr3_joint5",
    "fr3_joint6",
    "fr3_joint7",
]


ERROR_NAMES = {
    MoveItErrorCodes.SUCCESS: "SUCCESS",
    MoveItErrorCodes.FAILURE: "FAILURE",
    MoveItErrorCodes.PLANNING_FAILED: "PLANNING_FAILED",
    MoveItErrorCodes.INVALID_MOTION_PLAN: "INVALID_MOTION_PLAN",
    MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
        "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
    MoveItErrorCodes.CONTROL_FAILED: "CONTROL_FAILED",
    MoveItErrorCodes.TIMED_OUT: "TIMED_OUT",
    MoveItErrorCodes.PREEMPTED: "PREEMPTED",
    MoveItErrorCodes.START_STATE_IN_COLLISION: "START_STATE_IN_COLLISION",
    MoveItErrorCodes.START_STATE_INVALID: "START_STATE_INVALID",
    MoveItErrorCodes.GOAL_IN_COLLISION: "GOAL_IN_COLLISION",
    MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED: "GOAL_CONSTRAINTS_VIOLATED",
    MoveItErrorCodes.GOAL_STATE_INVALID: "GOAL_STATE_INVALID",
    MoveItErrorCodes.INVALID_GROUP_NAME: "INVALID_GROUP_NAME",
    MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS: "INVALID_GOAL_CONSTRAINTS",
    MoveItErrorCodes.INVALID_ROBOT_STATE: "INVALID_ROBOT_STATE",
    MoveItErrorCodes.ROBOT_STATE_STALE: "ROBOT_STATE_STALE",
    MoveItErrorCodes.NO_IK_SOLUTION: "NO_IK_SOLUTION",
}


@dataclass(frozen=True)
class JointStateSnapshot:
    positions: Dict[str, float]
    velocities: Dict[str, float]
    sequence: int


class MoveItJointGoalClient(Node):
    def __init__(
        self,
        state_topic: str,
        plan_service: str,
        execute_action: str | None,
        node_name: str = "moveit_joint_goal_test",
    ) -> None:
        super().__init__(node_name)
        self.current: Dict[str, float] = {}
        self.current_velocity: Dict[str, float] = {}
        self._joint_state_sequence = 0
        self._latest_joint_state = None
        self.state_topic = state_topic
        self.subscription = self.create_subscription(
            JointState, state_topic, self._joint_state_callback, 10
        )
        self.plan_client = self.create_client(GetMotionPlan, plan_service)
        self.execute_client = (
            ActionClient(self, ExecuteTrajectory, execute_action)
            if execute_action is not None
            else None
        )

    def _joint_state_callback(self, msg: JointState) -> None:
        positions = {
            name: msg.position[index]
            for index, name in enumerate(msg.name)
            if name in JOINTS
            and index < len(msg.position)
            and math.isfinite(msg.position[index])
        }
        if not all(joint in positions for joint in JOINTS):
            return

        velocities = {
            name: msg.velocity[index]
            for index, name in enumerate(msg.name)
            if name in JOINTS
            and index < len(msg.velocity)
            and math.isfinite(msg.velocity[index])
        }

        self._joint_state_sequence += 1
        self.current = {joint: positions[joint] for joint in JOINTS}
        self.current_velocity = {
            joint: velocities[joint] for joint in JOINTS if joint in velocities
        }
        self._latest_joint_state = JointStateSnapshot(
            positions=dict(self.current),
            velocities=dict(self.current_velocity),
            sequence=self._joint_state_sequence,
        )

    @property
    def joint_state_sequence(self) -> int:
        return self._joint_state_sequence

    def wait_for_joint_state(self, timeout: float) -> Dict[str, float]:
        snapshot = self.wait_for_joint_state_snapshot(timeout=timeout)
        return dict(snapshot.positions)

    def wait_for_joint_state_snapshot(
        self,
        timeout: float,
        *,
        after_sequence: int = -1,
        require_velocities: bool = False,
    ) -> JointStateSnapshot:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            snapshot = self._latest_joint_state
            if snapshot is None or snapshot.sequence <= after_sequence:
                continue
            if require_velocities and not all(
                joint in snapshot.velocities for joint in JOINTS
            ):
                continue
            return snapshot

        missing = [joint for joint in JOINTS if joint not in self.current]
        if require_velocities:
            missing_velocities = [
                joint for joint in JOINTS if joint not in self.current_velocity
            ]
        else:
            missing_velocities = []
        freshness = (
            f" No sample newer than sequence {after_sequence} was received."
            if after_sequence >= 0
            else ""
        )
        raise RuntimeError(
            f"Timed out waiting for a complete FR3 joint state on {self.state_topic}. "
            f"Missing positions: {missing}; missing velocities: {missing_velocities}."
            f"{freshness}"
        )

    def spin_for(self, duration: float) -> None:
        deadline = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            rclpy.spin_once(self, timeout_sec=min(0.1, max(0.0, remaining)))

    def wait_for_plan_service(self, timeout: float) -> None:
        if not self.plan_client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(
                "Timed out waiting for MoveIt planning service. "
                "Is moveit.launch.py running?"
            )

    def wait_for_execute_action(self, timeout: float) -> None:
        if self.execute_client is None:
            raise RuntimeError("Trajectory execution is disabled for this client.")
        if not self.execute_client.wait_for_server(timeout_sec=timeout):
            raise RuntimeError(
                "Timed out waiting for MoveIt execute action. "
                "Is moveit.launch.py running?"
            )

    def plan(self, request: GetMotionPlan.Request, timeout: float):
        future = self.plan_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            raise RuntimeError("Timed out waiting for MoveIt planning result.")
        return future.result().motion_plan_response

    def execute(self, trajectory, timeout: float):
        if self.execute_client is None:
            raise RuntimeError("Trajectory execution is disabled for this client.")
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        send_future = self.execute_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=timeout)
        if not send_future.done():
            raise RuntimeError("Timed out sending trajectory to MoveIt.")

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            raise RuntimeError("MoveIt rejected the execution request.")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout)
        if not result_future.done():
            raise RuntimeError("Timed out waiting for execution result.")
        return result_future.result().result


def parse_assignment(text: str) -> Tuple[str, float]:
    if ":=" not in text:
        raise argparse.ArgumentTypeError(
            f"Expected JOINT:=VALUE, for example fr3_joint7:=0.03. Got: {text}"
        )
    joint, value_text = text.split(":=", 1)
    joint = joint.strip()
    if joint not in JOINTS:
        raise argparse.ArgumentTypeError(f"Unknown joint '{joint}'. Expected one of: {JOINTS}")
    try:
        value = float(value_text)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"Invalid number in '{text}'") from exc
    if not math.isfinite(value):
        raise argparse.ArgumentTypeError(f"Value must be finite in '{text}'")
    return joint, value


def build_target(
    current: Dict[str, float],
    targets: List[Tuple[str, float]],
    deltas: List[Tuple[str, float]],
) -> Dict[str, float]:
    goal = dict(current)
    for joint, value in targets:
        goal[joint] = value
    for joint, delta in deltas:
        goal[joint] += delta
    return goal


def make_goal_constraints(target: Dict[str, float], tolerance: float) -> Constraints:
    constraints = Constraints()
    constraints.name = "fr3_joint_goal"
    for joint in JOINTS:
        joint_constraint = JointConstraint()
        joint_constraint.joint_name = joint
        joint_constraint.position = target[joint]
        joint_constraint.tolerance_above = tolerance
        joint_constraint.tolerance_below = tolerance
        joint_constraint.weight = 1.0
        constraints.joint_constraints.append(joint_constraint)
    return constraints


def make_plan_request(
    node: Node,
    current: Dict[str, float],
    target: Dict[str, float],
    args: argparse.Namespace,
) -> GetMotionPlan.Request:
    request = GetMotionPlan.Request()
    motion_request = request.motion_plan_request

    now = node.get_clock().now().to_msg()
    motion_request.workspace_parameters.header.stamp = now
    motion_request.workspace_parameters.header.frame_id = args.planning_frame
    motion_request.workspace_parameters.min_corner.x = -args.workspace_size
    motion_request.workspace_parameters.min_corner.y = -args.workspace_size
    motion_request.workspace_parameters.min_corner.z = -args.workspace_size
    motion_request.workspace_parameters.max_corner.x = args.workspace_size
    motion_request.workspace_parameters.max_corner.y = args.workspace_size
    motion_request.workspace_parameters.max_corner.z = args.workspace_size

    motion_request.start_state.joint_state.header.stamp = now
    motion_request.start_state.joint_state.name = list(JOINTS)
    motion_request.start_state.joint_state.position = [current[joint] for joint in JOINTS]
    motion_request.start_state.is_diff = False

    motion_request.goal_constraints.append(make_goal_constraints(target, args.goal_tolerance))
    motion_request.group_name = args.group
    motion_request.pipeline_id = args.pipeline
    motion_request.planner_id = args.planner
    motion_request.num_planning_attempts = args.planning_attempts
    motion_request.allowed_planning_time = args.planning_time
    motion_request.max_velocity_scaling_factor = args.velocity_scaling
    motion_request.max_acceleration_scaling_factor = args.acceleration_scaling
    return request


def validate_request(
    current: Dict[str, float],
    target: Dict[str, float],
    args: argparse.Namespace,
) -> None:
    too_large = [
        joint for joint in JOINTS if abs(target[joint] - current[joint]) > args.max_delta
    ]
    if too_large:
        details = ", ".join(
            f"{joint}={target[joint] - current[joint]:+.6f} rad"
            for joint in too_large
        )
        raise RuntimeError(
            "Requested move is too large for this test script. "
            f"Joints over --max-delta={args.max_delta:.6f} rad: {details}"
        )

    if not (0.0 < args.velocity_scaling <= 1.0):
        raise RuntimeError("--velocity-scaling must be in (0, 1].")
    if not (0.0 < args.acceleration_scaling <= 1.0):
        raise RuntimeError("--acceleration-scaling must be in (0, 1].")
    if args.goal_tolerance <= 0.0:
        raise RuntimeError("--goal-tolerance must be positive.")
    if args.planning_attempts <= 0:
        raise RuntimeError("--planning-attempts must be positive.")
    if args.planning_time <= 0.0:
        raise RuntimeError("--planning-time must be positive.")
    if args.workspace_size <= 0.0:
        raise RuntimeError("--workspace-size must be positive.")
    validate_post_execution_verification_args(args)


def add_post_execution_verification_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--settle-time",
        type=float,
        default=1.0,
        help="Seconds to process joint states and let the robot settle after execution.",
    )
    parser.add_argument("--verification-timeout", type=float, default=2.0)
    parser.add_argument(
        "--verification-tolerance",
        type=float,
        default=DEFAULT_TARGET_ERROR_LIMIT,
        help="Maximum measured error from each commanded target, in radians.",
    )
    parser.add_argument(
        "--joint7-verification-tolerance",
        type=float,
        default=DEFAULT_JOINT7_TARGET_ERROR_LIMIT,
        help=(
            "Optional measured-error override for fr3_joint7, in radians. "
            "Other joints use --verification-tolerance."
        ),
    )
    parser.add_argument(
        "--non-commanded-drift-limit",
        type=float,
        default=0.01,
        help="Maximum measured movement of a non-commanded joint, in radians.",
    )
    parser.add_argument("--max-settled-velocity", type=float, default=0.01)
    parser.add_argument(
        "--minimum-measured-delta",
        type=float,
        default=0.001,
        help="Minimum measured motion used to reject a joint that barely moved.",
    )


def validate_post_execution_verification_args(args: argparse.Namespace) -> None:
    if not math.isfinite(args.settle_time) or args.settle_time < 0.0:
        raise RuntimeError("--settle-time must be finite and non-negative.")
    for name in (
        "verification_timeout",
        "verification_tolerance",
        "joint7_verification_tolerance",
        "non_commanded_drift_limit",
        "max_settled_velocity",
        "minimum_measured_delta",
    ):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0.0:
            raise RuntimeError(
                f"--{name.replace('_', '-')} must be finite and positive."
            )


def collect_post_execution_verification(
    node: MoveItJointGoalClient,
    before: Mapping[str, float],
    target: Mapping[str, float],
    args: argparse.Namespace,
) -> tuple[MotionVerificationReport, JointStateSnapshot]:
    print(f"\nWaiting {args.settle_time:.3f} s for the robot to settle...")
    node.spin_for(args.settle_time)

    sequence_after_settle = node.joint_state_sequence
    snapshot = node.wait_for_joint_state_snapshot(
        timeout=args.verification_timeout,
        after_sequence=sequence_after_settle,
        require_velocities=True,
    )
    try:
        joint_target_error_limits = {
            "fr3_joint7": args.joint7_verification_tolerance,
        }
        report = verify_joint_motion(
            JOINTS,
            before,
            target,
            snapshot.positions,
            snapshot.velocities,
            target_error_limit=args.verification_tolerance,
            joint_target_error_limits=joint_target_error_limits,
            non_commanded_drift_limit=args.non_commanded_drift_limit,
            max_settled_velocity=args.max_settled_velocity,
            minimum_measured_delta=args.minimum_measured_delta,
        )
    except ValueError as exc:
        raise RuntimeError(f"Could not verify executed motion: {exc}") from exc

    print(
        format_verification_report(
            report,
            target_error_limit=args.verification_tolerance,
            joint_target_error_limits=joint_target_error_limits,
            non_commanded_drift_limit=args.non_commanded_drift_limit,
            max_settled_velocity=args.max_settled_velocity,
        )
    )
    return report, snapshot


def run_post_execution_verification(
    node: MoveItJointGoalClient,
    before: Mapping[str, float],
    target: Mapping[str, float],
    args: argparse.Namespace,
) -> bool:
    report, _snapshot = collect_post_execution_verification(
        node,
        before,
        target,
        args,
    )
    return report.passed


def format_joint_table(current: Dict[str, float], target: Dict[str, float]) -> str:
    lines = ["joint        current(rad)  target(rad)   delta(rad)"]
    for joint in JOINTS:
        lines.append(
            f"{joint:<11} {current[joint]:>11.6f}  {target[joint]:>10.6f}  "
            f"{target[joint] - current[joint]:>10.6f}"
        )
    return "\n".join(lines)


def trajectory_summary(trajectory) -> str:
    joint_trajectory = trajectory.joint_trajectory
    point_count = len(joint_trajectory.points)
    if point_count == 0:
        return "MoveIt returned no joint trajectory points."

    final_time = joint_trajectory.points[-1].time_from_start
    duration = final_time.sec + final_time.nanosec * 1e-9
    return (
        f"Planned trajectory: {point_count} points, "
        f"{duration:.3f} s, joints={list(joint_trajectory.joint_names)}"
    )


def error_name(code: int) -> str:
    return ERROR_NAMES.get(code, f"UNKNOWN({code})")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Send a small FR3 joint-space goal to MoveIt."
    )
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
    parser.add_argument("--max-delta", type=float, default=0.15)
    parser.add_argument("--workspace-size", type=float, default=2.0)
    parser.add_argument(
        "--target",
        action="append",
        type=parse_assignment,
        default=[],
        metavar="JOINT:=POSITION",
    )
    parser.add_argument(
        "--delta",
        action="append",
        type=parse_assignment,
        default=[],
        metavar="JOINT:=DELTA",
    )
    parser.add_argument("--server-timeout", type=float, default=10.0)
    parser.add_argument("--result-timeout", type=float, default=60.0)
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Execute the planned motion. Without this flag, MoveIt only plans.",
    )
    add_post_execution_verification_arguments(parser)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.target and not args.delta:
        print("Error: provide at least one --target or --delta.", file=sys.stderr)
        return 2

    rclpy.init()
    node = MoveItJointGoalClient(args.state_topic, args.plan_service, args.execute_action)

    try:
        current = node.wait_for_joint_state(timeout=5.0)
        target = build_target(current, args.target, args.delta)
        validate_request(current, target, args)

        print(format_joint_table(current, target))
        print(
            f"\nMoveIt group: {args.group}\n"
            f"Plan service: {args.plan_service}\n"
            f"Execute action: {args.execute_action}\n"
            f"Mode: {'EXECUTE' if args.execute else 'PLAN ONLY'}\n"
            f"Velocity scaling: {args.velocity_scaling:.3f}\n"
            f"Acceleration scaling: {args.acceleration_scaling:.3f}\n"
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
