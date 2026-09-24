#!/usr/bin/env python3
"""Compatibility entry point for the guarded Cartesian-delta test.

Use moveit_cartesian_delta_test.py for new commands. Direct execution of this
legacy filename is forwarded to the guarded Cartesian implementation below.
"""

if __name__ == "__main__":
    import sys as _sys

    from moveit_cartesian_delta_test import main as _cartesian_main

    print(
        "Notice: moveit_pose_delta_test.py is now an alias for "
        "moveit_cartesian_delta_test.py."
    )
    _sys.exit(_cartesian_main())

import argparse
import math
import sys
import time

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import (
    Constraints,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
)
from moveit_msgs.srv import GetCartesianPath, GetMotionPlan
from rclpy.action import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformException, TransformListener


ERROR_NAMES = {
    MoveItErrorCodes.SUCCESS: "SUCCESS",
    MoveItErrorCodes.FAILURE: "FAILURE",
    MoveItErrorCodes.PLANNING_FAILED: "PLANNING_FAILED",
    MoveItErrorCodes.INVALID_MOTION_PLAN: "INVALID_MOTION_PLAN",
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
    MoveItErrorCodes.INVALID_LINK_NAME: "INVALID_LINK_NAME",
    MoveItErrorCodes.FRAME_TRANSFORM_FAILURE: "FRAME_TRANSFORM_FAILURE",
    MoveItErrorCodes.ROBOT_STATE_STALE: "ROBOT_STATE_STALE",
    MoveItErrorCodes.NO_IK_SOLUTION: "NO_IK_SOLUTION",
}


class MoveItPoseDeltaClient(Node):
    def __init__(
        self,
        plan_service: str,
        cartesian_service: str,
        execute_action: str,
    ) -> None:
        super().__init__("moveit_pose_delta_test")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.plan_client = self.create_client(GetMotionPlan, plan_service)
        self.cartesian_client = self.create_client(GetCartesianPath, cartesian_service)
        self.execute_client = ActionClient(self, ExecuteTrajectory, execute_action)

    def wait_for_transform(self, planning_frame: str, ee_link: str, timeout: float):
        deadline = time.monotonic() + timeout
        last_error = ""
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                return self.tf_buffer.lookup_transform(
                    planning_frame, ee_link, rclpy.time.Time()
                )
            except TransformException as exc:
                last_error = str(exc)
        raise RuntimeError(
            f"Timed out waiting for TF {planning_frame} -> {ee_link}. Last error: {last_error}"
        )

    def wait_for_plan_service(self, timeout: float) -> None:
        if not self.plan_client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(
                "Timed out waiting for MoveIt planning service. "
                "Is moveit.launch.py running?"
            )

    def wait_for_cartesian_service(self, timeout: float) -> None:
        if not self.cartesian_client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(
                "Timed out waiting for MoveIt Cartesian path service. "
                "Is moveit.launch.py running?"
            )

    def wait_for_execute_action(self, timeout: float) -> None:
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

    def cartesian_path(self, request: GetCartesianPath.Request, timeout: float):
        future = self.cartesian_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            raise RuntimeError("Timed out waiting for MoveIt Cartesian path result.")
        return future.result()

    def execute(self, trajectory, timeout: float):
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


def make_pose_from_transform(transform, dx: float, dy: float, dz: float) -> Pose:
    pose = Pose()
    pose.position.x = transform.transform.translation.x + dx
    pose.position.y = transform.transform.translation.y + dy
    pose.position.z = transform.transform.translation.z + dz
    pose.orientation = transform.transform.rotation
    return pose


def make_goal_constraints(target_pose: Pose, args: argparse.Namespace) -> Constraints:
    constraints = Constraints()
    constraints.name = "fr3_pose_delta_goal"

    sphere = SolidPrimitive()
    sphere.type = SolidPrimitive.SPHERE
    sphere.dimensions = [args.position_tolerance]

    sphere_pose = Pose()
    sphere_pose.position = target_pose.position
    sphere_pose.orientation.w = 1.0

    position_constraint = PositionConstraint()
    position_constraint.header.frame_id = args.planning_frame
    position_constraint.link_name = args.ee_link
    position_constraint.constraint_region.primitives.append(sphere)
    position_constraint.constraint_region.primitive_poses.append(sphere_pose)
    position_constraint.weight = 1.0

    orientation_constraint = OrientationConstraint()
    orientation_constraint.header.frame_id = args.planning_frame
    orientation_constraint.link_name = args.ee_link
    orientation_constraint.orientation = target_pose.orientation
    orientation_constraint.absolute_x_axis_tolerance = args.orientation_tolerance
    orientation_constraint.absolute_y_axis_tolerance = args.orientation_tolerance
    orientation_constraint.absolute_z_axis_tolerance = args.orientation_tolerance
    orientation_constraint.weight = 1.0

    constraints.position_constraints.append(position_constraint)
    constraints.orientation_constraints.append(orientation_constraint)
    return constraints


def make_plan_request(node: Node, target_pose: Pose, args: argparse.Namespace) -> GetMotionPlan.Request:
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

    # Empty diff means "use MoveIt's current robot state" as the start state.
    motion_request.start_state.is_diff = True
    motion_request.goal_constraints.append(make_goal_constraints(target_pose, args))
    motion_request.group_name = args.group
    motion_request.pipeline_id = args.pipeline
    motion_request.planner_id = args.planner
    motion_request.num_planning_attempts = args.planning_attempts
    motion_request.allowed_planning_time = args.planning_time
    motion_request.max_velocity_scaling_factor = args.velocity_scaling
    motion_request.max_acceleration_scaling_factor = args.acceleration_scaling
    return request


def make_cartesian_request(
    node: Node,
    target_pose: Pose,
    args: argparse.Namespace,
) -> GetCartesianPath.Request:
    request = GetCartesianPath.Request()
    request.header.stamp = node.get_clock().now().to_msg()
    request.header.frame_id = args.planning_frame
    request.start_state.is_diff = True
    request.group_name = args.group
    request.link_name = args.ee_link
    request.waypoints.append(target_pose)
    request.max_step = args.eef_step
    request.jump_threshold = args.jump_threshold
    request.revolute_jump_threshold = args.revolute_jump_threshold
    request.avoid_collisions = args.avoid_collisions
    return request


def stamp_trajectory_timing(trajectory, min_duration: float, max_joint_velocity: float):
    """Add conservative timestamps if the Cartesian service returned untimed points."""
    points = trajectory.joint_trajectory.points
    if not points:
        return trajectory

    has_timing = any(
        point.time_from_start.sec != 0 or point.time_from_start.nanosec != 0
        for point in points
    )
    if has_timing:
        return trajectory

    if len(points) == 1:
        duration = min_duration
        points[0].time_from_start = duration_to_msg(duration)
        return trajectory

    cumulative_times = [0.2]
    previous = points[0].positions
    for point in points[1:]:
        if previous and point.positions:
            max_delta = max(abs(a - b) for a, b in zip(point.positions, previous))
        else:
            max_delta = 0.0
        dt = max(0.05, max_delta / max_joint_velocity)
        cumulative_times.append(cumulative_times[-1] + dt)
        previous = point.positions

    scale = 1.0
    if cumulative_times[-1] < min_duration:
        scale = min_duration / max(cumulative_times[-1], 1e-6)

    for point, time_from_start in zip(points, cumulative_times):
        point.time_from_start = duration_to_msg(time_from_start * scale)
    return trajectory


def duration_to_msg(seconds: float) -> Duration:
    duration = Duration()
    duration.sec = int(seconds)
    duration.nanosec = int((seconds - duration.sec) * 1e9)
    return duration


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


def validate_args(args: argparse.Namespace) -> None:
    distance = math.sqrt(args.dx * args.dx + args.dy * args.dy + args.dz * args.dz)
    if distance <= 0.0:
        raise RuntimeError("Provide a non-zero --dx, --dy, or --dz.")
    if distance > args.max_distance:
        raise RuntimeError(
            f"Requested Cartesian offset {distance:.4f} m exceeds --max-distance "
            f"{args.max_distance:.4f} m."
        )
    if not (0.0 < args.velocity_scaling <= 1.0):
        raise RuntimeError("--velocity-scaling must be in (0, 1].")
    if not (0.0 < args.acceleration_scaling <= 1.0):
        raise RuntimeError("--acceleration-scaling must be in (0, 1].")
    if args.position_tolerance <= 0.0:
        raise RuntimeError("--position-tolerance must be positive.")
    if args.orientation_tolerance <= 0.0:
        raise RuntimeError("--orientation-tolerance must be positive.")
    if args.workspace_size <= 0.0:
        raise RuntimeError("--workspace-size must be positive.")
    if args.eef_step <= 0.0:
        raise RuntimeError("--eef-step must be positive.")
    if args.jump_threshold < 0.0:
        raise RuntimeError("--jump-threshold must be non-negative.")
    if args.revolute_jump_threshold < 0.0:
        raise RuntimeError("--revolute-jump-threshold must be non-negative.")
    if args.min_duration <= 0.0:
        raise RuntimeError("--min-duration must be positive.")
    if args.max_joint_velocity <= 0.0:
        raise RuntimeError("--max-joint-velocity must be positive.")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Send a small end-effector pose offset to MoveIt."
    )
    parser.add_argument("--plan-service", default="/plan_kinematic_path")
    parser.add_argument("--cartesian-service", default="/compute_cartesian_path")
    parser.add_argument("--execute-action", default="/execute_trajectory")
    parser.add_argument("--group", default="fr3_arm")
    parser.add_argument("--planning-frame", default="base")
    parser.add_argument(
        "--ee-link",
        default="fr3_hand_tcp",
        help="Use fr3_link8 if MoveIt was launched without the hand.",
    )
    parser.add_argument("--dx", type=float, default=0.0, help="X offset in meters.")
    parser.add_argument("--dy", type=float, default=0.0, help="Y offset in meters.")
    parser.add_argument("--dz", type=float, default=0.0, help="Z offset in meters.")
    parser.add_argument("--max-distance", type=float, default=0.05)
    parser.add_argument("--position-tolerance", type=float, default=0.01)
    parser.add_argument("--orientation-tolerance", type=float, default=0.20)
    parser.add_argument("--pipeline", default="")
    parser.add_argument("--planner", default="")
    parser.add_argument("--planning-attempts", type=int, default=10)
    parser.add_argument("--planning-time", type=float, default=8.0)
    parser.add_argument("--velocity-scaling", type=float, default=0.05)
    parser.add_argument("--acceleration-scaling", type=float, default=0.05)
    parser.add_argument("--workspace-size", type=float, default=2.0)
    parser.add_argument(
        "--mode",
        choices=["cartesian", "pose"],
        default="cartesian",
        help="cartesian follows a straight-line end-effector path; pose uses normal MoveIt planning.",
    )
    parser.add_argument(
        "--eef-step",
        type=float,
        default=0.005,
        help="Cartesian interpolation step in meters.",
    )
    parser.add_argument(
        "--jump-threshold",
        type=float,
        default=2.0,
        help="Relative joint jump threshold for Cartesian path.",
    )
    parser.add_argument(
        "--revolute-jump-threshold",
        type=float,
        default=0.15,
        help="Absolute revolute joint jump threshold in rad.",
    )
    parser.add_argument(
        "--allow-collisions",
        action="store_true",
        help="Disable collision avoidance for Cartesian path. Not recommended.",
    )
    parser.add_argument(
        "--min-duration",
        type=float,
        default=3.0,
        help="Minimum duration for executing a Cartesian path, in seconds.",
    )
    parser.add_argument(
        "--max-joint-velocity",
        type=float,
        default=0.08,
        help="Conservative timing velocity for Cartesian path points, rad/s.",
    )
    parser.add_argument("--server-timeout", type=float, default=10.0)
    parser.add_argument("--tf-timeout", type=float, default=5.0)
    parser.add_argument("--result-timeout", type=float, default=60.0)
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Execute the planned motion. Without this flag, MoveIt only plans.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        validate_args(args)
    except RuntimeError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 2

    rclpy.init()
    args.avoid_collisions = not args.allow_collisions

    node = MoveItPoseDeltaClient(
        args.plan_service,
        args.cartesian_service,
        args.execute_action,
    )

    try:
        transform = node.wait_for_transform(args.planning_frame, args.ee_link, args.tf_timeout)
        target_pose = make_pose_from_transform(transform, args.dx, args.dy, args.dz)

        print(
            f"Current {args.ee_link} in {args.planning_frame}:\n"
            f"  x={transform.transform.translation.x:.4f}, "
            f"y={transform.transform.translation.y:.4f}, "
            f"z={transform.transform.translation.z:.4f}\n"
            f"Target:\n"
            f"  x={target_pose.position.x:.4f}, "
            f"y={target_pose.position.y:.4f}, "
            f"z={target_pose.position.z:.4f}\n"
            f"Offset: dx={args.dx:.4f}, dy={args.dy:.4f}, dz={args.dz:.4f} m\n"
            f"Planning mode: {args.mode}\n"
            f"Mode: {'EXECUTE' if args.execute else 'PLAN ONLY'}\n"
            f"Velocity scaling: {args.velocity_scaling:.3f}\n"
            f"Acceleration scaling: {args.acceleration_scaling:.3f}\n"
        )

        if args.mode == "cartesian":
            node.wait_for_cartesian_service(timeout=args.server_timeout)
            cartesian_request = make_cartesian_request(node, target_pose, args)
            cartesian_response = node.cartesian_path(cartesian_request, timeout=args.result_timeout)
            trajectory = stamp_trajectory_timing(
                cartesian_response.solution,
                args.min_duration,
                args.max_joint_velocity,
            )
            plan_code = cartesian_response.error_code.val
            print(trajectory_summary(trajectory))
            print(f"Cartesian path fraction: {cartesian_response.fraction:.3f}")
            print(f"Planning result: {error_name(plan_code)}")
            if cartesian_response.fraction < 0.999:
                print("Cartesian path was incomplete; not safe to execute.")
                return 1
        else:
            node.wait_for_plan_service(timeout=args.server_timeout)
            plan_request = make_plan_request(node, target_pose, args)
            plan_response = node.plan(plan_request, timeout=args.result_timeout)
            trajectory = plan_response.trajectory
            plan_code = plan_response.error_code.val
            print(trajectory_summary(trajectory))
            print(f"Planning result: {error_name(plan_code)}")

        if plan_code != MoveItErrorCodes.SUCCESS:
            return 1

        if not args.execute:
            print("Plan-only run complete. Add --execute to move the robot.")
            return 0

        node.wait_for_execute_action(timeout=args.server_timeout)
        execute_result = node.execute(trajectory, timeout=args.result_timeout)
        execute_code = execute_result.error_code.val
        print(f"Execution result: {error_name(execute_code)}")
        return 0 if execute_code == MoveItErrorCodes.SUCCESS else 1

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
