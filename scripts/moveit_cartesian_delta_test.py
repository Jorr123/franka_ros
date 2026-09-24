#!/usr/bin/env python3
"""Plan or execute one small relative Cartesian translation through MoveIt.

The translation is expressed in MoveIt's planning frame. The end-effector
orientation is held constant and MoveIt's Cartesian path service interpolates
the requested straight line. Real motion is impossible unless --execute is
provided.
"""

import argparse
from dataclasses import dataclass
import math
import sys
import time
from typing import Dict

from geometry_msgs.msg import Pose
import rclpy
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetCartesianPath
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener

from cartesian_motion_safety import (
    format_cartesian_verification_report,
    format_trajectory_safety_report,
    scale_timed_trajectory,
    trajectory_final_positions,
    validate_trajectory,
    verify_cartesian_motion,
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
    MoveItErrorCodes.INVALID_GROUP_NAME: "INVALID_GROUP_NAME",
    MoveItErrorCodes.INVALID_LINK_NAME: "INVALID_LINK_NAME",
    MoveItErrorCodes.FRAME_TRANSFORM_FAILURE: "FRAME_TRANSFORM_FAILURE",
    MoveItErrorCodes.ROBOT_STATE_STALE: "ROBOT_STATE_STALE",
    MoveItErrorCodes.NO_IK_SOLUTION: "NO_IK_SOLUTION",
}


@dataclass(frozen=True)
class JointStateSnapshot:
    positions: Dict[str, float]
    velocities: Dict[str, float]
    sequence: int


class MoveItCartesianDeltaClient(Node):
    def __init__(
        self,
        state_topic: str,
        cartesian_service: str,
        execute_action: str,
    ) -> None:
        super().__init__("moveit_cartesian_delta_test")
        self.state_topic = state_topic
        self._joint_state_sequence = 0
        self._latest_joint_state = None
        self.subscription = self.create_subscription(
            JointState, state_topic, self._joint_state_callback, 10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cartesian_client = self.create_client(
            GetCartesianPath, cartesian_service
        )
        self.execute_client = ActionClient(
            self, ExecuteTrajectory, execute_action
        )

    @property
    def joint_state_sequence(self) -> int:
        return self._joint_state_sequence

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
        self._latest_joint_state = JointStateSnapshot(
            positions={joint: positions[joint] for joint in JOINTS},
            velocities={
                joint: velocities[joint]
                for joint in JOINTS
                if joint in velocities
            },
            sequence=self._joint_state_sequence,
        )

    def wait_for_joint_state(
        self,
        timeout: float,
        *,
        after_sequence: int = -1,
        require_velocities: bool = True,
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
        raise RuntimeError(
            f"Timed out waiting for a fresh, complete FR3 joint state on "
            f"{self.state_topic}."
        )

    def spin_for(self, duration: float) -> None:
        deadline = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            rclpy.spin_once(
                self, timeout_sec=min(0.1, max(0.0, remaining))
            )

    def wait_for_transform(
        self, planning_frame: str, ee_link: str, timeout: float
    ):
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
            f"Timed out waiting for TF {planning_frame} -> {ee_link}. "
            f"Last error: {last_error}"
        )

    def wait_for_cartesian_service(self, timeout: float) -> None:
        if not self.cartesian_client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(
                "Timed out waiting for /compute_cartesian_path. "
                "Is the FR3 MoveIt launch running?"
            )

    def wait_for_execute_action(self, timeout: float) -> None:
        if not self.execute_client.wait_for_server(timeout_sec=timeout):
            raise RuntimeError(
                "Timed out waiting for /execute_trajectory. "
                "Is the FR3 MoveIt launch running?"
            )

    def cartesian_path(
        self, request: GetCartesianPath.Request, timeout: float
    ):
        future = self.cartesian_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            raise RuntimeError("Timed out waiting for Cartesian planning.")
        if future.result() is None:
            raise RuntimeError(
                "MoveIt Cartesian service returned no response."
            )
        return future.result()

    def execute(self, trajectory, timeout: float):
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        send_future = self.execute_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(
            self, send_future, timeout_sec=timeout
        )
        if not send_future.done():
            raise RuntimeError("Timed out sending trajectory to MoveIt.")

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("MoveIt rejected the execution request.")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=timeout
        )
        if not result_future.done():
            raise RuntimeError("Timed out waiting for execution result.")
        return result_future.result().result


def make_target_pose(transform, args: argparse.Namespace) -> Pose:
    pose = Pose()
    pose.position.x = transform.transform.translation.x + args.dx
    pose.position.y = transform.transform.translation.y + args.dy
    pose.position.z = transform.transform.translation.z + args.dz
    pose.orientation = transform.transform.rotation
    return pose


def make_cartesian_request(
    node: Node,
    target_pose: Pose,
    current_positions: Dict[str, float],
    args: argparse.Namespace,
) -> GetCartesianPath.Request:
    request = GetCartesianPath.Request()
    now = node.get_clock().now().to_msg()
    request.header.stamp = now
    request.header.frame_id = args.planning_frame

    request.start_state.joint_state.header.stamp = now
    request.start_state.joint_state.name = list(JOINTS)
    request.start_state.joint_state.position = [
        current_positions[joint] for joint in JOINTS
    ]
    request.start_state.is_diff = False

    request.group_name = args.group
    request.link_name = args.ee_link
    request.waypoints.append(target_pose)
    request.max_step = args.eef_step
    request.jump_threshold = args.jump_threshold
    request.revolute_jump_threshold = args.revolute_jump_threshold
    request.avoid_collisions = True
    return request


def validate_args(args: argparse.Namespace) -> None:
    values = (args.dx, args.dy, args.dz)
    if any(not math.isfinite(value) for value in values):
        raise RuntimeError("Cartesian deltas must be finite.")
    distance = math.sqrt(sum(value * value for value in values))
    if distance <= 0.0:
        raise RuntimeError("Provide a non-zero --dx, --dy, or --dz.")
    if distance > args.max_distance:
        raise RuntimeError(
            f"Requested Cartesian offset {distance:.6f} m exceeds "
            f"--max-distance {args.max_distance:.6f} m."
        )

    positive = (
        "max_distance",
        "eef_step",
        "max_start_velocity",
        "max_start_error",
        "max_joint_excursion",
        "max_joint_step",
        "max_planned_velocity",
        "max_planned_acceleration",
        "settle_time",
        "verification_timeout",
        "cartesian_verification_tolerance",
        "orientation_verification_tolerance",
        "joint_verification_tolerance",
        "max_settled_velocity",
        "minimum_cartesian_motion",
        "server_timeout",
        "tf_timeout",
        "result_timeout",
    )
    for name in positive:
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0.0:
            raise RuntimeError(
                f"--{name.replace('_', '-')} must be finite and positive."
            )

    if args.jump_threshold < 0.0:
        raise RuntimeError("--jump-threshold must be non-negative.")
    if args.revolute_jump_threshold <= 0.0:
        raise RuntimeError("--revolute-jump-threshold must be positive.")


def ensure_robot_settled(
    snapshot: JointStateSnapshot,
    max_velocity: float,
    *,
    phase: str,
) -> None:
    if not all(joint in snapshot.velocities for joint in JOINTS):
        raise RuntimeError(
            f"Cannot check {phase}: measured velocity is missing."
        )
    joint = max(JOINTS, key=lambda name: abs(snapshot.velocities[name]))
    velocity = abs(snapshot.velocities[joint])
    if velocity > max_velocity:
        raise RuntimeError(
            f"Robot is not settled {phase}: {joint} is moving at "
            f"{velocity:.6f} rad/s (limit {max_velocity:.6f})."
        )


def transform_position(transform) -> tuple[float, float, float]:
    translation = transform.transform.translation
    return (translation.x, translation.y, translation.z)


def transform_orientation(transform) -> tuple[float, float, float, float]:
    rotation = transform.transform.rotation
    return (rotation.x, rotation.y, rotation.z, rotation.w)


def pose_position(pose: Pose) -> tuple[float, float, float]:
    return (pose.position.x, pose.position.y, pose.position.z)


def pose_orientation(pose: Pose) -> tuple[float, float, float, float]:
    return (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    )


def error_name(code: int) -> str:
    return ERROR_NAMES.get(code, f"UNKNOWN({code})")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Plan or execute a small straight-line FR3 end-effector "
            "translation."
        )
    )
    parser.add_argument("--state-topic", default="/joint_states")
    parser.add_argument(
        "--cartesian-service", default="/compute_cartesian_path"
    )
    parser.add_argument("--execute-action", default="/execute_trajectory")
    parser.add_argument("--group", default="fr3_arm")
    parser.add_argument("--planning-frame", default="base")
    parser.add_argument(
        "--ee-link",
        default="fr3_hand_tcp",
        help="Use fr3_link8 if MoveIt was launched without the Franka hand.",
    )
    parser.add_argument(
        "--dx", type=float, default=0.0, help="Base-frame X delta in meters."
    )
    parser.add_argument(
        "--dy", type=float, default=0.0, help="Base-frame Y delta in meters."
    )
    parser.add_argument(
        "--dz", type=float, default=0.0, help="Base-frame Z delta in meters."
    )
    parser.add_argument(
        "--max-distance",
        type=float,
        default=0.02,
        help="Maximum length of one Cartesian command, in meters.",
    )
    parser.add_argument(
        "--eef-step",
        type=float,
        default=0.0005,
        help="Maximum Cartesian interpolation step, in meters.",
    )
    parser.add_argument(
        "--jump-threshold",
        type=float,
        default=2.0,
        help="MoveIt relative joint jump threshold.",
    )
    parser.add_argument(
        "--revolute-jump-threshold",
        type=float,
        default=0.10,
        help="MoveIt absolute revolute-joint jump threshold, in radians.",
    )
    parser.add_argument("--max-start-velocity", type=float, default=0.01)
    parser.add_argument("--max-start-error", type=float, default=0.01)
    parser.add_argument("--max-joint-excursion", type=float, default=0.10)
    parser.add_argument("--max-joint-step", type=float, default=0.03)
    parser.add_argument(
        "--max-planned-velocity",
        type=float,
        default=0.02,
        help="Absolute per-joint trajectory velocity limit, in rad/s.",
    )
    parser.add_argument(
        "--max-planned-acceleration",
        type=float,
        default=0.05,
        help="Absolute per-joint trajectory acceleration limit, in rad/s^2.",
    )
    parser.add_argument("--settle-time", type=float, default=1.0)
    parser.add_argument("--verification-timeout", type=float, default=2.0)
    parser.add_argument(
        "--cartesian-verification-tolerance", type=float, default=0.005
    )
    parser.add_argument(
        "--orientation-verification-tolerance", type=float, default=0.03
    )
    parser.add_argument(
        "--joint-verification-tolerance", type=float, default=0.01
    )
    parser.add_argument("--max-settled-velocity", type=float, default=0.01)
    parser.add_argument(
        "--minimum-cartesian-motion", type=float, default=0.001
    )
    parser.add_argument("--server-timeout", type=float, default=10.0)
    parser.add_argument("--tf-timeout", type=float, default=5.0)
    parser.add_argument("--result-timeout", type=float, default=60.0)
    parser.add_argument(
        "--execute",
        action="store_true",
        help=(
            "Execute the trajectory. Without this flag, the robot cannot move."
        ),
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        validate_args(args)
    except RuntimeError as exc:
        print(f"Input rejected: {exc}", file=sys.stderr)
        return 2

    rclpy.init()
    node = MoveItCartesianDeltaClient(
        args.state_topic,
        args.cartesian_service,
        args.execute_action,
    )

    try:
        start_snapshot = node.wait_for_joint_state(timeout=5.0)
        ensure_robot_settled(
            start_snapshot,
            args.max_start_velocity,
            phase="before planning",
        )
        start_transform = node.wait_for_transform(
            args.planning_frame, args.ee_link, args.tf_timeout
        )
        target_pose = make_target_pose(start_transform, args)

        print(
            f"Relative Cartesian command in frame '{args.planning_frame}'\n"
            f"End-effector link: {args.ee_link}\n"
            f"Current: x={start_transform.transform.translation.x:.6f}, "
            f"y={start_transform.transform.translation.y:.6f}, "
            f"z={start_transform.transform.translation.z:.6f} m\n"
            f"Delta:   dx={args.dx:+.6f}, dy={args.dy:+.6f}, "
            f"dz={args.dz:+.6f} m\n"
            f"Target:  x={target_pose.position.x:.6f}, "
            f"y={target_pose.position.y:.6f}, "
            f"z={target_pose.position.z:.6f} m\n"
            "Orientation: HOLD CURRENT\n"
            "Collision checking: ENABLED\n"
            f"Mode: {'EXECUTE' if args.execute else 'PLAN ONLY'}\n"
            "Joint trajectory limits:\n"
            f"  velocity:     {args.max_planned_velocity:.3f} rad/s\n"
            f"  acceleration: {args.max_planned_acceleration:.3f} rad/s^2\n"
        )

        node.wait_for_cartesian_service(timeout=args.server_timeout)
        request = make_cartesian_request(
            node, target_pose, start_snapshot.positions, args
        )
        response = node.cartesian_path(request, timeout=args.result_timeout)
        plan_code = response.error_code.val
        print(f"Cartesian path fraction: {response.fraction:.6f}")
        print(f"Planning result: {error_name(plan_code)}")

        if plan_code != MoveItErrorCodes.SUCCESS:
            return 1
        if response.fraction < 0.999:
            print(
                "Cartesian path is incomplete; execution is blocked.",
                file=sys.stderr,
            )
            return 1

        trajectory = response.solution
        try:
            timing_scale = scale_timed_trajectory(
                trajectory,
                max_velocity=args.max_planned_velocity,
                max_acceleration=args.max_planned_acceleration,
            )
            safety_report = validate_trajectory(
                trajectory,
                start_snapshot.positions,
                JOINTS,
                timing_scale=timing_scale,
                max_start_error=args.max_start_error,
                max_joint_excursion=args.max_joint_excursion,
                max_joint_step=args.max_joint_step,
                max_planned_velocity=args.max_planned_velocity,
                max_planned_acceleration=args.max_planned_acceleration,
            )
        except ValueError as exc:
            raise RuntimeError(str(exc)) from exc

        print("\n" + format_trajectory_safety_report(safety_report))
        if not args.execute:
            print(
                "\nPlan-only run complete. Inspect the path in RViz. "
                "Add --execute only after it looks correct."
            )
            return 0

        pre_execute = node.wait_for_joint_state(
            timeout=args.verification_timeout,
            after_sequence=start_snapshot.sequence,
        )
        ensure_robot_settled(
            pre_execute,
            args.max_start_velocity,
            phase="immediately before execution",
        )
        try:
            validate_trajectory(
                trajectory,
                pre_execute.positions,
                JOINTS,
                timing_scale=timing_scale,
                max_start_error=args.max_start_error,
                max_joint_excursion=args.max_joint_excursion,
                max_joint_step=args.max_joint_step,
                max_planned_velocity=args.max_planned_velocity,
                max_planned_acceleration=args.max_planned_acceleration,
            )
        except ValueError as exc:
            raise RuntimeError(
                f"Pre-execution measured-state check failed: {exc}"
            ) from exc
        print("\nPre-execution measured-state check: PASS")

        node.wait_for_execute_action(timeout=args.server_timeout)
        execute_result = node.execute(trajectory, timeout=args.result_timeout)
        execute_code = execute_result.error_code.val
        print(f"Execution result: {error_name(execute_code)}")
        execution_succeeded = execute_code == MoveItErrorCodes.SUCCESS
        if not execution_succeeded:
            print(
                "The controller rejected this command. Measured-state "
                "verification will run for diagnosis only; the command "
                "cannot be accepted as successful."
            )

        print(f"\nWaiting {args.settle_time:.3f} s for the robot to settle...")
        node.spin_for(args.settle_time)
        sequence_after_settle = node.joint_state_sequence
        final_snapshot = node.wait_for_joint_state(
            timeout=args.verification_timeout,
            after_sequence=sequence_after_settle,
        )
        final_transform = node.wait_for_transform(
            args.planning_frame, args.ee_link, args.tf_timeout
        )
        expected_final_joints = trajectory_final_positions(trajectory)
        try:
            verification = verify_cartesian_motion(
                start_position=transform_position(start_transform),
                target_position=pose_position(target_pose),
                final_position=transform_position(final_transform),
                target_orientation=pose_orientation(target_pose),
                final_orientation=transform_orientation(final_transform),
                expected_joint_positions=expected_final_joints,
                measured_joint_positions=final_snapshot.positions,
                measured_joint_velocities=final_snapshot.velocities,
                target_error_limit=args.cartesian_verification_tolerance,
                orientation_error_limit=(
                    args.orientation_verification_tolerance
                ),
                joint_target_error_limit=args.joint_verification_tolerance,
                max_settled_velocity=args.max_settled_velocity,
                minimum_measured_motion=args.minimum_cartesian_motion,
            )
        except ValueError as exc:
            raise RuntimeError(
                f"Could not verify Cartesian motion: {exc}"
            ) from exc

        print(
            "\n"
            + format_cartesian_verification_report(
                verification,
                target_error_limit=args.cartesian_verification_tolerance,
                orientation_error_limit=(
                    args.orientation_verification_tolerance
                ),
                joint_target_error_limit=args.joint_verification_tolerance,
                max_settled_velocity=args.max_settled_velocity,
                execution_succeeded=execution_succeeded,
            )
        )
        return 0 if execution_succeeded and verification.passed else 1

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
