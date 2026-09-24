#!/usr/bin/env python3
"""Plan or execute one explicitly selected small FR3 Cartesian motion.

The existing Cartesian interpolation and joint-space modes remain available.
The pilz-lin mode requests one MoveIt Pilz LIN trajectory and never falls back
to another planner. Execution is disabled unless the returned trajectory passes
the joint, collision, dense FK, and measured-state checks.
"""

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import sys
import time
from typing import Dict, Mapping, Sequence

from builtin_interfaces.msg import Duration
from control_msgs.msg import JointTrajectoryControllerState
from controller_manager_msgs.srv import ListControllers
from franka_msgs.msg import FrankaRobotState
from geometry_msgs.msg import Pose
import rclpy
from rcl_interfaces.srv import GetParameters
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
)
from moveit_msgs.srv import (
    GetCartesianPath,
    GetPositionFK,
    GetPositionIK,
    GetStateValidity,
)
from rclpy.parameter import parameter_value_to_python
from rclpy.qos import qos_profile_sensor_data
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformException, TransformListener

from cartesian_execution_diagnostics import (
    ControllerRuntimeConfiguration,
    LocalForwardKinematics,
    PIDGains,
    analyze_execution_tracking,
    build_cartesian_tracking_samples,
    controller_state_sample_from_message,
    franka_inertia_configuration_from_robot_state,
    format_controller_configuration,
    format_execution_tracking_report,
    format_franka_inertia_configuration,
    maximum_joint_velocity,
    write_execution_trace_csv,
)
from cartesian_motion_safety import (
    format_cartesian_verification_report,
    scale_timed_trajectory,
    trajectory_final_positions,
    validate_trajectory,
    verify_cartesian_motion,
)
from joint_motion_verification import (
    format_verification_report,
    verify_joint_motion,
)
from moveit_joint_goal_test import (
    JOINTS,
    MoveItJointGoalClient,
    add_post_execution_verification_arguments,
    error_name,
    format_joint_table,
    make_plan_request,
    trajectory_summary,
    validate_post_execution_verification_args,
)


@dataclass(frozen=True)
class IKSafetyReport:
    total_delta: float
    max_delta: float
    max_delta_joint: str
    wrist_delta: float


@dataclass(frozen=True)
class IKCandidate:
    attempt: int
    target: Dict[str, float]
    safety: IKSafetyReport


@dataclass(frozen=True)
class PlannedCartesianPathReport:
    point_count: int
    requested_distance: float
    max_cross_track_error: float
    max_cross_track_index: int
    max_vertical_line_error: float
    max_vertical_line_error_index: int
    minimum_z_offset: float
    max_progress_regression: float
    max_progress_regression_index: int
    max_segment_overrun: float
    max_segment_overrun_index: int
    final_target_error: float
    max_orientation_error: float
    max_orientation_error_index: int
    final_orientation_error: float


PILZ_EXECUTION_CHECKS = frozenset(
    {
        "nonempty_trajectory",
        "joint_trajectory",
        "joint_excursions",
        "collision_free",
        "cartesian_path",
    }
)


class CartesianIKJointClient(MoveItJointGoalClient):
    def __init__(
        self,
        state_topic: str,
        ik_service: str,
        plan_service: str,
        cartesian_service: str,
        fk_service: str,
        state_validity_service: str,
        execute_action: str,
        controller_name: str,
        controller_state_topic: str,
        controller_manager_service: str,
        controller_parameter_service: str,
        robot_description_parameter_service: str,
        robot_state_topic: str,
    ) -> None:
        super().__init__(
            state_topic,
            plan_service,
            execute_action,
            node_name="moveit_cartesian_ik_joint_test",
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.ik_client = self.create_client(GetPositionIK, ik_service)
        self.cartesian_client = self.create_client(
            GetCartesianPath,
            cartesian_service,
        )
        self.fk_client = self.create_client(GetPositionFK, fk_service)
        self.state_validity_client = self.create_client(
            GetStateValidity,
            state_validity_service,
        )
        self.controller_name = controller_name.lstrip("/")
        self.controller_state_topic = controller_state_topic
        self.controller_manager_service = controller_manager_service
        self.controller_parameter_service = controller_parameter_service
        self.robot_description_parameter_service = (
            robot_description_parameter_service
        )
        self.robot_state_topic = robot_state_topic
        self._controller_state_subscription = None
        self._controller_state_samples = []
        self._controller_state_error = None

    def wait_for_transform(
        self,
        planning_frame: str,
        ee_link: str,
        timeout: float,
    ):
        deadline = time.monotonic() + timeout
        last_error = ""
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                return self.tf_buffer.lookup_transform(
                    planning_frame,
                    ee_link,
                    rclpy.time.Time(),
                )
            except TransformException as exc:
                last_error = str(exc)
        raise RuntimeError(
            f"Timed out waiting for TF {planning_frame} -> {ee_link}. "
            f"Last error: {last_error}"
        )

    def wait_for_ik_service(self, timeout: float) -> None:
        if not self.ik_client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(
                "Timed out waiting for /compute_ik. "
                "Is the FR3 MoveIt launch running?"
            )

    def compute_ik(self, request: GetPositionIK.Request, timeout: float):
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            raise RuntimeError("Timed out waiting for MoveIt IK.")
        response = future.result()
        if response is None:
            raise RuntimeError("MoveIt IK service returned no response.")
        return response

    def wait_for_cartesian_service(self, timeout: float) -> None:
        if not self.cartesian_client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(
                "Timed out waiting for /compute_cartesian_path. "
                "Is the FR3 MoveIt launch running?"
            )

    def cartesian_path(
        self,
        request: GetCartesianPath.Request,
        timeout: float,
    ):
        future = self.cartesian_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            raise RuntimeError("Timed out waiting for MoveIt Cartesian path.")
        response = future.result()
        if response is None:
            raise RuntimeError(
                "MoveIt Cartesian path service returned no response."
            )
        return response

    def wait_for_fk_service(self, timeout: float) -> None:
        if not self.fk_client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(
                "Timed out waiting for /compute_fk. "
                "Is the FR3 MoveIt launch running?"
            )

    def compute_fk(self, request: GetPositionFK.Request, timeout: float):
        future = self.fk_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            raise RuntimeError("Timed out waiting for MoveIt FK.")
        response = future.result()
        if response is None:
            raise RuntimeError("MoveIt FK service returned no response.")
        return response

    def wait_for_state_validity_service(self, timeout: float) -> None:
        if not self.state_validity_client.wait_for_service(
            timeout_sec=timeout
        ):
            raise RuntimeError(
                "Timed out waiting for /check_state_validity. "
                "Is the FR3 MoveIt launch running?"
            )

    def check_state_validity(
        self,
        request: GetStateValidity.Request,
        timeout: float,
    ):
        future = self.state_validity_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            raise RuntimeError(
                "Timed out waiting for MoveIt state-validity result."
            )
        response = future.result()
        if response is None:
            raise RuntimeError(
                "MoveIt state-validity service returned no response."
            )
        return response

    def inspect_active_controller(
        self,
        timeout: float,
    ) -> ControllerRuntimeConfiguration:
        client = self.create_client(
            ListControllers,
            self.controller_manager_service,
        )
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                raise RuntimeError(
                    "Timed out waiting for the controller-manager "
                    "list_controllers service."
                )
            future = client.call_async(ListControllers.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            if not future.done() or future.result() is None:
                raise RuntimeError(
                    "Timed out reading the active controller configuration."
                )
            matches = [
                controller
                for controller in future.result().controller
                if controller.name.lstrip("/") == self.controller_name
            ]
            if len(matches) != 1:
                raise RuntimeError(
                    f"Could not find exactly one controller named "
                    f"{self.controller_name}."
                )
            controller = matches[0]
        finally:
            self.destroy_client(client)

        parameter_names = [
            "command_interfaces",
            "constraints.goal_time",
            "constraints.stopped_velocity_tolerance",
        ]
        gain_fields = ("p", "i", "d", "i_clamp", "ff_velocity_scale")
        for joint in JOINTS:
            parameter_names.extend(
                f"gains.{joint}.{field}" for field in gain_fields
            )
            parameter_names.extend(
                (
                    f"constraints.{joint}.goal",
                    f"constraints.{joint}.trajectory",
                )
            )
        parameters = self.get_remote_parameters(
            self.controller_parameter_service,
            parameter_names,
            timeout,
        )
        configured_interfaces = parameters["command_interfaces"]
        if not isinstance(configured_interfaces, (list, tuple)):
            raise RuntimeError(
                "Controller command_interfaces parameter is not an array."
            )

        gains = {}
        for joint in JOINTS:
            values = {
                field: _optional_parameter_float(
                    parameters[f"gains.{joint}.{field}"]
                )
                for field in gain_fields
            }
            gains[joint] = PIDGains(**values)

        claimed = tuple(controller.claimed_interfaces)
        expected_effort = {f"{joint}/effort" for joint in JOINTS}
        effort_confirmed = (
            controller.state == "active"
            and tuple(configured_interfaces) == ("effort",)
            and set(claimed) == expected_effort
        )
        goal_tolerances = {
            joint: _optional_parameter_float(
                parameters[f"constraints.{joint}.goal"]
            )
            for joint in JOINTS
        }
        trajectory_tolerances = {
            joint: _optional_parameter_float(
                parameters[f"constraints.{joint}.trajectory"]
            )
            for joint in JOINTS
        }
        return ControllerRuntimeConfiguration(
            name=controller.name,
            state=controller.state,
            controller_type=controller.type,
            configured_command_interfaces=tuple(configured_interfaces),
            claimed_interfaces=claimed,
            effort_interface_confirmed=effort_confirmed,
            gains=gains,
            goal_time_tolerance=_optional_parameter_float(
                parameters["constraints.goal_time"]
            ),
            stopped_velocity_tolerance=_optional_parameter_float(
                parameters["constraints.stopped_velocity_tolerance"]
            ),
            goal_tolerances=goal_tolerances,
            trajectory_tolerances=trajectory_tolerances,
        )

    def get_remote_parameters(
        self,
        service_name: str,
        names: Sequence[str],
        timeout: float,
    ) -> dict[str, object]:
        client = self.create_client(GetParameters, service_name)
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                raise RuntimeError(
                    f"Timed out waiting for parameter service {service_name}."
                )
            request = GetParameters.Request()
            request.names = list(names)
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            if not future.done() or future.result() is None:
                raise RuntimeError(
                    f"Timed out reading parameters from {service_name}."
                )
            values = list(future.result().values)
        finally:
            self.destroy_client(client)
        if len(values) != len(names):
            raise RuntimeError(
                f"Parameter service {service_name} returned "
                f"{len(values)} values for {len(names)} names."
            )
        return {
            name: (
                None
                if value.type == 0
                else parameter_value_to_python(value)
            )
            for name, value in zip(names, values)
        }

    def get_robot_description(self, timeout: float) -> str:
        parameters = self.get_remote_parameters(
            self.robot_description_parameter_service,
            ("robot_description",),
            timeout,
        )
        description = parameters["robot_description"]
        if not isinstance(description, str) or not description.strip():
            raise RuntimeError(
                "The runtime robot_description parameter is missing or empty."
            )
        return description

    def read_inertia_configuration(self, timeout: float):
        latest = []

        def callback(message: FrankaRobotState) -> None:
            if not latest:
                latest.append(message)

        subscription = self.create_subscription(
            FrankaRobotState,
            self.robot_state_topic,
            callback,
            qos_profile_sensor_data,
        )
        try:
            deadline = time.monotonic() + timeout
            while rclpy.ok() and time.monotonic() < deadline and not latest:
                rclpy.spin_once(self, timeout_sec=0.1)
        finally:
            self.destroy_subscription(subscription)
        if not latest:
            raise RuntimeError(
                "Timed out reading the configured Franka inertias "
                f"from {self.robot_state_topic}."
            )
        return franka_inertia_configuration_from_robot_state(latest[0])

    def start_controller_state_recording(self) -> None:
        if self._controller_state_subscription is not None:
            raise RuntimeError(
                "Controller-state recording is already active."
            )
        self._controller_state_samples = []
        self._controller_state_error = None

        def callback(message: JointTrajectoryControllerState) -> None:
            try:
                sample = controller_state_sample_from_message(
                    message,
                    receipt_time=time.monotonic(),
                )
            except RuntimeError as exc:
                self._controller_state_error = str(exc)
                return
            self._controller_state_samples.append(sample)

        self._controller_state_subscription = self.create_subscription(
            JointTrajectoryControllerState,
            self.controller_state_topic,
            callback,
            qos_profile_sensor_data,
        )

    def wait_for_controller_state_sample(self, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        while (
            rclpy.ok()
            and time.monotonic() < deadline
            and not self._controller_state_samples
        ):
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self._controller_state_samples:
            detail = (
                f" Last rejected sample: {self._controller_state_error}"
                if self._controller_state_error
                else ""
            )
            raise RuntimeError(
                f"Timed out waiting for {self.controller_state_topic}."
                + detail
            )

    def stop_controller_state_recording(self):
        subscription = self._controller_state_subscription
        if subscription is not None:
            self.destroy_subscription(subscription)
            self._controller_state_subscription = None
        return tuple(self._controller_state_samples)


def make_target_pose(transform, dx: float, dy: float, dz: float) -> Pose:
    pose = Pose()
    pose.position.x = transform.transform.translation.x + dx
    pose.position.y = transform.transform.translation.y + dy
    pose.position.z = transform.transform.translation.z + dz
    pose.orientation = transform.transform.rotation
    return pose


def _optional_parameter_float(value) -> float | None:
    if value is None:
        return None
    result = float(value)
    if not math.isfinite(result):
        raise RuntimeError("Controller PID parameter is not finite.")
    return result


def seconds_to_duration(seconds: float) -> Duration:
    total_nanoseconds = int(round(seconds * 1e9))
    duration = Duration()
    duration.sec, duration.nanosec = divmod(
        total_nanoseconds,
        1_000_000_000,
    )
    return duration


def make_ik_request(
    node: CartesianIKJointClient,
    target_pose: Pose,
    seed_positions: Mapping[str, float],
    constraint_positions: Mapping[str, float],
    velocities: Mapping[str, float],
    args: argparse.Namespace,
) -> GetPositionIK.Request:
    request = GetPositionIK.Request()
    ik_request = request.ik_request
    now = node.get_clock().now().to_msg()

    ik_request.group_name = args.group
    ik_request.robot_state.joint_state.header.stamp = now
    ik_request.robot_state.joint_state.name = list(JOINTS)
    ik_request.robot_state.joint_state.position = [
        seed_positions[joint] for joint in JOINTS
    ]
    ik_request.robot_state.joint_state.velocity = [
        velocities.get(joint, 0.0) for joint in JOINTS
    ]
    ik_request.robot_state.is_diff = False

    ik_request.avoid_collisions = True
    ik_request.ik_link_name = args.ee_link
    ik_request.pose_stamped.header.stamp = now
    ik_request.pose_stamped.header.frame_id = args.planning_frame
    ik_request.pose_stamped.pose = target_pose
    ik_request.timeout = seconds_to_duration(args.ik_timeout)

    for joint in JOINTS:
        local_constraint = JointConstraint()
        local_constraint.joint_name = joint
        local_constraint.position = constraint_positions[joint]
        tolerance = (
            args.ik_joint7_tolerance
            if joint == "fr3_joint7"
            else args.max_delta
        )
        local_constraint.tolerance_above = tolerance
        local_constraint.tolerance_below = tolerance
        local_constraint.weight = 1.0
        ik_request.constraints.joint_constraints.append(local_constraint)
    return request


def add_local_joint_constraints(
    constraints,
    current_positions: Mapping[str, float],
    *,
    max_joint_delta: float,
    max_wrist_joint_delta: float,
) -> None:
    for joint in JOINTS:
        constraint = JointConstraint()
        constraint.joint_name = joint
        constraint.position = current_positions[joint]
        tolerance = (
            max_wrist_joint_delta
            if joint == "fr3_joint7"
            else max_joint_delta
        )
        constraint.tolerance_above = tolerance
        constraint.tolerance_below = tolerance
        constraint.weight = 1.0
        constraints.joint_constraints.append(constraint)


def make_cartesian_request(
    node: CartesianIKJointClient,
    target_pose: Pose,
    current_positions: Mapping[str, float],
    velocities: Mapping[str, float],
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
    request.start_state.joint_state.velocity = [
        velocities.get(joint, 0.0) for joint in JOINTS
    ]
    request.start_state.is_diff = False

    request.group_name = args.group
    request.link_name = args.ee_link
    request.waypoints.append(target_pose)
    request.max_step = args.eef_step
    request.jump_threshold = args.jump_threshold
    request.revolute_jump_threshold = args.revolute_jump_threshold
    request.avoid_collisions = True
    add_local_joint_constraints(
        request.path_constraints,
        current_positions,
        max_joint_delta=args.max_delta,
        max_wrist_joint_delta=args.max_wrist_joint_delta,
    )
    return request


def make_fk_request(
    node: CartesianIKJointClient,
    positions: Mapping[str, float],
    args: argparse.Namespace,
) -> GetPositionFK.Request:
    request = GetPositionFK.Request()
    now = node.get_clock().now().to_msg()
    request.header.stamp = now
    request.header.frame_id = args.planning_frame
    request.fk_link_names = [args.ee_link]
    request.robot_state.joint_state.header.stamp = now
    request.robot_state.joint_state.name = list(JOINTS)
    request.robot_state.joint_state.position = [
        positions[joint] for joint in JOINTS
    ]
    request.robot_state.is_diff = False
    return request


def make_state_validity_request(
    node: CartesianIKJointClient,
    positions: Mapping[str, float],
    args: argparse.Namespace,
) -> GetStateValidity.Request:
    request = GetStateValidity.Request()
    now = node.get_clock().now().to_msg()
    request.robot_state.joint_state.header.stamp = now
    request.robot_state.joint_state.name = list(JOINTS)
    request.robot_state.joint_state.position = [
        positions[joint] for joint in JOINTS
    ]
    request.robot_state.is_diff = True
    request.group_name = args.group
    return request


def extract_fk_pose(response, ee_link: str) -> Pose:
    names = list(response.fk_link_names)
    poses = list(response.pose_stamped)
    if len(names) != len(poses):
        raise RuntimeError(
            "MoveIt FK returned mismatched link-name and pose vectors."
        )
    normalized = [name.lstrip("/") for name in names]
    expected = ee_link.lstrip("/")
    if expected not in normalized:
        raise RuntimeError(
            f"MoveIt FK response is missing end-effector link {ee_link}."
        )
    return poses[normalized.index(expected)].pose


def trajectory_fk_poses(
    node: CartesianIKJointClient,
    sampled_positions: Sequence[Mapping[str, float]],
    args: argparse.Namespace,
) -> tuple[Pose, ...]:
    poses = []
    for sample_index, positions in enumerate(sampled_positions):
        request = make_fk_request(node, positions, args)
        response = node.compute_fk(request, timeout=args.fk_timeout)
        code = response.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(
                f"MoveIt FK failed for trajectory sample {sample_index}: "
                f"{error_name(code)}."
            )
        poses.append(extract_fk_pose(response, args.ee_link))
    return tuple(poses)


def validate_collision_free_samples(
    node: CartesianIKJointClient,
    sampled_positions: Sequence[Mapping[str, float]],
    args: argparse.Namespace,
) -> int:
    if not sampled_positions:
        raise RuntimeError(
            "Cannot check collisions for an empty trajectory."
        )
    for sample_index, positions in enumerate(sampled_positions):
        request = make_state_validity_request(node, positions, args)
        response = node.check_state_validity(
            request,
            timeout=args.fk_timeout,
        )
        if not response.valid:
            contact = ""
            if response.contacts:
                first = response.contacts[0]
                contact = (
                    f" between {first.contact_body_1} and "
                    f"{first.contact_body_2}"
                )
            raise RuntimeError(
                f"Collision detected at dense trajectory sample "
                f"{sample_index}{contact}. Execution is blocked."
            )
    return len(sampled_positions)


def sample_trajectory_joint_positions(
    trajectory,
    *,
    max_joint_step: float,
    max_samples: int,
) -> tuple[Dict[str, float], ...]:
    """Densely sample linear interpolation between trajectory points."""
    if not math.isfinite(max_joint_step) or max_joint_step <= 0.0:
        raise RuntimeError("FK joint sample step must be finite and positive.")
    if max_samples < 2:
        raise RuntimeError("FK path verification requires at least 2 samples.")

    names = list(trajectory.joint_trajectory.joint_names)
    if set(names) != set(JOINTS) or len(names) != len(JOINTS):
        raise RuntimeError(
            "Cannot verify Cartesian path FK: trajectory joint names do not "
            "match the seven FR3 joints."
        )
    points = list(trajectory.joint_trajectory.points)
    if len(points) < 2:
        raise RuntimeError(
            "Cannot verify Cartesian path FK: trajectory has fewer than "
            "two points."
        )
    point_positions = []
    for point_index, point in enumerate(points):
        if len(point.positions) != len(names):
            raise RuntimeError(
                f"Cannot verify Cartesian path FK: trajectory point "
                f"{point_index} has an invalid position vector."
            )
        values = tuple(float(value) for value in point.positions)
        if any(not math.isfinite(value) for value in values):
            raise RuntimeError(
                f"Cannot verify Cartesian path FK: trajectory point "
                f"{point_index} is not finite."
            )
        point_positions.append(values)

    samples = [dict(zip(names, point_positions[0]))]
    for segment_index in range(1, len(point_positions)):
        previous = point_positions[segment_index - 1]
        current = point_positions[segment_index]
        maximum_delta = max(
            abs(current[index] - previous[index])
            for index in range(len(names))
        )
        subdivisions = max(1, math.ceil(maximum_delta / max_joint_step))
        if len(samples) + subdivisions > max_samples:
            raise RuntimeError(
                "Cartesian FK verification would require more than "
                f"{max_samples} samples. Reduce the requested move or "
                "increase --fk-joint-sample-step."
            )
        for subdivision in range(1, subdivisions + 1):
            alpha = subdivision / subdivisions
            values = tuple(
                previous[index]
                + alpha * (current[index] - previous[index])
                for index in range(len(names))
            )
            samples.append(dict(zip(names, values)))
    return tuple(samples)


_IK_SEED_PATTERNS = (
    (1.0, -1.0, 1.0, -1.0, 1.0, -1.0),
    (-1.0, 1.0, -1.0, 1.0, -1.0, 1.0),
    (1.0, 1.0, -1.0, -1.0, 1.0, -1.0),
    (-1.0, -1.0, 1.0, 1.0, -1.0, 1.0),
    (1.0, -1.0, -1.0, 1.0, 1.0, -1.0),
    (-1.0, 1.0, 1.0, -1.0, -1.0, 1.0),
)


def make_ik_seed_candidates(
    current: Mapping[str, float],
    *,
    attempts: int,
    perturbation: float,
    max_offset: float,
) -> tuple[Dict[str, float], ...]:
    """Return deterministic local seeds while leaving joint 7 unchanged."""
    missing = [joint for joint in JOINTS if joint not in current]
    if missing:
        raise RuntimeError(f"IK seed state is missing joints: {missing}")
    if attempts <= 0:
        raise RuntimeError("IK seed generation requires at least one attempt.")

    seeds = [dict(current)]
    for index in range(attempts - 1):
        pattern = _IK_SEED_PATTERNS[index % len(_IK_SEED_PATTERNS)]
        level = 1 + index // len(_IK_SEED_PATTERNS)
        offset = min(perturbation * level, max_offset)
        seed = dict(current)
        for joint, direction in zip(JOINTS[:6], pattern):
            seed[joint] += direction * offset
        seeds.append(seed)
    return tuple(seeds)


def select_closest_ik_candidate(
    candidates: Sequence[IKCandidate],
) -> IKCandidate:
    if not candidates:
        raise RuntimeError("No valid local IK candidates were provided.")
    return min(
        candidates,
        key=lambda candidate: (
            candidate.safety.total_delta,
            candidate.safety.max_delta,
            candidate.attempt,
        ),
    )


def add_wrist_path_constraint(
    plan_request,
    current: Mapping[str, float],
    tolerance: float,
) -> None:
    constraint = JointConstraint()
    constraint.joint_name = "fr3_joint7"
    constraint.position = current["fr3_joint7"]
    constraint.tolerance_above = tolerance
    constraint.tolerance_below = tolerance
    constraint.weight = 1.0
    plan_request.motion_plan_request.path_constraints.joint_constraints.append(
        constraint
    )


def make_cartesian_pose_goal(
    plan_request,
    target_pose: Pose,
    args: argparse.Namespace,
) -> Constraints:
    """Build the Cartesian pose goal consumed by the Pilz LIN planner."""
    header = plan_request.motion_plan_request.workspace_parameters.header
    goal = Constraints()
    goal.name = "fr3_pilz_lin_pose_goal"

    target_region = SolidPrimitive()
    target_region.type = SolidPrimitive.SPHERE
    target_region.dimensions = [args.planned_cartesian_goal_tolerance]

    position_constraint = PositionConstraint()
    position_constraint.header = header
    position_constraint.link_name = args.ee_link
    position_constraint.constraint_region.primitives.append(target_region)
    position_constraint.constraint_region.primitive_poses.append(target_pose)
    position_constraint.weight = 1.0
    goal.position_constraints.append(position_constraint)

    orientation_constraint = OrientationConstraint()
    orientation_constraint.header = header
    orientation_constraint.link_name = args.ee_link
    orientation_constraint.orientation = target_pose.orientation
    orientation_constraint.absolute_x_axis_tolerance = (
        args.pilz_goal_orientation_tolerance
    )
    orientation_constraint.absolute_y_axis_tolerance = (
        args.pilz_goal_orientation_tolerance
    )
    orientation_constraint.absolute_z_axis_tolerance = (
        args.pilz_goal_orientation_tolerance
    )
    orientation_constraint.parameterization = (
        OrientationConstraint.ROTATION_VECTOR
    )
    orientation_constraint.weight = 1.0
    goal.orientation_constraints.append(orientation_constraint)
    return goal


def make_pilz_lin_plan_request(
    node: CartesianIKJointClient,
    target_pose: Pose,
    current_positions: Mapping[str, float],
    args: argparse.Namespace,
):
    """Create a MoveIt motion request for the Pilz Cartesian LIN planner."""
    request = make_plan_request(
        node,
        dict(current_positions),
        dict(current_positions),
        args,
    )
    motion_request = request.motion_plan_request
    # Pilz LIN requires an exactly zero-velocity start state. The measured
    # velocities are checked against --max-start-velocity before this request.
    motion_request.start_state.joint_state.velocity = [0.0] * len(JOINTS)
    motion_request.goal_constraints.clear()
    motion_request.goal_constraints.append(
        make_cartesian_pose_goal(request, target_pose, args)
    )
    motion_request.pipeline_id = "pilz_industrial_motion_planner"
    motion_request.planner_id = "LIN"
    motion_request.num_planning_attempts = 1
    return request


def extract_ik_solution(response) -> Dict[str, float]:
    names = list(response.solution.joint_state.name)
    positions = list(response.solution.joint_state.position)
    if len(names) != len(positions):
        raise RuntimeError(
            "MoveIt IK returned mismatched joint-name and position vectors."
        )
    if len(names) != len(set(names)):
        raise RuntimeError("MoveIt IK returned duplicate joint names.")

    solution = dict(zip(names, positions))
    missing = [joint for joint in JOINTS if joint not in solution]
    if missing:
        raise RuntimeError(
            f"MoveIt IK solution is missing FR3 joints: {missing}"
        )
    non_finite = [
        joint for joint in JOINTS if not math.isfinite(solution[joint])
    ]
    if non_finite:
        raise RuntimeError(
            f"MoveIt IK returned non-finite values for: {non_finite}"
        )
    return {joint: solution[joint] for joint in JOINTS}


def validate_ik_target(
    current: Mapping[str, float],
    target: Mapping[str, float],
    *,
    max_joint_delta: float,
    max_total_joint_delta: float,
    max_wrist_joint_delta: float,
) -> IKSafetyReport:
    missing = [
        joint
        for joint in JOINTS
        if joint not in current or joint not in target
    ]
    if missing:
        raise RuntimeError(f"IK safety check is missing joints: {missing}")

    deltas = {
        joint: abs(target[joint] - current[joint]) for joint in JOINTS
    }
    maximum_joint = max(JOINTS, key=deltas.get)
    maximum = deltas[maximum_joint]
    total = sum(deltas.values())
    wrist = deltas["fr3_joint7"]

    violations = []
    if maximum > max_joint_delta:
        violations.append(
            f"{maximum_joint} changes {maximum:.6f} rad "
            f"(limit {max_joint_delta:.6f})"
        )
    if total > max_total_joint_delta:
        violations.append(
            f"total absolute joint change is {total:.6f} rad "
            f"(limit {max_total_joint_delta:.6f})"
        )
    if wrist > max_wrist_joint_delta:
        violations.append(
            f"fr3_joint7 changes {wrist:.6f} rad "
            f"(limit {max_wrist_joint_delta:.6f}); this wrist joint did not "
            "track the previous Cartesian trajectory"
        )
    if total <= 1e-6:
        violations.append("IK solution contains no meaningful movement")
    if violations:
        raise RuntimeError("Unsafe IK target: " + "; ".join(violations))

    return IKSafetyReport(
        total_delta=total,
        max_delta=maximum,
        max_delta_joint=maximum_joint,
        wrist_delta=wrist,
    )


def find_local_ik_candidate(
    node: CartesianIKJointClient,
    target_pose: Pose,
    current_positions: Mapping[str, float],
    velocities: Mapping[str, float],
    args: argparse.Namespace,
) -> tuple[IKCandidate, tuple[str, ...]]:
    """Try deterministic nearby seeds and select the smallest safe IK move."""
    ik_seeds = make_ik_seed_candidates(
        current_positions,
        attempts=args.ik_attempts,
        perturbation=args.ik_seed_perturbation,
        max_offset=0.5 * args.max_delta,
    )
    candidates = []
    attempt_results = []
    for attempt, seed in enumerate(ik_seeds, start=1):
        ik_request = make_ik_request(
            node,
            target_pose,
            seed,
            current_positions,
            velocities,
            args,
        )
        ik_response = node.compute_ik(
            ik_request,
            timeout=args.result_timeout,
        )
        ik_code = ik_response.error_code.val
        if ik_code != MoveItErrorCodes.SUCCESS:
            attempt_results.append(
                f"attempt {attempt}: {error_name(ik_code)}"
            )
            continue

        try:
            candidate_target = extract_ik_solution(ik_response)
            candidate_safety = validate_ik_target(
                current_positions,
                candidate_target,
                max_joint_delta=args.max_delta,
                max_total_joint_delta=args.max_total_joint_delta,
                max_wrist_joint_delta=args.max_wrist_joint_delta,
            )
        except RuntimeError as exc:
            attempt_results.append(
                f"attempt {attempt}: REJECTED ({exc})"
            )
            continue

        candidates.append(
            IKCandidate(
                attempt=attempt,
                target=candidate_target,
                safety=candidate_safety,
            )
        )
        attempt_results.append(
            f"attempt {attempt}: LOCAL_SUCCESS "
            f"(total={candidate_safety.total_delta:.6f}, "
            f"max={candidate_safety.max_delta:.6f} rad)"
        )

    if not candidates:
        raise RuntimeError(
            "NO_LOCAL_IK_SOLUTION: MoveIt found no collision-free IK "
            "solution inside the measured-state joint neighborhood."
        )
    return select_closest_ik_candidate(candidates), tuple(attempt_results)


def trajectory_joint_excursion(
    trajectory,
    current: Mapping[str, float],
    joint: str,
) -> float:
    names = list(trajectory.joint_trajectory.joint_names)
    if joint not in names:
        raise RuntimeError(f"Planned trajectory is missing {joint}.")
    index = names.index(joint)
    points = trajectory.joint_trajectory.points
    if not points:
        raise RuntimeError("Planned trajectory contains no points.")
    return max(
        abs(point.positions[index] - current[joint]) for point in points
    )


def require_nonempty_trajectory(trajectory) -> None:
    names = list(trajectory.joint_trajectory.joint_names)
    points = list(trajectory.joint_trajectory.points)
    if not names or len(points) < 2:
        raise RuntimeError(
            "Pilz LIN returned an empty or unusable trajectory. "
            "Execution is blocked."
        )


def trajectory_total_joint_excursion(
    trajectory,
    current: Mapping[str, float],
) -> tuple[float, int]:
    names = list(trajectory.joint_trajectory.joint_names)
    if set(names) != set(JOINTS) or len(names) != len(JOINTS):
        raise RuntimeError(
            "Cannot measure total joint excursion: trajectory joint names "
            "do not match the seven FR3 joints."
        )
    points = list(trajectory.joint_trajectory.points)
    if not points:
        raise RuntimeError(
            "Cannot measure total joint excursion for an empty trajectory."
        )
    name_index = {joint: names.index(joint) for joint in JOINTS}
    totals = []
    for point_index, point in enumerate(points):
        if len(point.positions) != len(names):
            raise RuntimeError(
                f"Trajectory point {point_index} has an invalid position "
                "vector."
            )
        totals.append(
            sum(
                abs(point.positions[name_index[joint]] - current[joint])
                for joint in JOINTS
            )
        )
    maximum_index = max(range(len(totals)), key=totals.__getitem__)
    return totals[maximum_index], maximum_index


def validate_pilz_joint_excursions(
    trajectory,
    current: Mapping[str, float],
    *,
    max_wrist_joint_delta: float,
    max_total_joint_delta: float,
) -> tuple[float, float, int]:
    wrist_excursion = trajectory_joint_excursion(
        trajectory,
        current,
        "fr3_joint7",
    )
    total_excursion, total_index = trajectory_total_joint_excursion(
        trajectory,
        current,
    )
    violations = []
    if wrist_excursion > max_wrist_joint_delta + 1e-12:
        violations.append(
            f"fr3_joint7 excursion is {wrist_excursion:.6f} rad "
            f"(limit {max_wrist_joint_delta:.6f})"
        )
    if total_excursion > max_total_joint_delta + 1e-12:
        violations.append(
            f"total joint excursion is {total_excursion:.6f} rad at "
            f"point {total_index} (limit {max_total_joint_delta:.6f})"
        )
    if violations:
        raise RuntimeError(
            "Unsafe Pilz LIN joint path: " + "; ".join(violations)
        )
    return wrist_excursion, total_excursion, total_index


def require_pilz_execution_checks(completed: set[str]) -> None:
    missing = sorted(PILZ_EXECUTION_CHECKS - completed)
    if missing:
        raise RuntimeError(
            "Pilz LIN execution blocked because validation did not complete: "
            + ", ".join(missing)
        )


def validate_preexecution_state(
    planned_start: Mapping[str, float],
    measured_start: Mapping[str, float],
    limit: float,
) -> tuple[str, float]:
    missing = [
        joint
        for joint in JOINTS
        if joint not in planned_start or joint not in measured_start
    ]
    if missing:
        raise RuntimeError(
            f"Pre-execution state check is missing joints: {missing}"
        )
    errors = {
        joint: abs(measured_start[joint] - planned_start[joint])
        for joint in JOINTS
    }
    joint = max(JOINTS, key=errors.get)
    error = errors[joint]
    if error > limit:
        raise RuntimeError(
            f"Robot state changed after IK/planning: {joint} differs by "
            f"{error:.6f} rad (limit {limit:.6f}). Re-run the command to "
            "compute IK from a fresh state."
        )
    return joint, error


def ensure_robot_settled(
    velocities: Mapping[str, float],
    limit: float,
    *,
    phase: str,
) -> None:
    missing = [joint for joint in JOINTS if joint not in velocities]
    if missing:
        raise RuntimeError(
            f"Cannot check whether the robot is settled {phase}; "
            f"missing velocities: {missing}"
        )
    joint = max(JOINTS, key=lambda name: abs(velocities[name]))
    velocity = abs(velocities[joint])
    if velocity > limit:
        raise RuntimeError(
            f"Robot is not settled {phase}: {joint} is moving at "
            f"{velocity:.6f} rad/s (limit {limit:.6f})."
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


def quaternion_angular_distance(
    first: Sequence[float],
    second: Sequence[float],
) -> float:
    if len(first) != 4 or len(second) != 4:
        raise RuntimeError("Cartesian path quaternion must contain 4 values.")
    if any(not math.isfinite(value) for value in (*first, *second)):
        raise RuntimeError("Cartesian path quaternion is not finite.")
    first_norm = math.sqrt(sum(value * value for value in first))
    second_norm = math.sqrt(sum(value * value for value in second))
    if first_norm <= 0.0 or second_norm <= 0.0:
        raise RuntimeError("Cartesian path quaternion has zero length.")
    dot = abs(
        sum(a * b for a, b in zip(first, second))
        / (first_norm * second_norm)
    )
    return 2.0 * math.acos(min(1.0, max(-1.0, dot)))


def validate_planned_cartesian_path(
    *,
    start_position: Sequence[float],
    target_position: Sequence[float],
    target_orientation: Sequence[float],
    poses: Sequence[Pose],
    max_cross_track_error: float,
    max_progress_regression: float,
    max_segment_overrun: float,
    final_target_error_limit: float,
    max_orientation_error: float,
) -> PlannedCartesianPathReport:
    limits = (
        max_cross_track_error,
        max_progress_regression,
        max_segment_overrun,
        final_target_error_limit,
        max_orientation_error,
    )
    if any(not math.isfinite(value) or value <= 0.0 for value in limits):
        raise RuntimeError(
            "Planned Cartesian path limits must be finite and positive."
        )
    if len(start_position) != 3 or len(target_position) != 3:
        raise RuntimeError(
            "Cartesian path start and target must contain 3 values."
        )
    start = tuple(float(value) for value in start_position)
    target = tuple(float(value) for value in target_position)
    if any(not math.isfinite(value) for value in (*start, *target)):
        raise RuntimeError(
            "Cartesian path start or target contains non-finite values."
        )
    if len(poses) < 2:
        raise RuntimeError(
            "Cartesian path FK verification requires at least two poses."
        )

    displacement = tuple(target[i] - start[i] for i in range(3))
    distance = math.sqrt(sum(value * value for value in displacement))
    if distance <= 0.0:
        raise RuntimeError("Cartesian path target equals its start position.")
    direction = tuple(value / distance for value in displacement)

    maximum_cross_track = 0.0
    maximum_cross_track_index = 0
    maximum_vertical_line_error = 0.0
    maximum_vertical_line_error_index = 0
    minimum_z_offset = 0.0
    maximum_regression = 0.0
    maximum_regression_index = 0
    maximum_overrun = 0.0
    maximum_overrun_index = 0
    maximum_orientation = 0.0
    maximum_orientation_index = 0
    final_orientation = 0.0
    previous_progress = 0.0
    final_position = None

    for index, pose in enumerate(poses):
        position = pose_position(pose)
        if any(not math.isfinite(value) for value in position):
            raise RuntimeError(
                f"Cartesian path FK pose {index} is not finite."
            )
        relative = tuple(position[i] - start[i] for i in range(3))
        progress = sum(
            relative[i] * direction[i] for i in range(3)
        )
        closest_segment_progress = min(distance, max(0.0, progress))
        perpendicular = tuple(
            relative[i] - closest_segment_progress * direction[i]
            for i in range(3)
        )
        cross_track = math.sqrt(
            sum(value * value for value in perpendicular)
        )
        if cross_track > maximum_cross_track:
            maximum_cross_track = cross_track
            maximum_cross_track_index = index

        expected_z = start[2] + progress * direction[2]
        vertical_error = abs(position[2] - expected_z)
        if vertical_error > maximum_vertical_line_error:
            maximum_vertical_line_error = vertical_error
            maximum_vertical_line_error_index = index
        minimum_z_offset = min(minimum_z_offset, position[2] - start[2])

        if index > 0:
            regression = max(0.0, previous_progress - progress)
            if regression > maximum_regression:
                maximum_regression = regression
                maximum_regression_index = index
        previous_progress = progress

        overrun = max(0.0, -progress, progress - distance)
        if overrun > maximum_overrun:
            maximum_overrun = overrun
            maximum_overrun_index = index

        orientation_error = quaternion_angular_distance(
            pose_orientation(pose),
            target_orientation,
        )
        if orientation_error > maximum_orientation:
            maximum_orientation = orientation_error
            maximum_orientation_index = index
        final_orientation = orientation_error
        final_position = position

    final_error = math.sqrt(
        sum(
            (final_position[i] - target[i]) ** 2
            for i in range(3)
        )
    )
    violations = []
    if maximum_cross_track > max_cross_track_error + 1e-12:
        violations.append(
            f"TCP cross-track error is {maximum_cross_track:.6f} m at "
            f"point {maximum_cross_track_index} "
            f"(limit {max_cross_track_error:.6f})"
        )
    if maximum_regression > max_progress_regression + 1e-12:
        violations.append(
            f"TCP moves backward by {maximum_regression:.6f} m at point "
            f"{maximum_regression_index} "
            f"(limit {max_progress_regression:.6f})"
        )
    if maximum_overrun > max_segment_overrun + 1e-12:
        violations.append(
            f"TCP leaves the commanded line segment by "
            f"{maximum_overrun:.6f} m at point "
            f"{maximum_overrun_index} "
            f"(limit {max_segment_overrun:.6f})"
        )
    if final_error > final_target_error_limit + 1e-12:
        violations.append(
            f"planned TCP endpoint error is {final_error:.6f} m "
            f"(limit {final_target_error_limit:.6f})"
        )
    if maximum_orientation > max_orientation_error + 1e-12:
        violations.append(
            f"TCP orientation changes by {maximum_orientation:.6f} rad at "
            f"point {maximum_orientation_index} "
            f"(limit {max_orientation_error:.6f})"
        )
    if violations:
        raise RuntimeError(
            "Unsafe planned Cartesian path: " + "; ".join(violations)
        )

    return PlannedCartesianPathReport(
        point_count=len(poses),
        requested_distance=distance,
        max_cross_track_error=maximum_cross_track,
        max_cross_track_index=maximum_cross_track_index,
        max_vertical_line_error=maximum_vertical_line_error,
        max_vertical_line_error_index=maximum_vertical_line_error_index,
        minimum_z_offset=minimum_z_offset,
        max_progress_regression=maximum_regression,
        max_progress_regression_index=maximum_regression_index,
        max_segment_overrun=maximum_overrun,
        max_segment_overrun_index=maximum_overrun_index,
        final_target_error=final_error,
        max_orientation_error=maximum_orientation,
        max_orientation_error_index=maximum_orientation_index,
        final_orientation_error=final_orientation,
    )


def format_ik_safety_report(report: IKSafetyReport) -> str:
    return "\n".join(
        [
            "IK target safety check: PASS",
            f"  Total absolute joint change: {report.total_delta:.6f} rad",
            f"  Maximum joint change:        {report.max_delta:.6f} rad "
            f"({report.max_delta_joint})",
            f"  Joint 7 change:              {report.wrist_delta:.6f} rad",
        ]
    )


def format_planned_cartesian_path_report(
    report: PlannedCartesianPathReport,
) -> str:
    return "\n".join(
        [
            "Planned TCP path FK check: PASS",
            f"  FK samples:                 {report.point_count}",
            f"  Requested distance:         "
            f"{report.requested_distance:.6f} m",
            f"  Maximum TCP line deviation: "
            f"{report.max_cross_track_error:.6f} m "
            f"(point {report.max_cross_track_index})",
            f"  Maximum vertical line error:"
            f"  {report.max_vertical_line_error:.6f} m "
            f"(point {report.max_vertical_line_error_index})",
            f"  Lowest Z offset from start:  "
            f"{report.minimum_z_offset:+.6f} m",
            f"  Maximum backward step:       "
            f"{report.max_progress_regression:.6f} m",
            f"  Maximum segment overrun:     "
            f"{report.max_segment_overrun:.6f} m",
            f"  Final Cartesian target error: "
            f"{report.final_target_error:.6f} m",
            f"  Final orientation error:     "
            f"{report.final_orientation_error:.6f} rad",
            f"  Maximum orientation error:   "
            f"{report.max_orientation_error:.6f} rad",
        ]
    )


def format_joint_trajectory_safety_report(report) -> str:
    return "\n".join(
        [
            "Joint-space trajectory safety check: PASS",
            f"  Points:                    {report.point_count}",
            f"  Duration after scaling:    {report.duration:.3f} s",
            f"  Timing slowdown factor:    {report.timing_scale:.3f}x",
            f"  Maximum start error:       {report.max_start_error:.6f} rad "
            f"({report.max_start_error_joint})",
            f"  Maximum joint excursion:   {report.max_joint_excursion:.6f} rad "
            f"({report.max_joint_excursion_joint})",
            f"  Maximum adjacent step:     {report.max_joint_step:.6f} rad "
            f"({report.max_joint_step_joint})",
            f"  Maximum planned velocity:  {report.max_planned_velocity:.6f} rad/s "
            f"({report.max_planned_velocity_joint})",
            f"  Maximum planned accel:     {report.max_planned_acceleration:.6f} "
            f"rad/s^2 ({report.max_planned_acceleration_joint})",
        ]
    )


def validate_command_args(args: argparse.Namespace) -> None:
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
        "revolute_jump_threshold",
        "max_planned_cross_track_error",
        "max_planned_progress_regression",
        "max_planned_segment_overrun",
        "planned_cartesian_goal_tolerance",
        "max_planned_orientation_error",
        "pilz_goal_orientation_tolerance",
        "fk_joint_sample_step",
        "ik_timeout",
        "ik_seed_perturbation",
        "ik_joint7_tolerance",
        "max_delta",
        "max_total_joint_delta",
        "max_wrist_joint_delta",
        "planning_time",
        "workspace_size",
        "velocity_scaling",
        "acceleration_scaling",
        "goal_tolerance",
        "max_start_velocity",
        "max_start_error",
        "max_joint_excursion",
        "max_joint_step",
        "max_planned_velocity",
        "max_planned_acceleration",
        "cartesian_verification_tolerance",
        "orientation_verification_tolerance",
        "joint_verification_tolerance",
        "initial_dip_window",
        "server_timeout",
        "fk_timeout",
        "tf_timeout",
        "result_timeout",
    )
    for name in positive:
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0.0:
            raise RuntimeError(
                f"--{name.replace('_', '-')} must be finite and positive."
            )
    if args.ik_attempts <= 0:
        raise RuntimeError("--ik-attempts must be positive.")
    if args.planning_attempts <= 0:
        raise RuntimeError("--planning-attempts must be positive.")
    if args.max_fk_samples < 2:
        raise RuntimeError("--max-fk-samples must be at least 2.")
    if args.ik_seed_perturbation > args.max_delta:
        raise RuntimeError(
            "--ik-seed-perturbation cannot exceed --max-joint-delta."
        )
    if args.ik_joint7_tolerance > args.max_wrist_joint_delta:
        raise RuntimeError(
            "--ik-joint7-tolerance cannot exceed "
            "--max-wrist-joint-delta."
        )
    if args.velocity_scaling > 1.0:
        raise RuntimeError("--velocity-scaling must not exceed 1.0.")
    if args.acceleration_scaling > 1.0:
        raise RuntimeError("--acceleration-scaling must not exceed 1.0.")
    if not math.isfinite(args.jump_threshold) or args.jump_threshold < 0.0:
        raise RuntimeError(
            "--jump-threshold must be finite and non-negative."
        )
    validate_post_execution_verification_args(args)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Plan or execute a small guarded Cartesian translation through "
            "MoveIt. Straight Cartesian interpolation is the default."
        )
    )
    parser.add_argument("--state-topic", default="/joint_states")
    parser.add_argument("--ik-service", default="/compute_ik")
    parser.add_argument("--plan-service", default="/plan_kinematic_path")
    parser.add_argument(
        "--cartesian-service",
        default="/compute_cartesian_path",
    )
    parser.add_argument("--fk-service", default="/compute_fk")
    parser.add_argument(
        "--state-validity-service",
        default="/check_state_validity",
    )
    parser.add_argument("--execute-action", default="/execute_trajectory")
    parser.add_argument("--controller-name", default="fr3_arm_controller")
    parser.add_argument(
        "--controller-state-topic",
        default="/fr3_arm_controller/controller_state",
    )
    parser.add_argument(
        "--controller-manager-service",
        default="/controller_manager/list_controllers",
    )
    parser.add_argument(
        "--controller-parameter-service",
        default="/fr3_arm_controller/get_parameters",
    )
    parser.add_argument(
        "--robot-description-parameter-service",
        default="/robot_state_publisher/get_parameters",
    )
    parser.add_argument(
        "--robot-state-topic",
        default="/franka_robot_state_broadcaster/robot_state",
    )
    parser.add_argument(
        "--diagnostic-output",
        type=Path,
        help=(
            "CSV path for desired, actual, error, effort, and local-FK "
            "execution samples. By default a timestamped file is created "
            "under debug/ only when --execute is used."
        ),
    )
    parser.add_argument(
        "--initial-dip-window",
        type=float,
        default=1.0,
        help=(
            "Seconds after execution start used only to correlate the "
            "initial measured TCP Z motion with joint tracking errors."
        ),
    )
    parser.add_argument(
        "--path-mode",
        choices=(
            "cartesian",
            "pilz-lin",
            "joint-space",
        ),
        default="cartesian",
        help=(
            "Use guarded straight Cartesian interpolation by default. "
            "Use pilz-lin for MoveIt's industrial linear planner. "
            "The endpoint-only joint-space planner is diagnostic."
        ),
    )
    parser.add_argument("--group", default="fr3_arm")
    parser.add_argument("--planning-frame", default="base")
    parser.add_argument("--ee-link", default="fr3_hand_tcp")
    parser.add_argument("--pipeline", default="")
    parser.add_argument("--planner", default="RRTConnectkConfigDefault")
    parser.add_argument("--planning-attempts", type=int, default=5)
    parser.add_argument("--planning-time", type=float, default=5.0)
    parser.add_argument("--workspace-size", type=float, default=2.0)
    parser.add_argument("--velocity-scaling", type=float, default=0.03)
    parser.add_argument("--acceleration-scaling", type=float, default=0.03)
    parser.add_argument("--goal-tolerance", type=float, default=0.002)

    parser.add_argument("--dx", type=float, default=0.0)
    parser.add_argument("--dy", type=float, default=0.0)
    parser.add_argument("--dz", type=float, default=0.0)
    parser.add_argument("--max-distance", type=float, default=0.01)
    parser.add_argument(
        "--eef-step",
        type=float,
        default=0.00025,
        help="Maximum Cartesian interpolation step, in meters.",
    )
    parser.add_argument(
        "--jump-threshold",
        type=float,
        default=0.0,
        help=(
            "MoveIt relative joint-jump threshold. Disabled by default for "
            "short micro-motions; absolute jump and trajectory checks remain."
        ),
    )
    parser.add_argument(
        "--revolute-jump-threshold",
        type=float,
        default=0.10,
        help="MoveIt absolute revolute-joint jump threshold, in radians.",
    )
    parser.add_argument(
        "--max-planned-cross-track-error",
        type=float,
        default=0.001,
        help=(
            "Maximum FK distance from the requested Cartesian line before "
            "execution is blocked."
        ),
    )
    parser.add_argument(
        "--max-planned-progress-regression",
        type=float,
        default=0.0005,
        help="Maximum allowed backward TCP step along the commanded line.",
    )
    parser.add_argument(
        "--max-planned-segment-overrun",
        type=float,
        default=0.0005,
        help="Maximum planned TCP travel beyond either command endpoint.",
    )
    parser.add_argument(
        "--planned-cartesian-goal-tolerance",
        type=float,
        default=0.005,
        help="Maximum FK error at the planned Cartesian endpoint.",
    )
    parser.add_argument(
        "--max-planned-orientation-error",
        type=float,
        default=0.03,
        help="Maximum TCP orientation change along the planned path.",
    )
    parser.add_argument(
        "--pilz-goal-orientation-tolerance",
        type=float,
        default=0.001,
        help="Pilz LIN Cartesian goal orientation tolerance, in radians.",
    )
    parser.add_argument(
        "--fk-joint-sample-step",
        type=float,
        default=0.002,
        help=(
            "Maximum joint interpolation step between independent MoveIt "
            "FK path samples, in radians."
        ),
    )
    parser.add_argument(
        "--max-fk-samples",
        type=int,
        default=500,
        help="Maximum independent FK samples allowed for one plan.",
    )
    parser.add_argument("--ik-timeout", type=float, default=1.0)
    parser.add_argument(
        "--ik-attempts",
        type=int,
        default=5,
        help="Number of deterministic nearby IK seeds to evaluate.",
    )
    parser.add_argument(
        "--ik-seed-perturbation",
        type=float,
        default=0.01,
        help="Joint offset used to construct nearby IK seeds, in radians.",
    )
    parser.add_argument(
        "--ik-joint7-tolerance",
        type=float,
        default=0.001,
        help=(
            "Keep fr3_joint7 near its measured seed while solving "
            "translation-only IK."
        ),
    )
    parser.add_argument(
        "--max-joint-delta",
        dest="max_delta",
        type=float,
        default=0.12,
        help="Maximum IK target change for any one joint, in radians.",
    )
    parser.add_argument(
        "--max-total-joint-delta",
        type=float,
        default=0.30,
        help="Maximum sum of the seven absolute IK joint changes.",
    )
    parser.add_argument(
        "--max-wrist-joint-delta",
        type=float,
        default=0.002,
        help=(
            "Maximum allowed fr3_joint7 target and path excursion. The "
            "translation-only IK request separately locks this joint near "
            "its measured seed."
        ),
    )

    parser.add_argument("--max-start-velocity", type=float, default=0.01)
    parser.add_argument("--max-start-error", type=float, default=0.003)
    parser.add_argument("--max-joint-excursion", type=float, default=0.12)
    parser.add_argument("--max-joint-step", type=float, default=0.04)
    parser.add_argument(
        "--max-planned-velocity",
        type=float,
        default=0.02,
        help="Absolute trajectory velocity limit, in rad/s.",
    )
    parser.add_argument(
        "--max-planned-acceleration",
        type=float,
        default=0.05,
        help="Absolute trajectory acceleration limit, in rad/s^2.",
    )

    parser.add_argument(
        "--cartesian-verification-tolerance",
        type=float,
        default=0.003,
        help="Maximum measured TCP target error, in meters.",
    )
    parser.add_argument(
        "--orientation-verification-tolerance",
        type=float,
        default=0.03,
    )
    parser.add_argument(
        "--joint-verification-tolerance",
        type=float,
        default=0.01,
        help=(
            "Maximum measured error from the planned IK joint target. "
            "Cartesian position and orientation are verified separately."
        ),
    )
    parser.add_argument("--server-timeout", type=float, default=10.0)
    parser.add_argument("--fk-timeout", type=float, default=5.0)
    parser.add_argument("--tf-timeout", type=float, default=5.0)
    parser.add_argument("--result-timeout", type=float, default=60.0)
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Execute the plan. Without this flag the robot cannot move.",
    )
    add_post_execution_verification_arguments(parser)
    parser.set_defaults(verification_tolerance=0.01)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        validate_command_args(args)
    except RuntimeError as exc:
        print(f"Input rejected: {exc}", file=sys.stderr)
        return 2

    rclpy.init()
    node = CartesianIKJointClient(
        args.state_topic,
        args.ik_service,
        args.plan_service,
        args.cartesian_service,
        args.fk_service,
        args.state_validity_service,
        args.execute_action,
        args.controller_name,
        args.controller_state_topic,
        args.controller_manager_service,
        args.controller_parameter_service,
        args.robot_description_parameter_service,
        args.robot_state_topic,
    )

    try:
        pilz_checks: set[str] = set()
        start_snapshot = node.wait_for_joint_state_snapshot(
            timeout=5.0,
            require_velocities=True,
        )
        ensure_robot_settled(
            start_snapshot.velocities,
            args.max_start_velocity,
            phase="before planning",
        )
        start_transform = node.wait_for_transform(
            args.planning_frame,
            args.ee_link,
            args.tf_timeout,
        )
        target_pose = make_target_pose(
            start_transform,
            args.dx,
            args.dy,
            args.dz,
        )

        common_description = (
            f"Relative Cartesian endpoint in frame '{args.planning_frame}'\n"
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
        )
        if args.path_mode == "cartesian":
            print(
                common_description
                + "Collision checking: ENABLED\n"
                + f"Waypoint joints 1-6 constraints: CURRENT +/-"
                + f"{args.max_delta:.6f} rad\n"
                + f"Waypoint joint 7 constraint: CURRENT +/-"
                + f"{args.max_wrist_joint_delta:.6f} rad\n"
                + "MoveIt path type: STRAIGHT CARTESIAN INTERPOLATION\n"
                + f"Cartesian interpolation step: {args.eef_step:.6f} m\n"
                + "Planned TCP FK validation: ENABLED\n"
                + f"Mode: {'EXECUTE' if args.execute else 'PLAN ONLY'}\n"
            )

            node.wait_for_cartesian_service(timeout=args.server_timeout)
            cartesian_request = make_cartesian_request(
                node,
                target_pose,
                start_snapshot.positions,
                start_snapshot.velocities,
                args,
            )
            cartesian_response = node.cartesian_path(
                cartesian_request,
                timeout=args.result_timeout,
            )
            planning_code = cartesian_response.error_code.val
            print(
                f"Cartesian path fraction: "
                f"{cartesian_response.fraction:.6f}"
            )
            print(f"Planning result: {error_name(planning_code)}")
            if planning_code != MoveItErrorCodes.SUCCESS:
                return 1
            if cartesian_response.fraction < 0.999:
                raise RuntimeError(
                    "MoveIt returned an incomplete Cartesian path; "
                    f"fraction={cartesian_response.fraction:.6f}. "
                    "Execution is blocked."
                )
            trajectory = cartesian_response.solution
            print("\n" + trajectory_summary(trajectory))
            target = trajectory_final_positions(trajectory)
            ik_safety = validate_ik_target(
                start_snapshot.positions,
                target,
                max_joint_delta=args.max_delta,
                max_total_joint_delta=args.max_total_joint_delta,
                max_wrist_joint_delta=args.max_wrist_joint_delta,
            )
            print("\n" + format_joint_table(start_snapshot.positions, target))
            print("\n" + format_ik_safety_report(ik_safety))
        elif args.path_mode == "pilz-lin":
            print(
                common_description
                + "Collision checking: ENABLED\n"
                + "MoveIt path type: PILZ INDUSTRIAL LIN\n"
                + "MoveIt pipeline: pilz_industrial_motion_planner\n"
                + "MoveIt planner: LIN\n"
                + "Planned TCP FK validation: ENABLED\n"
                + f"Mode: {'EXECUTE' if args.execute else 'PLAN ONLY'}\n"
            )

            node.wait_for_plan_service(timeout=args.server_timeout)
            plan_request = make_pilz_lin_plan_request(
                node,
                target_pose,
                start_snapshot.positions,
                args,
            )
            plan_response = node.plan(
                plan_request,
                timeout=args.result_timeout,
            )
            plan_code = plan_response.error_code.val
            print(
                f"Planning result: {error_name(plan_code)} "
                f"(code={plan_code})"
            )
            if plan_code != MoveItErrorCodes.SUCCESS:
                raise RuntimeError(
                    f"Pilz LIN planning failed with MoveIt error "
                    f"{error_name(plan_code)} (code={plan_code}). "
                    "No fallback planner was used."
                )
            trajectory = plan_response.trajectory
            require_nonempty_trajectory(trajectory)
            pilz_checks.add("nonempty_trajectory")
            print("\n" + trajectory_summary(trajectory))
            target = trajectory_final_positions(trajectory)
            print("\n" + format_joint_table(start_snapshot.positions, target))
        else:
            print(
                common_description
                + "IK collision checking: ENABLED\n"
                + f"IK joints 1-6 constraints: CURRENT +/-"
                + f"{args.max_delta:.6f} rad\n"
                + f"IK joint 7 constraint: CURRENT +/-"
                + f"{args.ik_joint7_tolerance:.6f} rad\n"
                + f"IK local seed attempts: {args.ik_attempts}\n"
                + "MoveIt path type: JOINT SPACE "
                + "(not a straight Cartesian path)\n"
                + f"MoveIt planner: {args.planner}\n"
                + f"Mode: {'EXECUTE' if args.execute else 'PLAN ONLY'}\n"
            )

            node.wait_for_ik_service(timeout=args.server_timeout)
            selected_ik, attempt_results = find_local_ik_candidate(
                node,
                target_pose,
                start_snapshot.positions,
                start_snapshot.velocities,
                args,
            )
            print("\nLocal IK search:")
            for result in attempt_results:
                print(f"  {result}")
            target = selected_ik.target
            ik_safety = selected_ik.safety
            print(
                f"IK result: SUCCESS (selected attempt {selected_ik.attempt} "
                f"of {args.ik_attempts})"
            )
            print("\n" + format_joint_table(start_snapshot.positions, target))
            print("\n" + format_ik_safety_report(ik_safety))

            node.wait_for_plan_service(timeout=args.server_timeout)
            plan_request = make_plan_request(
                node,
                start_snapshot.positions,
                target,
                args,
            )
            add_wrist_path_constraint(
                plan_request,
                start_snapshot.positions,
                args.max_wrist_joint_delta,
            )
            plan_response = node.plan(
                plan_request,
                timeout=args.result_timeout,
            )
            plan_code = plan_response.error_code.val
            print("\n" + trajectory_summary(plan_response.trajectory))
            print(f"Planning result: {error_name(plan_code)}")
            if plan_code != MoveItErrorCodes.SUCCESS:
                return 1
            trajectory = plan_response.trajectory

        try:
            timing_scale = scale_timed_trajectory(
                trajectory,
                max_velocity=args.max_planned_velocity,
                max_acceleration=args.max_planned_acceleration,
            )
            trajectory_safety = validate_trajectory(
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

        if args.path_mode == "pilz-lin":
            pilz_checks.add("joint_trajectory")
        else:
            planned_final = trajectory_final_positions(trajectory)
            final_errors = {
                joint: abs(planned_final[joint] - target[joint])
                for joint in JOINTS
            }
            final_error_joint = max(JOINTS, key=final_errors.get)
            if final_errors[final_error_joint] > args.goal_tolerance:
                raise RuntimeError(
                    f"Planned endpoint differs from the IK target on "
                    f"{final_error_joint} by "
                    f"{final_errors[final_error_joint]:.6f} rad "
                    f"(limit {args.goal_tolerance:.6f})."
                )

        if args.path_mode == "pilz-lin":
            (
                wrist_path_excursion,
                total_excursion,
                total_excursion_index,
            ) = validate_pilz_joint_excursions(
                trajectory,
                start_snapshot.positions,
                max_wrist_joint_delta=args.max_wrist_joint_delta,
                max_total_joint_delta=args.max_total_joint_delta,
            )
            pilz_checks.add("joint_excursions")
        else:
            wrist_path_excursion = trajectory_joint_excursion(
                trajectory,
                start_snapshot.positions,
                "fr3_joint7",
            )
            if wrist_path_excursion > args.max_wrist_joint_delta:
                raise RuntimeError(
                    f"Planned trajectory moves fr3_joint7 by "
                    f"{wrist_path_excursion:.6f} rad along the path "
                    f"(limit {args.max_wrist_joint_delta:.6f})."
                )

        print("\n" + format_joint_trajectory_safety_report(trajectory_safety))
        print(
            f"  Maximum joint 7 excursion: {wrist_path_excursion:.6f} rad"
        )
        if args.path_mode == "pilz-lin":
            print(
                f"  Maximum total joint excursion: "
                f"{total_excursion:.6f} rad "
                f"(point {total_excursion_index})"
            )

        if args.path_mode in (
            "cartesian",
            "pilz-lin",
        ):
            sampled_positions = sample_trajectory_joint_positions(
                trajectory,
                max_joint_step=args.fk_joint_sample_step,
                max_samples=args.max_fk_samples,
            )
            if args.path_mode == "pilz-lin":
                node.wait_for_state_validity_service(
                    timeout=args.server_timeout
                )
                collision_sample_count = validate_collision_free_samples(
                    node,
                    sampled_positions,
                    args,
                )
                print(
                    "\nDense collision-state check: PASS "
                    f"({collision_sample_count} samples)"
                )
                pilz_checks.add("collision_free")
            node.wait_for_fk_service(timeout=args.server_timeout)
            fk_poses = trajectory_fk_poses(
                node,
                sampled_positions,
                args,
            )
            planned_cartesian_report = validate_planned_cartesian_path(
                start_position=transform_position(start_transform),
                target_position=pose_position(target_pose),
                target_orientation=pose_orientation(target_pose),
                poses=fk_poses,
                max_cross_track_error=(
                    args.max_planned_cross_track_error
                ),
                max_progress_regression=(
                    args.max_planned_progress_regression
                ),
                max_segment_overrun=args.max_planned_segment_overrun,
                final_target_error_limit=(
                    args.planned_cartesian_goal_tolerance
                ),
                max_orientation_error=(
                    args.max_planned_orientation_error
                ),
            )
            print(
                "\n"
                + format_planned_cartesian_path_report(
                    planned_cartesian_report
                )
            )
            if args.path_mode == "pilz-lin":
                pilz_checks.add("cartesian_path")

        if args.path_mode == "pilz-lin":
            require_pilz_execution_checks(pilz_checks)

        if not args.execute:
            if args.path_mode == "cartesian":
                print(
                    "\nPlan-only run complete. MoveIt FK verified the "
                    "planned straight TCP path. Inspect it in RViz and add "
                    "--execute only after it looks correct."
                )
            elif args.path_mode == "pilz-lin":
                print(
                    "\nPlan-only run complete. MoveIt FK verified the "
                    "Pilz LIN trajectory against the requested TCP line. "
                    "Inspect it in RViz and add --execute only after it "
                    "looks correct."
                )
            else:
                print(
                    "\nPlan-only run complete. Inspect the joint-space path "
                    "in RViz. This diagnostic path is not guaranteed to keep "
                    "the TCP on a straight line."
                )
            return 0

        controller_configuration = node.inspect_active_controller(
            timeout=args.server_timeout
        )
        print(
            "\n"
            + format_controller_configuration(controller_configuration)
        )
        inertia_configuration = node.read_inertia_configuration(
            timeout=args.server_timeout
        )
        print(
            "\n"
            + format_franka_inertia_configuration(
                inertia_configuration
            )
        )
        robot_description = node.get_robot_description(
            timeout=args.server_timeout
        )
        local_fk = LocalForwardKinematics(robot_description, args.ee_link)

        node.start_controller_state_recording()
        controller_samples = ()
        try:
            node.wait_for_controller_state_sample(
                timeout=args.server_timeout
            )
            pre_execute = node.wait_for_joint_state_snapshot(
                timeout=args.verification_timeout,
                after_sequence=start_snapshot.sequence,
                require_velocities=True,
            )
            velocity_joint, maximum_start_velocity = maximum_joint_velocity(
                pre_execute.velocities
            )
            ensure_robot_settled(
                pre_execute.velocities,
                args.max_start_velocity,
                phase="immediately before execution",
            )
            changed_joint, changed_error = validate_preexecution_state(
                start_snapshot.positions,
                pre_execute.positions,
                args.max_start_error,
            )
            try:
                preexecution_trajectory_safety = validate_trajectory(
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
                    f"Pre-execution trajectory check failed: {exc}"
                ) from exc
            print(
                "\nPre-execution measured-state checks: PASS\n"
                f"  Maximum start-state position error: "
                f"{preexecution_trajectory_safety.max_start_error:.6f} rad "
                f"on {preexecution_trajectory_safety.max_start_error_joint}\n"
                f"  Measured drift since planning began: "
                f"{changed_error:.6f} rad on {changed_joint}\n"
                f"  Maximum measured joint velocity:   "
                f"{maximum_start_velocity:.6f} rad/s on {velocity_joint}"
            )

            start_tcp_from_measured_fk = local_fk.position(
                pre_execute.positions
            )
            node.wait_for_execute_action(timeout=args.server_timeout)
            execution_start_time = time.monotonic()
            execution_error = None
            try:
                execute_result = node.execute(
                    trajectory,
                    timeout=args.result_timeout,
                )
                execute_code = execute_result.error_code.val
                execution_succeeded = (
                    execute_code == MoveItErrorCodes.SUCCESS
                )
                print(f"Execution result: {error_name(execute_code)}")
            except RuntimeError as exc:
                execution_error = str(exc)
                execution_succeeded = False
                print(f"Execution result: ERROR ({execution_error})")
            if not execution_succeeded:
                print(
                    "The controller did not report success. The following "
                    "measurement is diagnostic only."
                )

            print(
                f"\nWaiting {args.settle_time:.3f} s for the robot to "
                "settle..."
            )
            node.spin_for(args.settle_time)
            sequence_after_settle = node.joint_state_sequence
            final_snapshot = node.wait_for_joint_state_snapshot(
                timeout=args.verification_timeout,
                after_sequence=sequence_after_settle,
                require_velocities=True,
            )
            final_transform = node.wait_for_transform(
                args.planning_frame,
                args.ee_link,
                args.tf_timeout,
            )
        finally:
            controller_samples = node.stop_controller_state_recording()

        tracking_samples = build_cartesian_tracking_samples(
            controller_samples,
            execution_start_time=execution_start_time,
            fk=local_fk,
        )
        tracking_report = analyze_execution_tracking(
            tracking_samples,
            start_tcp=start_tcp_from_measured_fk,
            command_duration=trajectory_safety.duration,
            initial_window_duration=args.initial_dip_window,
            joint7_goal_tolerance=(
                controller_configuration.goal_tolerances["fr3_joint7"]
                or args.joint_verification_tolerance
            ),
        )
        diagnostic_output = args.diagnostic_output
        if diagnostic_output is None:
            diagnostic_output = Path("debug") / (
                "cartesian_execution_trace_"
                + time.strftime("%Y%m%d_%H%M%S")
                + ".csv"
            )
        write_execution_trace_csv(diagnostic_output, tracking_samples)
        print("\n" + format_execution_tracking_report(tracking_report))
        print(f"  Full execution trace:                  {diagnostic_output}")

        joint_verification = verify_joint_motion(
            JOINTS,
            start_snapshot.positions,
            target,
            final_snapshot.positions,
            final_snapshot.velocities,
            target_error_limit=args.verification_tolerance,
            joint_target_error_limits={
                "fr3_joint7": args.joint7_verification_tolerance,
            },
            non_commanded_drift_limit=args.non_commanded_drift_limit,
            max_settled_velocity=args.max_settled_velocity,
            minimum_measured_delta=args.minimum_measured_delta,
            command_delta_threshold=args.minimum_measured_delta,
        )
        cartesian_verification = verify_cartesian_motion(
            start_position=transform_position(start_transform),
            target_position=pose_position(target_pose),
            final_position=transform_position(final_transform),
            target_orientation=pose_orientation(target_pose),
            final_orientation=transform_orientation(final_transform),
            expected_joint_positions=target,
            measured_joint_positions=final_snapshot.positions,
            measured_joint_velocities=final_snapshot.velocities,
            target_error_limit=args.cartesian_verification_tolerance,
            orientation_error_limit=args.orientation_verification_tolerance,
            joint_target_error_limit=args.joint_verification_tolerance,
            max_settled_velocity=args.max_settled_velocity,
            minimum_measured_motion=args.minimum_measured_delta,
        )

        print(
            "\n"
            + format_verification_report(
                joint_verification,
                target_error_limit=args.verification_tolerance,
                joint_target_error_limits={
                    "fr3_joint7": args.joint7_verification_tolerance,
                },
                non_commanded_drift_limit=args.non_commanded_drift_limit,
                max_settled_velocity=args.max_settled_velocity,
                require_joint_motion=False,
            )
        )
        print(
            "\n"
            + format_cartesian_verification_report(
                cartesian_verification,
                target_error_limit=args.cartesian_verification_tolerance,
                orientation_error_limit=(
                    args.orientation_verification_tolerance
                ),
                joint_target_error_limit=args.joint_verification_tolerance,
                max_settled_velocity=args.max_settled_velocity,
                execution_succeeded=execution_succeeded,
            )
        )

        passed = (
            execution_succeeded
            and joint_verification.target_state_passed
            and cartesian_verification.passed
        )
        success_labels = {
            "cartesian": "VERIFIED STRAIGHT CARTESIAN SUCCESS",
            "pilz-lin": "VERIFIED PILZ LIN CARTESIAN SUCCESS",
            "joint-space": "VERIFIED IK JOINT-SPACE CARTESIAN SUCCESS",
        }
        result_label = (
            success_labels[args.path_mode]
            if passed
            else "FAILED - DO NOT SEND ANOTHER COMMAND"
        )
        print(f"\nOverall result: {result_label}")
        return 0 if passed else 1

    except KeyboardInterrupt:
        print("Interrupted.")
        return 130
    except (RuntimeError, ValueError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
