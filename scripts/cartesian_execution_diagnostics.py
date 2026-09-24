#!/usr/bin/env python3
"""Read-only diagnostics for one Cartesian trajectory execution."""

import csv
from dataclasses import dataclass
import math
from pathlib import Path
from typing import Mapping, Sequence


JOINTS = tuple(f"fr3_joint{index}" for index in range(1, 8))


@dataclass(frozen=True)
class PIDGains:
    p: float | None
    i: float | None
    d: float | None
    i_clamp: float | None
    ff_velocity_scale: float | None


@dataclass(frozen=True)
class ControllerRuntimeConfiguration:
    name: str
    state: str
    controller_type: str
    configured_command_interfaces: tuple[str, ...]
    claimed_interfaces: tuple[str, ...]
    effort_interface_confirmed: bool
    gains: Mapping[str, PIDGains]
    goal_time_tolerance: float | None
    stopped_velocity_tolerance: float | None
    goal_tolerances: Mapping[str, float | None]
    trajectory_tolerances: Mapping[str, float | None]


@dataclass(frozen=True)
class InertiaProperties:
    frame_id: str
    mass: float
    center_of_mass: tuple[float, float, float]
    inertia: tuple[tuple[float, float, float], ...]


@dataclass(frozen=True)
class FrankaInertiaConfiguration:
    end_effector: InertiaProperties
    load: InertiaProperties
    total: InertiaProperties


@dataclass(frozen=True)
class ControllerStateSample:
    receipt_time: float
    message_time: float
    desired_positions: Mapping[str, float]
    actual_positions: Mapping[str, float]
    position_errors: Mapping[str, float]
    desired_velocities: Mapping[str, float]
    actual_velocities: Mapping[str, float]
    velocity_errors: Mapping[str, float]
    output_efforts: Mapping[str, float]


@dataclass(frozen=True)
class CartesianTrackingSample:
    time_from_execute: float
    controller: ControllerStateSample
    desired_tcp: tuple[float, float, float]
    actual_tcp: tuple[float, float, float]
    cartesian_error: float


@dataclass(frozen=True)
class Joint7TrackingDiagnostics:
    command_duration: float
    desired_motion: float
    actual_motion: float
    final_position_error: float
    maximum_position_error: float
    maximum_position_error_time: float
    desired_motion_onset_time: float | None
    actual_motion_onset_time: float | None
    onset_delay: float | None
    lag_fraction: float
    maximum_desired_step: float
    maximum_desired_step_time: float
    maximum_desired_step_residual: float
    maximum_desired_step_residual_time: float
    desired_step_residual_threshold: float
    desired_trajectory_discontinuous: bool
    initially_sticks: bool
    continuously_lags: bool
    reached_goal_tolerance: bool
    reached_then_rebounded: bool


@dataclass(frozen=True)
class ExecutionTrackingReport:
    sample_count: int
    minimum_actual_z_offset: float
    minimum_actual_z_time: float
    desired_z_offset_at_minimum: float
    maximum_cartesian_error: float
    maximum_cartesian_error_time: float
    first_increasing_error_joint: str | None
    first_increasing_error_time: float | None
    maximum_joint_errors: Mapping[str, float]
    maximum_velocity_errors: Mapping[str, float]
    initial_window_duration: float
    initial_minimum_actual_z_offset: float
    initial_minimum_actual_z_time: float
    initial_desired_z_offset_at_minimum: float
    initial_joint_errors_at_minimum: Mapping[str, float]
    initial_z_error_correlations: Mapping[str, float | None]
    joint7: Joint7TrackingDiagnostics
    final_desired_offset: tuple[float, float, float]
    final_actual_offset: tuple[float, float, float]


OFFICIAL_FRANKA_HUMBLE_GAINS = {
    "fr3_joint1": PIDGains(600.0, 0.0, 30.0, 1.0, 0.0),
    "fr3_joint2": PIDGains(600.0, 0.0, 30.0, 1.0, 0.0),
    "fr3_joint3": PIDGains(600.0, 0.0, 30.0, 1.0, 0.0),
    "fr3_joint4": PIDGains(600.0, 0.0, 30.0, 1.0, 0.0),
    "fr3_joint5": PIDGains(250.0, 0.0, 10.0, 1.0, 0.0),
    "fr3_joint6": PIDGains(150.0, 0.0, 10.0, 1.0, 0.0),
    "fr3_joint7": PIDGains(50.0, 0.0, 5.0, 1.0, 0.0),
}
OFFICIAL_FRANKA_HUMBLE_COMMAND_INTERFACES = ("effort",)
OFFICIAL_HUMBLE_GOAL_TIME_TOLERANCE = 0.0
OFFICIAL_HUMBLE_STOPPED_VELOCITY_TOLERANCE = 0.01
OFFICIAL_HUMBLE_JOINT_TOLERANCE = 0.0


class LocalForwardKinematics:
    """Compute TCP positions from the exact robot_description used by ROS."""

    def __init__(self, robot_description: str, ee_link: str) -> None:
        if not robot_description.strip():
            raise RuntimeError("robot_description is empty.")
        try:
            import pinocchio as pin
        except ImportError as exc:
            raise RuntimeError(
                "Pinocchio is required for local execution FK diagnostics."
            ) from exc

        try:
            self._model = pin.buildModelFromXML(robot_description)
        except Exception as exc:
            raise RuntimeError(
                f"Could not build the local FK model: {exc}"
            ) from exc
        self._pin = pin
        self._data = self._model.createData()
        self._neutral = pin.neutral(self._model)

        frame_id = self._model.getFrameId(ee_link)
        if frame_id >= len(self._model.frames):
            raise RuntimeError(
                f"robot_description has no TCP frame named {ee_link}."
            )
        self._frame_id = frame_id
        self._joint_q_indices = {}
        for joint in JOINTS:
            joint_id = self._model.getJointId(joint)
            if joint_id == 0:
                raise RuntimeError(
                    f"robot_description has no joint named {joint}."
                )
            model_joint = self._model.joints[joint_id]
            if model_joint.nq != 1:
                raise RuntimeError(
                    f"Local FK expected one coordinate for {joint}, "
                    f"but the model reports nq={model_joint.nq}."
                )
            self._joint_q_indices[joint] = model_joint.idx_q

    def position(
        self,
        joint_positions: Mapping[str, float],
    ) -> tuple[float, float, float]:
        missing = [joint for joint in JOINTS if joint not in joint_positions]
        if missing:
            raise RuntimeError(
                f"Local FK joint state is missing positions: {missing}"
            )
        values = [float(joint_positions[joint]) for joint in JOINTS]
        if any(not math.isfinite(value) for value in values):
            raise RuntimeError("Local FK joint state contains non-finite values.")

        q = self._neutral.copy()
        for joint, value in zip(JOINTS, values):
            q[self._joint_q_indices[joint]] = value
        self._pin.forwardKinematics(self._model, self._data, q)
        self._pin.updateFramePlacements(self._model, self._data)
        translation = self._data.oMf[self._frame_id].translation
        return tuple(float(translation[index]) for index in range(3))


def maximum_joint_velocity(
    velocities: Mapping[str, float],
) -> tuple[str, float]:
    missing = [joint for joint in JOINTS if joint not in velocities]
    if missing:
        raise RuntimeError(
            f"Measured joint state is missing velocities: {missing}"
        )
    values = {joint: abs(float(velocities[joint])) for joint in JOINTS}
    if any(not math.isfinite(value) for value in values.values()):
        raise RuntimeError("Measured joint velocity is not finite.")
    joint = max(JOINTS, key=values.get)
    return joint, values[joint]


def controller_state_sample_from_message(
    message,
    *,
    receipt_time: float,
) -> ControllerStateSample:
    names = tuple(message.joint_names)
    if len(names) != len(set(names)):
        raise RuntimeError("Controller state contains duplicate joint names.")
    missing = [joint for joint in JOINTS if joint not in names]
    if missing:
        raise RuntimeError(
            f"Controller state is missing FR3 joints: {missing}"
        )

    desired = (
        message.reference
        if len(message.reference.positions) == len(names)
        else message.desired
    )
    actual = (
        message.feedback
        if len(message.feedback.positions) == len(names)
        else message.actual
    )
    desired_positions = _joint_values(
        names,
        desired.positions,
        "desired positions",
        required=True,
    )
    actual_positions = _joint_values(
        names,
        actual.positions,
        "actual positions",
        required=True,
    )
    position_errors = _joint_values(
        names,
        message.error.positions,
        "position errors",
        required=False,
    )
    if not position_errors:
        position_errors = {
            joint: desired_positions[joint] - actual_positions[joint]
            for joint in JOINTS
        }

    desired_velocities = _joint_values(
        names,
        desired.velocities,
        "desired velocities",
        required=True,
    )
    actual_velocities = _joint_values(
        names,
        actual.velocities,
        "actual velocities",
        required=True,
    )
    velocity_errors = _joint_values(
        names,
        message.error.velocities,
        "velocity errors",
        required=False,
    )
    if not velocity_errors:
        velocity_errors = {
            joint: desired_velocities[joint] - actual_velocities[joint]
            for joint in JOINTS
        }
    output_efforts = _joint_values(
        names,
        message.output.effort,
        "output efforts",
        required=False,
    )

    stamp = message.header.stamp
    message_time = float(stamp.sec) + float(stamp.nanosec) * 1e-9
    return ControllerStateSample(
        receipt_time=float(receipt_time),
        message_time=message_time,
        desired_positions=desired_positions,
        actual_positions=actual_positions,
        position_errors=position_errors,
        desired_velocities=desired_velocities,
        actual_velocities=actual_velocities,
        velocity_errors=velocity_errors,
        output_efforts=output_efforts,
    )


def _inertia_properties(stamped) -> InertiaProperties:
    inertia = stamped.inertia
    values = (
        inertia.m,
        inertia.com.x,
        inertia.com.y,
        inertia.com.z,
        inertia.ixx,
        inertia.ixy,
        inertia.ixz,
        inertia.iyy,
        inertia.iyz,
        inertia.izz,
    )
    if any(not math.isfinite(float(value)) for value in values):
        raise RuntimeError(
            "Franka inertia configuration contains non-finite values."
        )
    return InertiaProperties(
        frame_id=stamped.header.frame_id,
        mass=float(inertia.m),
        center_of_mass=(
            float(inertia.com.x),
            float(inertia.com.y),
            float(inertia.com.z),
        ),
        inertia=(
            (float(inertia.ixx), float(inertia.ixy), float(inertia.ixz)),
            (float(inertia.ixy), float(inertia.iyy), float(inertia.iyz)),
            (float(inertia.ixz), float(inertia.iyz), float(inertia.izz)),
        ),
    )


def franka_inertia_configuration_from_robot_state(
    message,
) -> FrankaInertiaConfiguration:
    return FrankaInertiaConfiguration(
        end_effector=_inertia_properties(message.inertia_ee),
        load=_inertia_properties(message.inertia_load),
        total=_inertia_properties(message.inertia_total),
    )


def build_cartesian_tracking_samples(
    samples: Sequence[ControllerStateSample],
    *,
    execution_start_time: float,
    fk,
) -> tuple[CartesianTrackingSample, ...]:
    execution_samples = [
        sample
        for sample in samples
        if sample.receipt_time >= execution_start_time
    ]
    if len(execution_samples) < 2:
        raise RuntimeError(
            "Fewer than two controller-state samples were captured during "
            "execution."
        )

    result = []
    for sample in execution_samples:
        desired_tcp = tuple(fk.position(sample.desired_positions))
        actual_tcp = tuple(fk.position(sample.actual_positions))
        if len(desired_tcp) != 3 or len(actual_tcp) != 3:
            raise RuntimeError("Execution FK must return three coordinates.")
        if any(
            not math.isfinite(value)
            for value in (*desired_tcp, *actual_tcp)
        ):
            raise RuntimeError("Execution FK returned non-finite coordinates.")
        error = _distance(desired_tcp, actual_tcp)
        result.append(
            CartesianTrackingSample(
                time_from_execute=(
                    sample.receipt_time - execution_start_time
                ),
                controller=sample,
                desired_tcp=desired_tcp,
                actual_tcp=actual_tcp,
                cartesian_error=error,
            )
        )
    return tuple(result)


def analyze_execution_tracking(
    samples: Sequence[CartesianTrackingSample],
    *,
    start_tcp: Sequence[float],
    command_duration: float,
    initial_window_duration: float,
    joint7_goal_tolerance: float,
    error_growth_epsilon: float = 1e-5,
    consecutive_growth_samples: int = 2,
    desired_step_residual_threshold: float = 1e-3,
) -> ExecutionTrackingReport:
    if len(samples) < 2:
        raise RuntimeError(
            "Execution tracking analysis requires at least two samples."
        )
    if len(start_tcp) != 3 or any(
        not math.isfinite(float(value)) for value in start_tcp
    ):
        raise RuntimeError("Execution tracking start TCP is invalid.")
    if not math.isfinite(error_growth_epsilon) or error_growth_epsilon < 0.0:
        raise RuntimeError("Tracking error growth epsilon is invalid.")
    if consecutive_growth_samples <= 0:
        raise RuntimeError(
            "Consecutive tracking-error growth samples must be positive."
        )
    for label, value in (
        ("command duration", command_duration),
        ("initial tracking window", initial_window_duration),
        ("joint 7 goal tolerance", joint7_goal_tolerance),
        ("desired-step residual threshold", desired_step_residual_threshold),
    ):
        if not math.isfinite(value) or value <= 0.0:
            raise RuntimeError(f"Execution {label} must be positive.")

    start = tuple(float(value) for value in start_tcp)
    minimum_sample = min(samples, key=lambda sample: sample.actual_tcp[2])
    maximum_cartesian_sample = max(
        samples,
        key=lambda sample: sample.cartesian_error,
    )
    first_joint, first_time = _first_increasing_joint_error(
        samples,
        epsilon=error_growth_epsilon,
        consecutive_samples=consecutive_growth_samples,
    )
    maximum_joint_errors = {
        joint: max(
            abs(sample.controller.position_errors[joint])
            for sample in samples
        )
        for joint in JOINTS
    }
    maximum_velocity_errors = {
        joint: max(
            abs(sample.controller.velocity_errors[joint])
            for sample in samples
        )
        for joint in JOINTS
    }
    initial_samples = [
        sample
        for sample in samples
        if sample.time_from_execute <= initial_window_duration
    ]
    if len(initial_samples) < 2:
        raise RuntimeError(
            "Fewer than two controller samples were captured in the initial "
            "tracking window."
        )
    initial_minimum = min(
        initial_samples,
        key=lambda sample: sample.actual_tcp[2],
    )
    initial_z_offsets = [
        sample.actual_tcp[2] - start[2] for sample in initial_samples
    ]
    initial_z_error_correlations = {
        joint: _pearson_correlation(
            initial_z_offsets,
            [
                sample.controller.position_errors[joint]
                for sample in initial_samples
            ],
        )
        for joint in JOINTS
    }
    joint7 = _analyze_joint7_tracking(
        samples,
        command_duration=command_duration,
        goal_tolerance=joint7_goal_tolerance,
        desired_step_residual_threshold=(
            desired_step_residual_threshold
        ),
    )
    final = samples[-1]
    return ExecutionTrackingReport(
        sample_count=len(samples),
        minimum_actual_z_offset=minimum_sample.actual_tcp[2] - start[2],
        minimum_actual_z_time=minimum_sample.time_from_execute,
        desired_z_offset_at_minimum=minimum_sample.desired_tcp[2] - start[2],
        maximum_cartesian_error=maximum_cartesian_sample.cartesian_error,
        maximum_cartesian_error_time=(
            maximum_cartesian_sample.time_from_execute
        ),
        first_increasing_error_joint=first_joint,
        first_increasing_error_time=first_time,
        maximum_joint_errors=maximum_joint_errors,
        maximum_velocity_errors=maximum_velocity_errors,
        initial_window_duration=initial_window_duration,
        initial_minimum_actual_z_offset=(
            initial_minimum.actual_tcp[2] - start[2]
        ),
        initial_minimum_actual_z_time=initial_minimum.time_from_execute,
        initial_desired_z_offset_at_minimum=(
            initial_minimum.desired_tcp[2] - start[2]
        ),
        initial_joint_errors_at_minimum=dict(
            initial_minimum.controller.position_errors
        ),
        initial_z_error_correlations=initial_z_error_correlations,
        joint7=joint7,
        final_desired_offset=tuple(
            final.desired_tcp[index] - start[index] for index in range(3)
        ),
        final_actual_offset=tuple(
            final.actual_tcp[index] - start[index] for index in range(3)
        ),
    )


def _analyze_joint7_tracking(
    samples: Sequence[CartesianTrackingSample],
    *,
    command_duration: float,
    goal_tolerance: float,
    desired_step_residual_threshold: float,
) -> Joint7TrackingDiagnostics:
    joint = "fr3_joint7"
    command_samples = [
        sample
        for sample in samples
        if sample.time_from_execute <= command_duration
    ]
    if len(command_samples) < 3:
        raise RuntimeError(
            "Fewer than three joint-7 samples were captured during the "
            "commanded trajectory."
        )

    first = command_samples[0]
    final = command_samples[-1]
    desired_start = first.controller.desired_positions[joint]
    actual_start = first.controller.actual_positions[joint]
    desired_final = final.controller.desired_positions[joint]
    actual_final = final.controller.actual_positions[joint]
    desired_motion = desired_final - desired_start
    actual_motion = actual_final - actual_start
    direction = 1.0 if desired_motion >= 0.0 else -1.0

    maximum_error_sample = max(
        command_samples,
        key=lambda sample: abs(
            sample.controller.position_errors[joint]
        ),
    )
    step_records = []
    for previous, current in zip(command_samples, command_samples[1:]):
        delta_time = (
            current.time_from_execute - previous.time_from_execute
        )
        if delta_time <= 0.0:
            raise RuntimeError(
                "Controller-state receipt times are not strictly increasing."
            )
        desired_step = (
            current.controller.desired_positions[joint]
            - previous.controller.desired_positions[joint]
        )
        predicted_step = 0.5 * (
            previous.controller.desired_velocities[joint]
            + current.controller.desired_velocities[joint]
        ) * delta_time
        step_records.append(
            (
                current.time_from_execute,
                abs(desired_step),
                abs(desired_step - predicted_step),
            )
        )
    maximum_step = max(step_records, key=lambda record: record[1])
    maximum_residual = max(step_records, key=lambda record: record[2])

    motion_size = abs(desired_motion)
    onset_threshold = min(
        0.001,
        max(0.0001, motion_size * 0.05),
    )
    desired_onset = _motion_onset_time(
        command_samples,
        joint,
        desired_start,
        direction,
        onset_threshold,
        desired=True,
    )
    actual_onset = _motion_onset_time(
        command_samples,
        joint,
        actual_start,
        direction,
        onset_threshold,
        desired=False,
    )
    onset_delay = (
        actual_onset - desired_onset
        if desired_onset is not None and actual_onset is not None
        else None
    )
    initially_sticks = (
        motion_size >= 0.002
        and desired_onset is not None
        and (
            actual_onset is None
            or actual_onset - desired_onset >= 0.15
        )
    )

    lag_samples = [
        sample
        for sample in command_samples
        if desired_onset is not None
        and sample.time_from_execute >= desired_onset
    ]
    lag_count = sum(
        direction
        * (
            sample.controller.desired_positions[joint]
            - sample.controller.actual_positions[joint]
        )
        > 1e-4
        for sample in lag_samples
    )
    lag_fraction = (
        lag_count / len(lag_samples) if lag_samples else 0.0
    )
    continuously_lags = (
        lag_fraction >= 0.80
        and abs(final.controller.position_errors[joint])
        > goal_tolerance
    )

    second_half = [
        sample
        for sample in command_samples
        if sample.time_from_execute >= command_duration * 0.5
    ]
    target_errors = [
        abs(
            sample.controller.actual_positions[joint]
            - desired_final
        )
        for sample in second_half
    ]
    reached_goal = any(error <= goal_tolerance for error in target_errors)
    final_target_error = abs(actual_final - desired_final)
    reached_then_rebounded = (
        reached_goal
        and final_target_error > goal_tolerance
        and final_target_error
        > min(target_errors) + max(0.001, goal_tolerance * 0.1)
    )

    return Joint7TrackingDiagnostics(
        command_duration=command_duration,
        desired_motion=desired_motion,
        actual_motion=actual_motion,
        final_position_error=(
            final.controller.position_errors[joint]
        ),
        maximum_position_error=abs(
            maximum_error_sample.controller.position_errors[joint]
        ),
        maximum_position_error_time=(
            maximum_error_sample.time_from_execute
        ),
        desired_motion_onset_time=desired_onset,
        actual_motion_onset_time=actual_onset,
        onset_delay=onset_delay,
        lag_fraction=lag_fraction,
        maximum_desired_step=maximum_step[1],
        maximum_desired_step_time=maximum_step[0],
        maximum_desired_step_residual=maximum_residual[2],
        maximum_desired_step_residual_time=maximum_residual[0],
        desired_step_residual_threshold=desired_step_residual_threshold,
        desired_trajectory_discontinuous=(
            maximum_residual[2] > desired_step_residual_threshold
        ),
        initially_sticks=initially_sticks,
        continuously_lags=continuously_lags,
        reached_goal_tolerance=reached_goal,
        reached_then_rebounded=reached_then_rebounded,
    )


def _motion_onset_time(
    samples: Sequence[CartesianTrackingSample],
    joint: str,
    start: float,
    direction: float,
    threshold: float,
    *,
    desired: bool,
) -> float | None:
    for sample in samples:
        positions = (
            sample.controller.desired_positions
            if desired
            else sample.controller.actual_positions
        )
        if direction * (positions[joint] - start) >= threshold:
            return sample.time_from_execute
    return None


def format_controller_configuration(
    configuration: ControllerRuntimeConfiguration,
) -> str:
    confirmed = "YES" if configuration.effort_interface_confirmed else "NO"
    lines = [
        "Active trajectory-controller configuration (read-only)",
        f"  Name:                         {configuration.name}",
        f"  State:                        {configuration.state}",
        f"  Type:                         {configuration.controller_type}",
        "  Configured command interface: "
        + ", ".join(configuration.configured_command_interfaces),
        "  Claimed command interfaces:  "
        + ", ".join(configuration.claimed_interfaces),
        f"  Seven-joint effort interface: {confirmed}",
        "  Active PID gains:",
        "    joint          p          i          d    i_clamp  ff_velocity",
    ]
    for joint in JOINTS:
        gains = configuration.gains[joint]
        lines.append(
            f"    {joint:<11} "
            f"{_format_optional(gains.p):>10} "
            f"{_format_optional(gains.i):>10} "
            f"{_format_optional(gains.d):>10} "
            f"{_format_optional(gains.i_clamp):>10} "
            f"{_format_optional(gains.ff_velocity_scale):>12}"
        )
    lines.extend(
        (
            "  Active trajectory tolerances:",
            "    Goal-time tolerance:         "
            f"{_format_optional(configuration.goal_time_tolerance)} s",
            "    Stopped-velocity tolerance:  "
            f"{_format_optional(configuration.stopped_velocity_tolerance)} "
            "rad/s",
            "    joint           trajectory(rad)      goal(rad)",
        )
    )
    for joint in JOINTS:
        lines.append(
            f"    {joint:<12} "
            f"{_format_optional(configuration.trajectory_tolerances[joint]):>16} "
            f"{_format_optional(configuration.goal_tolerances[joint]):>14}"
        )

    command_match = (
        configuration.configured_command_interfaces
        == OFFICIAL_FRANKA_HUMBLE_COMMAND_INTERFACES
    )
    gain_matches = {
        joint: _pid_matches(
            configuration.gains[joint],
            OFFICIAL_FRANKA_HUMBLE_GAINS[joint],
        )
        for joint in JOINTS
    }
    controller_law_matches = command_match and all(gain_matches.values())
    tolerance_defaults_match = (
        _optional_matches(
            configuration.goal_time_tolerance,
            OFFICIAL_HUMBLE_GOAL_TIME_TOLERANCE,
        )
        and _optional_matches(
            configuration.stopped_velocity_tolerance,
            OFFICIAL_HUMBLE_STOPPED_VELOCITY_TOLERANCE,
        )
        and all(
            _optional_matches(
                configuration.goal_tolerances[joint],
                OFFICIAL_HUMBLE_JOINT_TOLERANCE,
            )
            and _optional_matches(
                configuration.trajectory_tolerances[joint],
                OFFICIAL_HUMBLE_JOINT_TOLERANCE,
            )
            for joint in JOINTS
        )
    )
    lines.extend(
        (
            "  Comparison with unmodified Franka Humble configuration:",
            "    Baseline: franka_fr3_moveit_config/config/"
            "fr3_ros_controllers.yaml",
            "    Command interface:           "
            + _match_label(command_match),
            "    PID and velocity feedforward:"
            + f" {_match_label(controller_law_matches)}",
            "    Effective tolerance defaults:"
            + f" {_match_label(tolerance_defaults_match)}",
            "    Note: the official Franka YAML does not explicitly set "
            "goal, trajectory, or goal-time tolerances; zero-valued "
            "Humble defaults are used.",
        )
    )
    return "\n".join(lines)


def format_franka_inertia_configuration(
    configuration: FrankaInertiaConfiguration,
) -> str:
    lines = [
        "Configured Franka inertia properties (read-only)",
    ]
    for label, properties in (
        ("End effector", configuration.end_effector),
        ("External load", configuration.load),
        ("Total", configuration.total),
    ):
        com = properties.center_of_mass
        lines.extend(
            (
                f"  {label}:",
                f"    Frame: {properties.frame_id or '(not reported)'}",
                f"    Mass:  {properties.mass:.6f} kg",
                "    COM:   "
                f"x={com[0]:+.6f}, y={com[1]:+.6f}, "
                f"z={com[2]:+.6f} m",
                "    Inertia tensor about COM [kg m^2]:",
            )
        )
        for row in properties.inertia:
            lines.append(
                "      ["
                + ", ".join(f"{value:+.9f}" for value in row)
                + "]"
            )
    return "\n".join(lines)


def format_execution_tracking_report(
    report: ExecutionTrackingReport,
) -> str:
    if report.first_increasing_error_joint is None:
        first_growth = "not detected"
    else:
        first_growth = (
            f"{report.first_increasing_error_joint} at "
            f"{report.first_increasing_error_time:.6f} s"
        )
    desired = report.final_desired_offset
    actual = report.final_actual_offset
    joint7 = report.joint7
    lines = [
        "Execution tracking diagnostics (read-only)",
        f"  Controller samples:                    {report.sample_count}",
        "  Minimum actual Z offset:               "
        f"{report.minimum_actual_z_offset:+.6f} m",
        "  Time of minimum actual Z:              "
        f"{report.minimum_actual_z_time:.6f} s",
        "  Desired Z offset at that time:         "
        f"{report.desired_z_offset_at_minimum:+.6f} m",
        "  Maximum desired-vs-actual TCP error:   "
        f"{report.maximum_cartesian_error:.6f} m at "
        f"{report.maximum_cartesian_error_time:.6f} s",
        f"  First increasing joint tracking error: {first_growth}",
        "  Initial TCP Z-dip correlation window:  "
        f"0.000000 to {report.initial_window_duration:.6f} s",
        "    Minimum actual Z offset:             "
        f"{report.initial_minimum_actual_z_offset:+.6f} m at "
        f"{report.initial_minimum_actual_z_time:.6f} s",
        "    Desired Z offset at that time:       "
        f"{report.initial_desired_z_offset_at_minimum:+.6f} m",
        "    Joint position errors at initial minimum Z:",
    ]
    for joint in JOINTS:
        lines.append(
            f"      {joint}: "
            f"{report.initial_joint_errors_at_minimum[joint]:+.6f} rad"
        )
    lines.append(
        "    Pearson correlation of Z offset with joint position error:"
    )
    for joint in JOINTS:
        correlation = report.initial_z_error_correlations[joint]
        text = "n/a" if correlation is None else f"{correlation:+.6f}"
        lines.append(f"      {joint}: {text}")

    lines.extend(
        (
            "  Joint-7 tracking classification during commanded trajectory:",
            "    Desired motion:                    "
            f"{joint7.desired_motion:+.6f} rad",
            "    Actual motion:                     "
            f"{joint7.actual_motion:+.6f} rad",
            "    Final position error:              "
            f"{joint7.final_position_error:+.6f} rad",
            "    Maximum position error:            "
            f"{joint7.maximum_position_error:.6f} rad at "
            f"{joint7.maximum_position_error_time:.6f} s",
            "    Desired motion onset:              "
            f"{_format_time(joint7.desired_motion_onset_time)}",
            "    Actual motion onset:               "
            f"{_format_time(joint7.actual_motion_onset_time)}",
            "    Actual onset delay:                "
            f"{_format_time(joint7.onset_delay)}",
            "    Fraction of moving samples lagging:"
            f" {joint7.lag_fraction:.3f}",
            "    Maximum desired position step:     "
            f"{joint7.maximum_desired_step:.6f} rad at "
            f"{joint7.maximum_desired_step_time:.6f} s",
            "    Maximum desired step residual:     "
            f"{joint7.maximum_desired_step_residual:.6f} rad at "
            f"{joint7.maximum_desired_step_residual_time:.6f} s "
            f"(diagnostic threshold "
            f"{joint7.desired_step_residual_threshold:.6f})",
            "    Desired trajectory discontinuity:  "
            f"{_yes_no(joint7.desired_trajectory_discontinuous)}",
            "    Initially sticks:                  "
            f"{_yes_no(joint7.initially_sticks)}",
            "    Continuously lags:                 "
            f"{_yes_no(joint7.continuously_lags)}",
            "    Reached configured goal tolerance: "
            f"{_yes_no(joint7.reached_goal_tolerance)}",
            "    Reached then rebounded:            "
            f"{_yes_no(joint7.reached_then_rebounded)}",
        )
    )
    lines.extend(
        (
        "  Final desired TCP offset:              "
        f"dx={desired[0]:+.6f}, dy={desired[1]:+.6f}, "
        f"dz={desired[2]:+.6f} m",
        "  Final actual TCP offset:               "
        f"dx={actual[0]:+.6f}, dy={actual[1]:+.6f}, "
        f"dz={actual[2]:+.6f} m",
        "  Maximum position tracking error by joint:",
        )
    )
    for joint in JOINTS:
        lines.append(
            f"    {joint}: {report.maximum_joint_errors[joint]:.6f} rad"
        )
    lines.append("  Maximum velocity tracking error by joint:")
    for joint in JOINTS:
        lines.append(
            f"    {joint}: "
            f"{report.maximum_velocity_errors[joint]:.6f} rad/s"
        )
    return "\n".join(lines)


def write_execution_trace_csv(
    path: Path,
    samples: Sequence[CartesianTrackingSample],
) -> None:
    if not samples:
        raise RuntimeError("Cannot write an empty execution trace.")
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "time_from_execute_s",
        "controller_stamp_s",
        "desired_tcp_x_m",
        "desired_tcp_y_m",
        "desired_tcp_z_m",
        "actual_tcp_x_m",
        "actual_tcp_y_m",
        "actual_tcp_z_m",
        "cartesian_error_m",
    ]
    for joint in JOINTS:
        fields.extend(
            (
                f"{joint}_desired_position_rad",
                f"{joint}_actual_position_rad",
                f"{joint}_position_error_rad",
                f"{joint}_desired_velocity_rad_s",
                f"{joint}_actual_velocity_rad_s",
                f"{joint}_velocity_error_rad_s",
                f"{joint}_output_effort_nm",
            )
        )

    with path.open("w", encoding="ascii", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for sample in samples:
            controller = sample.controller
            row = {
                "time_from_execute_s": sample.time_from_execute,
                "controller_stamp_s": controller.message_time,
                "desired_tcp_x_m": sample.desired_tcp[0],
                "desired_tcp_y_m": sample.desired_tcp[1],
                "desired_tcp_z_m": sample.desired_tcp[2],
                "actual_tcp_x_m": sample.actual_tcp[0],
                "actual_tcp_y_m": sample.actual_tcp[1],
                "actual_tcp_z_m": sample.actual_tcp[2],
                "cartesian_error_m": sample.cartesian_error,
            }
            for joint in JOINTS:
                row.update(
                    {
                        f"{joint}_desired_position_rad": (
                            controller.desired_positions[joint]
                        ),
                        f"{joint}_actual_position_rad": (
                            controller.actual_positions[joint]
                        ),
                        f"{joint}_position_error_rad": (
                            controller.position_errors[joint]
                        ),
                        f"{joint}_desired_velocity_rad_s": (
                            controller.desired_velocities.get(joint, "")
                        ),
                        f"{joint}_actual_velocity_rad_s": (
                            controller.actual_velocities.get(joint, "")
                        ),
                        f"{joint}_velocity_error_rad_s": (
                            controller.velocity_errors.get(joint, "")
                        ),
                        f"{joint}_output_effort_nm": (
                            controller.output_efforts.get(joint, "")
                        ),
                    }
                )
            writer.writerow(row)


def _first_increasing_joint_error(
    samples: Sequence[CartesianTrackingSample],
    *,
    epsilon: float,
    consecutive_samples: int,
) -> tuple[str | None, float | None]:
    candidates = []
    for joint_index, joint in enumerate(JOINTS):
        run_start = None
        run_length = 0
        for index in range(1, len(samples)):
            previous = abs(
                samples[index - 1].controller.position_errors[joint]
            )
            current = abs(samples[index].controller.position_errors[joint])
            if current > previous + epsilon:
                if run_start is None:
                    run_start = index
                run_length += 1
                if run_length >= consecutive_samples:
                    candidates.append(
                        (
                            samples[run_start].time_from_execute,
                            joint_index,
                            joint,
                        )
                    )
                    break
            else:
                run_start = None
                run_length = 0
    if not candidates:
        return None, None
    first = min(candidates)
    return first[2], first[0]


def _joint_values(
    names: Sequence[str],
    values: Sequence[float],
    label: str,
    *,
    required: bool,
) -> dict[str, float]:
    if not values:
        if required:
            raise RuntimeError(f"Controller state has no {label}.")
        return {}
    if len(values) != len(names):
        raise RuntimeError(
            f"Controller-state {label} vector has length {len(values)}; "
            f"expected {len(names)}."
        )
    result = {
        joint: float(values[names.index(joint)])
        for joint in JOINTS
    }
    if any(not math.isfinite(value) for value in result.values()):
        raise RuntimeError(f"Controller-state {label} is not finite.")
    return result


def _distance(first: Sequence[float], second: Sequence[float]) -> float:
    return math.sqrt(
        sum((float(first[index]) - float(second[index])) ** 2 for index in range(3))
    )


def _pearson_correlation(
    first: Sequence[float],
    second: Sequence[float],
) -> float | None:
    if len(first) != len(second) or len(first) < 2:
        return None
    first_mean = sum(first) / len(first)
    second_mean = sum(second) / len(second)
    first_centered = [value - first_mean for value in first]
    second_centered = [value - second_mean for value in second]
    first_scale = math.sqrt(sum(value * value for value in first_centered))
    second_scale = math.sqrt(sum(value * value for value in second_centered))
    if first_scale <= 1e-15 or second_scale <= 1e-15:
        return None
    return sum(
        first_value * second_value
        for first_value, second_value in zip(
            first_centered,
            second_centered,
        )
    ) / (first_scale * second_scale)


def _optional_matches(actual: float | None, expected: float) -> bool:
    return actual is not None and math.isclose(
        actual,
        expected,
        rel_tol=0.0,
        abs_tol=1e-12,
    )


def _pid_matches(actual: PIDGains, expected: PIDGains) -> bool:
    return all(
        _optional_matches(actual_value, expected_value)
        for actual_value, expected_value in (
            (actual.p, expected.p),
            (actual.i, expected.i),
            (actual.d, expected.d),
            (actual.i_clamp, expected.i_clamp),
            (
                actual.ff_velocity_scale,
                expected.ff_velocity_scale,
            ),
        )
    )


def _match_label(matches: bool) -> str:
    return "MATCH" if matches else "DIFFERS"


def _yes_no(value: bool) -> str:
    return "YES" if value else "NO"


def _format_time(value: float | None) -> str:
    return "not detected" if value is None else f"{value:.6f} s"


def _format_optional(value: float | None) -> str:
    return "not-set" if value is None else f"{value:.6f}"
