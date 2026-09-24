#!/usr/bin/env python3
"""Pure safety checks for small MoveIt Cartesian trajectories."""

from dataclasses import dataclass
import math
from typing import Mapping, Sequence, Tuple


@dataclass(frozen=True)
class TrajectorySafetyReport:
    joint_names: Tuple[str, ...]
    point_count: int
    duration: float
    timing_scale: float
    max_start_error: float
    max_start_error_joint: str
    max_joint_excursion: float
    max_joint_excursion_joint: str
    max_joint_step: float
    max_joint_step_joint: str
    max_planned_velocity: float
    max_planned_velocity_joint: str
    max_planned_acceleration: float
    max_planned_acceleration_joint: str


@dataclass(frozen=True)
class CartesianVerificationReport:
    requested_distance: float
    measured_distance: float
    projected_motion: float
    target_error: float
    orientation_error: float
    max_joint_target_error: float
    max_joint_target_error_joint: str
    max_settled_velocity: float
    max_settled_velocity_joint: str
    direction_ok: bool
    movement_ok: bool
    position_ok: bool
    orientation_ok: bool
    joint_target_ok: bool
    settled_velocity_ok: bool

    @property
    def passed(self) -> bool:
        return (
            self.direction_ok
            and self.movement_ok
            and self.position_ok
            and self.orientation_ok
            and self.joint_target_ok
            and self.settled_velocity_ok
        )


def scale_timed_trajectory(
    trajectory,
    *,
    max_velocity: float,
    max_acceleration: float,
) -> float:
    """Slow a timed trajectory to absolute joint-dynamics limits."""
    _validate_positive("max_velocity", max_velocity)
    _validate_positive("max_acceleration", max_acceleration)

    points = trajectory.joint_trajectory.points
    if len(points) < 2:
        raise ValueError(
            "Cartesian trajectory must contain at least two points."
        )

    original_times = [
        _duration_seconds(point.time_from_start) for point in points
    ]
    _validate_strict_timing(original_times)

    original_max_velocity = 0.0
    original_max_acceleration = 0.0
    for point in points:
        if point.velocities:
            original_max_velocity = max(
                original_max_velocity,
                max(abs(value) for value in point.velocities),
            )
        if point.accelerations:
            original_max_acceleration = max(
                original_max_acceleration,
                max(abs(value) for value in point.accelerations),
            )

    timing_scale = max(
        1.0,
        original_max_velocity / max_velocity,
        math.sqrt(original_max_acceleration / max_acceleration),
    )
    for point, original_time in zip(points, original_times):
        _set_duration(point.time_from_start, original_time * timing_scale)
        if point.velocities:
            point.velocities = [
                value / timing_scale for value in point.velocities
            ]
        if point.accelerations:
            point.accelerations = [
                value / (timing_scale * timing_scale)
                for value in point.accelerations
            ]

    return timing_scale


def validate_trajectory(
    trajectory,
    current_positions: Mapping[str, float],
    expected_joints: Sequence[str],
    *,
    timing_scale: float,
    max_start_error: float,
    max_joint_excursion: float,
    max_joint_step: float,
    max_planned_velocity: float,
    max_planned_acceleration: float,
) -> TrajectorySafetyReport:
    """Reject malformed, discontinuous, or unexpectedly large paths."""
    for name, value in (
        ("timing_scale", timing_scale),
        ("max_start_error", max_start_error),
        ("max_joint_excursion", max_joint_excursion),
        ("max_joint_step", max_joint_step),
        ("max_planned_velocity", max_planned_velocity),
        ("max_planned_acceleration", max_planned_acceleration),
    ):
        _validate_positive(name, value)

    expected = tuple(expected_joints)
    names = tuple(trajectory.joint_trajectory.joint_names)
    if len(names) != len(set(names)):
        raise ValueError(
            "Cartesian trajectory contains duplicate joint names."
        )
    if set(names) != set(expected) or len(names) != len(expected):
        raise ValueError(
            "Cartesian trajectory joint names do not match the seven FR3 "
            "arm joints. "
            f"Received: {list(names)}"
        )

    missing = [joint for joint in expected if joint not in current_positions]
    if missing:
        raise ValueError(f"Measured start state is missing joints: {missing}")
    non_finite_start = [
        joint
        for joint in expected
        if not math.isfinite(current_positions[joint])
    ]
    if non_finite_start:
        raise ValueError(
            "Measured start state contains non-finite values: "
            f"{non_finite_start}"
        )

    points = trajectory.joint_trajectory.points
    if len(points) < 2:
        raise ValueError(
            "Cartesian trajectory must contain at least two points."
        )

    times = []
    for index, point in enumerate(points):
        _validate_vector(
            f"point {index} positions", point.positions, len(names)
        )
        _validate_vector(
            f"point {index} velocities", point.velocities, len(names)
        )
        _validate_vector(
            f"point {index} accelerations", point.accelerations, len(names)
        )
        times.append(_duration_seconds(point.time_from_start))
    _validate_strict_timing(times)

    name_index = {joint: index for index, joint in enumerate(names)}
    first = points[0]
    start_errors = {
        joint: abs(
            first.positions[name_index[joint]] - current_positions[joint]
        )
        for joint in expected
    }
    start_error_joint = max(expected, key=start_errors.get)
    maximum_start_error = start_errors[start_error_joint]

    excursion_joint = expected[0]
    maximum_excursion = 0.0
    step_joint = expected[0]
    maximum_step = 0.0
    velocity_joint = expected[0]
    maximum_velocity = 0.0
    acceleration_joint = expected[0]
    maximum_acceleration = 0.0

    for point_index, point in enumerate(points):
        for joint in expected:
            index = name_index[joint]
            excursion = abs(point.positions[index] - current_positions[joint])
            if excursion > maximum_excursion:
                maximum_excursion = excursion
                excursion_joint = joint

            velocity = abs(point.velocities[index])
            if velocity > maximum_velocity:
                maximum_velocity = velocity
                velocity_joint = joint

            acceleration = abs(point.accelerations[index])
            if acceleration > maximum_acceleration:
                maximum_acceleration = acceleration
                acceleration_joint = joint

            if point_index > 0:
                step = abs(
                    point.positions[index]
                    - points[point_index - 1].positions[index]
                )
                if step > maximum_step:
                    maximum_step = step
                    step_joint = joint

    violations = []
    if _exceeds_limit(maximum_start_error, max_start_error):
        violations.append(
            f"planned start differs from measured {start_error_joint} by "
            f"{maximum_start_error:.6f} rad (limit {max_start_error:.6f})"
        )
    if _exceeds_limit(maximum_excursion, max_joint_excursion):
        violations.append(
            f"{excursion_joint} moves {maximum_excursion:.6f} rad along "
            "the path "
            f"(limit {max_joint_excursion:.6f})"
        )
    if _exceeds_limit(maximum_step, max_joint_step):
        violations.append(
            f"{step_joint} jumps {maximum_step:.6f} rad between adjacent "
            "points "
            f"(limit {max_joint_step:.6f})"
        )
    if _exceeds_limit(maximum_velocity, max_planned_velocity):
        violations.append(
            f"{velocity_joint} reaches {maximum_velocity:.6f} rad/s "
            f"(limit {max_planned_velocity:.6f})"
        )
    if _exceeds_limit(maximum_acceleration, max_planned_acceleration):
        violations.append(
            f"{acceleration_joint} reaches {maximum_acceleration:.6f} rad/s^2 "
            f"(limit {max_planned_acceleration:.6f})"
        )
    if violations:
        raise ValueError(
            "Unsafe Cartesian trajectory: " + "; ".join(violations)
        )

    return TrajectorySafetyReport(
        joint_names=names,
        point_count=len(points),
        duration=times[-1],
        timing_scale=timing_scale,
        max_start_error=maximum_start_error,
        max_start_error_joint=start_error_joint,
        max_joint_excursion=maximum_excursion,
        max_joint_excursion_joint=excursion_joint,
        max_joint_step=maximum_step,
        max_joint_step_joint=step_joint,
        max_planned_velocity=maximum_velocity,
        max_planned_velocity_joint=velocity_joint,
        max_planned_acceleration=maximum_acceleration,
        max_planned_acceleration_joint=acceleration_joint,
    )


def trajectory_final_positions(trajectory) -> dict[str, float]:
    names = trajectory.joint_trajectory.joint_names
    points = trajectory.joint_trajectory.points
    if not points:
        raise ValueError("Trajectory has no points.")
    final = points[-1].positions
    if len(final) != len(names):
        raise ValueError(
            "Final trajectory point has an invalid position vector."
        )
    return dict(zip(names, final))


def verify_cartesian_motion(
    *,
    start_position: Sequence[float],
    target_position: Sequence[float],
    final_position: Sequence[float],
    target_orientation: Sequence[float],
    final_orientation: Sequence[float],
    expected_joint_positions: Mapping[str, float],
    measured_joint_positions: Mapping[str, float],
    measured_joint_velocities: Mapping[str, float],
    target_error_limit: float,
    orientation_error_limit: float,
    joint_target_error_limit: float,
    max_settled_velocity: float,
    minimum_measured_motion: float,
) -> CartesianVerificationReport:
    """Verify that measured robot state agrees with the Cartesian command."""
    for name, value in (
        ("target_error_limit", target_error_limit),
        ("orientation_error_limit", orientation_error_limit),
        ("joint_target_error_limit", joint_target_error_limit),
        ("max_settled_velocity", max_settled_velocity),
        ("minimum_measured_motion", minimum_measured_motion),
    ):
        _validate_positive(name, value)

    start = _finite_vector("start_position", start_position, 3)
    target = _finite_vector("target_position", target_position, 3)
    final = _finite_vector("final_position", final_position, 3)
    target_q = _normalized_quaternion(target_orientation)
    final_q = _normalized_quaternion(final_orientation)

    joints = tuple(expected_joint_positions)
    if not joints:
        raise ValueError("Expected final joint state is empty.")
    for label, values in (
        ("expected_joint_positions", expected_joint_positions),
        ("measured_joint_positions", measured_joint_positions),
        ("measured_joint_velocities", measured_joint_velocities),
    ):
        missing = [joint for joint in joints if joint not in values]
        if missing:
            raise ValueError(f"{label} is missing joints: {missing}")
        non_finite = [
            joint for joint in joints if not math.isfinite(values[joint])
        ]
        if non_finite:
            raise ValueError(
                f"{label} has non-finite values for: {non_finite}"
            )

    requested = tuple(target[i] - start[i] for i in range(3))
    measured = tuple(final[i] - start[i] for i in range(3))
    requested_distance = _norm(requested)
    if requested_distance <= 0.0:
        raise ValueError(
            "Cartesian verification requires a non-zero translation."
        )

    measured_distance = _norm(measured)
    projected_motion = sum(
        measured[i] * requested[i] for i in range(3)
    ) / requested_distance
    target_error = _norm(tuple(final[i] - target[i] for i in range(3)))
    orientation_error = _quaternion_angle(target_q, final_q)

    joint_errors = {
        joint: abs(
            measured_joint_positions[joint] - expected_joint_positions[joint]
        )
        for joint in joints
    }
    joint_error_joint = max(joints, key=joint_errors.get)
    maximum_joint_error = joint_errors[joint_error_joint]

    velocity_joint = max(
        joints, key=lambda joint: abs(measured_joint_velocities[joint])
    )
    maximum_velocity = abs(measured_joint_velocities[velocity_joint])
    required_motion = min(minimum_measured_motion, 0.5 * requested_distance)

    return CartesianVerificationReport(
        requested_distance=requested_distance,
        measured_distance=measured_distance,
        projected_motion=projected_motion,
        target_error=target_error,
        orientation_error=orientation_error,
        max_joint_target_error=maximum_joint_error,
        max_joint_target_error_joint=joint_error_joint,
        max_settled_velocity=maximum_velocity,
        max_settled_velocity_joint=velocity_joint,
        direction_ok=projected_motion > 0.0,
        movement_ok=projected_motion >= required_motion,
        position_ok=target_error <= target_error_limit,
        orientation_ok=orientation_error <= orientation_error_limit,
        joint_target_ok=maximum_joint_error <= joint_target_error_limit,
        settled_velocity_ok=maximum_velocity <= max_settled_velocity,
    )


def format_trajectory_safety_report(report: TrajectorySafetyReport) -> str:
    return "\n".join(
        [
            "Cartesian trajectory safety check: PASS",
            f"  Points:                    {report.point_count}",
            f"  Duration after scaling:    {report.duration:.3f} s",
            f"  Timing slowdown factor:    {report.timing_scale:.3f}x",
            f"  Maximum start error:       {report.max_start_error:.6f} rad "
            f"({report.max_start_error_joint})",
            "  Maximum joint excursion:   "
            f"{report.max_joint_excursion:.6f} rad "
            f"({report.max_joint_excursion_joint})",
            f"  Maximum adjacent step:     {report.max_joint_step:.6f} rad "
            f"({report.max_joint_step_joint})",
            "  Maximum planned velocity:  "
            f"{report.max_planned_velocity:.6f} rad/s "
            f"({report.max_planned_velocity_joint})",
            "  Maximum planned accel:     "
            f"{report.max_planned_acceleration:.6f} rad/s^2 "
            f"({report.max_planned_acceleration_joint})",
        ]
    )


def format_cartesian_verification_report(
    report: CartesianVerificationReport,
    *,
    target_error_limit: float,
    orientation_error_limit: float,
    joint_target_error_limit: float,
    max_settled_velocity: float,
    execution_succeeded: bool = True,
) -> str:
    if not execution_succeeded:
        final_result = (
            "Final result: CONTROLLER EXECUTION FAILED - "
            "MEASUREMENT IS DIAGNOSTIC ONLY"
        )
    elif report.passed:
        final_result = "Final result: VERIFIED CARTESIAN SUCCESS"
    else:
        final_result = (
            "Final result: CARTESIAN VERIFICATION FAILED - "
            "DO NOT SEND ANOTHER COMMAND"
        )

    return "\n".join(
        [
            "Post-execution Cartesian verification",
            "",
            f"Requested translation:  {report.requested_distance:.6f} m",
            f"Measured translation:   {report.measured_distance:.6f} m",
            f"Motion along command:   {report.projected_motion:.6f} m "
            f"({_pass_fail(report.direction_ok and report.movement_ok)})",
            f"Cartesian target error: {report.target_error:.6f} m "
            f"({_pass_fail(report.position_ok)}; "
            f"limit {target_error_limit:.6f})",
            f"Orientation error:      {report.orientation_error:.6f} rad "
            f"({_pass_fail(report.orientation_ok)}; "
            f"limit {orientation_error_limit:.6f})",
            "Joint target error:     "
            f"{report.max_joint_target_error:.6f} rad on "
            f"{report.max_joint_target_error_joint} "
            f"({_pass_fail(report.joint_target_ok)}; "
            f"limit {joint_target_error_limit:.6f})",
            "Settled velocity:       "
            f"{report.max_settled_velocity:.6f} rad/s on "
            f"{report.max_settled_velocity_joint} "
            f"({_pass_fail(report.settled_velocity_ok)}; "
            f"limit {max_settled_velocity:.6f})",
            "",
            final_result,
        ]
    )


def _duration_seconds(duration) -> float:
    value = float(duration.sec) + float(duration.nanosec) * 1e-9
    if not math.isfinite(value) or value < 0.0:
        raise ValueError("Trajectory contains an invalid time_from_start.")
    return value


def _set_duration(duration, seconds: float) -> None:
    total_nanoseconds = int(round(seconds * 1e9))
    duration.sec, duration.nanosec = divmod(total_nanoseconds, 1_000_000_000)


def _validate_strict_timing(times: Sequence[float]) -> None:
    if not times or times[-1] <= 0.0:
        raise ValueError(
            "MoveIt returned an untimed Cartesian trajectory; execution is "
            "blocked."
        )
    for index in range(1, len(times)):
        if times[index] <= times[index - 1]:
            raise ValueError(
                "Cartesian trajectory timestamps are not strictly increasing "
                f"at points {index - 1} and {index}."
            )


def _validate_vector(
    label: str, values: Sequence[float], expected_size: int
) -> None:
    if len(values) != expected_size:
        raise ValueError(
            f"{label} has {len(values)} values; expected {expected_size}. "
            "Execution requires MoveIt-generated positions, velocities, "
            "and accelerations."
        )
    if any(not math.isfinite(value) for value in values):
        raise ValueError(f"{label} contains a non-finite value.")


def _validate_positive(name: str, value: float) -> None:
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be finite and positive.")


def _exceeds_limit(value: float, limit: float) -> bool:
    numerical_tolerance = max(1e-12, abs(limit) * 1e-9)
    return value > limit + numerical_tolerance


def _finite_vector(
    name: str, values: Sequence[float], expected_size: int
) -> Tuple[float, ...]:
    if len(values) != expected_size:
        raise ValueError(f"{name} must contain {expected_size} values.")
    result = tuple(float(value) for value in values)
    if any(not math.isfinite(value) for value in result):
        raise ValueError(f"{name} contains a non-finite value.")
    return result


def _normalized_quaternion(values: Sequence[float]) -> Tuple[float, ...]:
    quaternion = _finite_vector("quaternion", values, 4)
    magnitude = _norm(quaternion)
    if magnitude <= 1e-12:
        raise ValueError("Quaternion magnitude must be non-zero.")
    return tuple(value / magnitude for value in quaternion)


def _quaternion_angle(
    first: Sequence[float], second: Sequence[float]
) -> float:
    dot = abs(sum(a * b for a, b in zip(first, second)))
    return 2.0 * math.acos(min(1.0, max(-1.0, dot)))


def _norm(values: Sequence[float]) -> float:
    return math.sqrt(sum(value * value for value in values))


def _pass_fail(value: bool) -> str:
    return "PASS" if value else "FAIL"
