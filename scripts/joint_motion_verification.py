#!/usr/bin/env python3
"""Pure post-execution checks for measured FR3 joint motion."""

from dataclasses import dataclass
import math
from typing import Any, Mapping, Sequence, Tuple


DEFAULT_TARGET_ERROR_LIMIT = 0.01
DEFAULT_JOINT7_TARGET_ERROR_LIMIT = 0.01


@dataclass(frozen=True)
class CommandedJointResult:
    joint: str
    requested_delta: float
    measured_delta: float
    target_error: float
    target_error_limit: float
    direction_ok: bool
    movement_ok: bool
    target_ok: bool


@dataclass(frozen=True)
class MotionVerificationReport:
    commanded_joints: Tuple[CommandedJointResult, ...]
    max_non_commanded_drift: float
    max_non_commanded_drift_joint: str
    non_commanded_drift_ok: bool
    max_settled_velocity: float
    max_settled_velocity_joint: str
    settled_velocity_ok: bool

    @property
    def passed(self) -> bool:
        commanded_ok = all(
            result.direction_ok and result.movement_ok and result.target_ok
            for result in self.commanded_joints
        )
        return (
            bool(self.commanded_joints)
            and commanded_ok
            and self.non_commanded_drift_ok
            and self.settled_velocity_ok
        )

    @property
    def target_state_passed(self) -> bool:
        """Whether the final measured joint state is acceptable.

        This omits per-joint direction and minimum-motion requirements. It is
        intended for redundant Cartesian IK, where several joint coordinates
        can remain near their starts while the measured end effector reaches
        the requested pose.
        """
        target_ok = all(
            result.target_ok for result in self.commanded_joints
        )
        return (
            bool(self.commanded_joints)
            and target_ok
            and self.non_commanded_drift_ok
            and self.settled_velocity_ok
        )


def verification_report_payload(
    report: MotionVerificationReport,
) -> dict[str, Any]:
    """Convert a passed or failed report into a JSON-safe audit payload."""
    return {
        "status": "PASSED" if report.passed else "FAILED",
        "commanded_joints": [
            {
                "joint": result.joint,
                "requested_delta": result.requested_delta,
                "measured_delta": result.measured_delta,
                "target_error": result.target_error,
                "target_error_limit": result.target_error_limit,
                "direction_ok": result.direction_ok,
                "movement_ok": result.movement_ok,
                "target_ok": result.target_ok,
            }
            for result in report.commanded_joints
        ],
        "max_non_commanded_drift": report.max_non_commanded_drift,
        "max_non_commanded_drift_joint": report.max_non_commanded_drift_joint,
        "non_commanded_drift_ok": report.non_commanded_drift_ok,
        "max_settled_velocity": report.max_settled_velocity,
        "max_settled_velocity_joint": report.max_settled_velocity_joint,
        "settled_velocity_ok": report.settled_velocity_ok,
    }


def verify_joint_motion(
    joints: Sequence[str],
    before: Mapping[str, float],
    target: Mapping[str, float],
    after: Mapping[str, float],
    final_velocity: Mapping[str, float],
    *,
    target_error_limit: float,
    joint_target_error_limits: Mapping[str, float] | None = None,
    non_commanded_drift_limit: float,
    max_settled_velocity: float,
    minimum_measured_delta: float,
    command_delta_threshold: float = 1e-9,
) -> MotionVerificationReport:
    """Compare a commanded joint target with a fresh measured robot state."""
    _validate_threshold("target_error_limit", target_error_limit)
    joint_target_error_limits = dict(joint_target_error_limits or {})
    unknown_limit_joints = sorted(set(joint_target_error_limits) - set(joints))
    if unknown_limit_joints:
        raise ValueError(
            "joint_target_error_limits contains unknown joints: "
            f"{unknown_limit_joints}"
        )
    for joint, limit in joint_target_error_limits.items():
        _validate_threshold(f"joint_target_error_limits[{joint}]", limit)
    _validate_threshold("non_commanded_drift_limit", non_commanded_drift_limit)
    _validate_threshold("max_settled_velocity", max_settled_velocity)
    _validate_threshold("minimum_measured_delta", minimum_measured_delta)
    if (
        not math.isfinite(command_delta_threshold)
        or command_delta_threshold < 0.0
    ):
        raise ValueError(
            "command_delta_threshold must be finite and non-negative."
        )

    _validate_joint_map("before", joints, before)
    _validate_joint_map("target", joints, target)
    _validate_joint_map("after", joints, after)
    _validate_joint_map("final_velocity", joints, final_velocity)

    commanded = [
        joint
        for joint in joints
        if abs(target[joint] - before[joint]) > command_delta_threshold
    ]
    if not commanded:
        raise ValueError("Post-execution verification requires at least one moving joint.")

    commanded_results = []
    for joint in commanded:
        requested_delta = target[joint] - before[joint]
        measured_delta = after[joint] - before[joint]
        joint_error_limit = joint_target_error_limits.get(
            joint,
            target_error_limit,
        )
        required_motion = min(minimum_measured_delta, 0.5 * abs(requested_delta))
        commanded_results.append(
            CommandedJointResult(
                joint=joint,
                requested_delta=requested_delta,
                measured_delta=measured_delta,
                target_error=abs(after[joint] - target[joint]),
                target_error_limit=joint_error_limit,
                direction_ok=requested_delta * measured_delta > 0.0,
                movement_ok=abs(measured_delta) >= required_motion,
                target_ok=abs(after[joint] - target[joint]) <= joint_error_limit,
            )
        )

    non_commanded = [joint for joint in joints if joint not in commanded]
    if non_commanded:
        drift_joint = max(non_commanded, key=lambda joint: abs(after[joint] - before[joint]))
        max_drift = abs(after[drift_joint] - before[drift_joint])
    else:
        drift_joint = "n/a"
        max_drift = 0.0

    velocity_joint = max(joints, key=lambda joint: abs(final_velocity[joint]))
    maximum_velocity = abs(final_velocity[velocity_joint])

    return MotionVerificationReport(
        commanded_joints=tuple(commanded_results),
        max_non_commanded_drift=max_drift,
        max_non_commanded_drift_joint=drift_joint,
        non_commanded_drift_ok=max_drift <= non_commanded_drift_limit,
        max_settled_velocity=maximum_velocity,
        max_settled_velocity_joint=velocity_joint,
        settled_velocity_ok=maximum_velocity <= max_settled_velocity,
    )


def format_verification_report(
    report: MotionVerificationReport,
    *,
    target_error_limit: float,
    joint_target_error_limits: Mapping[str, float] | None = None,
    non_commanded_drift_limit: float,
    max_settled_velocity: float,
    require_joint_motion: bool = True,
) -> str:
    lines = [
        "Post-execution verification",
        "",
        "joint        requested    measured   target error  direction  motion  accuracy",
    ]
    for result in report.commanded_joints:
        lines.append(
            f"{result.joint:<11} {result.requested_delta:>+10.6f}  "
            f"{result.measured_delta:>+10.6f}  {result.target_error:>12.6f}  "
            f"{_pass_fail(result.direction_ok):>9}  "
            f"{_pass_fail(result.movement_ok):>6}  "
            f"{_pass_fail(result.target_ok):>8}"
        )

    if not require_joint_motion:
        lines.extend(
            [
                "",
                "Direction and motion are diagnostic for redundant Cartesian "
                "IK components.",
            ]
        )

    verification_passed = (
        report.passed
        if require_joint_motion
        else report.target_state_passed
    )
    success_label = (
        "Final result: VERIFIED SUCCESS"
        if require_joint_motion
        else "Final result: VERIFIED JOINT TARGET STATE"
    )
    failure_label = (
        "Final result: VERIFICATION FAILED - DO NOT SEND ANOTHER COMMAND"
        if require_joint_motion
        else (
            "Final result: JOINT TARGET-STATE VERIFICATION FAILED - "
            "DO NOT SEND ANOTHER COMMAND"
        )
    )

    configured_joint_limits = dict(joint_target_error_limits or {})
    target_limit_text = f"Target-error limit:  {target_error_limit:.6f} rad"
    if configured_joint_limits:
        overrides = ", ".join(
            f"{joint}={limit:.6f} rad"
            for joint, limit in sorted(configured_joint_limits.items())
        )
        target_limit_text = (
            f"Target-error limits: default={target_error_limit:.6f} rad; "
            f"{overrides}"
        )

    lines.extend(
        [
            "",
            "Non-commanded drift: "
            f"{_pass_fail(report.non_commanded_drift_ok)} "
            f"(max {report.max_non_commanded_drift:.6f} rad on "
            f"{report.max_non_commanded_drift_joint}; "
            f"limit {non_commanded_drift_limit:.6f})",
            "Settled velocity:     "
            f"{_pass_fail(report.settled_velocity_ok)} "
            f"(max {report.max_settled_velocity:.6f} rad/s on "
            f"{report.max_settled_velocity_joint}; "
            f"limit {max_settled_velocity:.6f})",
            target_limit_text,
            "",
            success_label if verification_passed else failure_label,
        ]
    )
    return "\n".join(lines)


def _validate_joint_map(
    label: str,
    joints: Sequence[str],
    values: Mapping[str, float],
) -> None:
    missing = [joint for joint in joints if joint not in values]
    if missing:
        raise ValueError(f"{label} is missing joints: {missing}")
    non_finite = [joint for joint in joints if not math.isfinite(values[joint])]
    if non_finite:
        raise ValueError(f"{label} contains non-finite values for: {non_finite}")


def _validate_threshold(name: str, value: float) -> None:
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be finite and positive.")


def _pass_fail(value: bool) -> str:
    return "PASS" if value else "FAIL"
