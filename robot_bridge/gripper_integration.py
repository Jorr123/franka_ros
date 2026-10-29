"""Sequential arm/gripper execution with fresh measurements and scene checks."""

import math
import time

from .protocol import JOINTS, state_reference
from .chunk_planning import check_collisions
from fr3_ai_supervisor import ExecutionRejected


def check_gripper_path(planner, proposal, arm_positions, *, before_width=None, after_arm=False):
    command = proposal["gripper_command"]["operation"]
    if command == "hold":
        return
    config = planner.gripper.config
    start = proposal["basis_state"]["gripper_width_m"] if before_width is None else before_width
    target = 0.08 if command == "open" else config.grasp_width_m - config.epsilon_inner_m
    count = max(1, math.ceil(abs(target - start) / 0.001))
    widths = [start + (target - start) * i / count for i in range(count + 1)]
    check_collisions(planner, [[arm_positions[j] for j in JOINTS]] * len(widths), proposal,
                     gripper_widths=widths,
                     deadline_override=time.time() + config.timeout_seconds if after_arm else None)


def execute_gripper(planner, context, proposal, arm_result):
    operation = proposal["gripper_command"]["operation"]
    if arm_result.get("status") not in {"VERIFIED_SUCCESS", "EXECUTED_WITH_WARNINGS"}:
        raise ExecutionRejected("GRIPPER_BLOCKED", "Arm execution did not complete; gripper was not commanded")
    if operation == "hold":
        return {**arm_result, "gripper": {"status": "PASSED", "operation": "hold", "command_sent": False}}
    command_started = False
    try:
        # The arm may run longer than the pre-execution observation deadline.
        # At this boundary validate NEW state against its measured final pose.
        def fresh_boundary():
            status, raw = planner.snapshot()
            if status != 200:
                raise RuntimeError("Fresh independent gripper state unavailable")
            live = state_reference(raw)
            if live["joint_state_publisher"] != proposal["basis_state"]["joint_state_publisher"]:
                raise RuntimeError("Robot state publisher changed")
            age = (time.time() * 1000 - live["joint_state_received_unix_ms"]) / 1000
            if not -planner.limits.max_future_skew_seconds <= age <= planner.limits.max_state_age_seconds:
                raise RuntimeError("Independent state is stale at the gripper boundary")
            if max(live["joint_state_age_seconds"], live["gripper_state_age_seconds"]) > planner.limits.max_state_age_seconds:
                raise RuntimeError("Joint/gripper measurements are stale")
            if abs(live["gripper_width_m"] - proposal["basis_state"]["gripper_width_m"]) > planner.limits.max_gripper_drift_m:
                raise RuntimeError("Gripper moved during arm execution")
            measured = planner.node.wait_for_joint_state_snapshot(
                timeout=planner.args.state_timeout, after_sequence=planner.node.joint_state_sequence,
                require_velocities=True)
            final = arm_result["verification"]["measured_joint_positions"]
            for i, joint in enumerate(JOINTS):
                if abs(measured.velocities[joint]) > planner.args.max_start_velocity:
                    raise RuntimeError("Arm must be settled before a gripper command")
                if abs(measured.positions[joint] - final[joint]) > planner.args.start_state_tolerance:
                    raise RuntimeError("Arm moved after final measurement")
                if abs(measured.positions[joint] - live["joint_positions_rad"][i]) > planner.limits.max_basis_drift_rad:
                    raise RuntimeError("Independent arm state disagrees")
            return live, measured

        live, measured = fresh_boundary()
        check_gripper_path(planner, proposal, measured.positions, before_width=live["gripper_width_m"], after_arm=True)
        fresh_boundary()  # Scene checking must not age the execution guard.
        command_started = True
        result = planner.gripper.execute(operation)
        status, raw = planner.snapshot()
        if status != 200:
            raise RuntimeError("Independent state unavailable after gripper command")
        independent = state_reference(raw)
        if independent["joint_state_publisher"] != proposal["basis_state"]["joint_state_publisher"]:
            raise RuntimeError("Robot state publisher changed after gripper command")
        age = (time.time() * 1000 - independent["joint_state_received_unix_ms"]) / 1000
        if not -planner.limits.max_future_skew_seconds <= age <= planner.limits.max_state_age_seconds or max(independent["joint_state_age_seconds"], independent["gripper_state_age_seconds"]) > planner.limits.max_state_age_seconds:
            raise RuntimeError("Independent state stale after gripper command")
        if abs(independent["gripper_width_m"] - result["measured_width_m"]) > planner.limits.max_gripper_drift_m:
            raise RuntimeError("Independent gripper measurement disagrees after command")
        settled = planner.node.wait_for_joint_state_snapshot(
            timeout=planner.args.state_timeout, after_sequence=planner.node.joint_state_sequence,
            require_velocities=True)
        for i, joint in enumerate(JOINTS):
            if abs(settled.velocities[joint]) > planner.args.max_start_velocity:
                raise RuntimeError("Arm is moving after gripper command")
            if abs(settled.positions[joint] - arm_result["verification"]["measured_joint_positions"][joint]) > planner.args.start_state_tolerance:
                raise RuntimeError("Arm drifted during gripper command")
            if abs(settled.positions[joint] - independent["joint_positions_rad"][i]) > planner.limits.max_basis_drift_rad:
                raise RuntimeError("Independent arm state disagrees after gripper command")
        result["post_gripper_joint_positions_rad"] = dict(settled.positions)
        return {**arm_result, "gripper": result}
    except (RuntimeError, ValueError, KeyError, TypeError) as error:
        raise ExecutionRejected("GRIPPER_FAILED", str(error), details={
            "arm_result": arm_result, "gripper": {"status": "FAILED", "operation": operation,
            "command_may_have_started": command_started, "reason": str(error)}}) from error
