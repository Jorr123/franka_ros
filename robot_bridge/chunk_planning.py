"""Ubuntu adapter: dense joint chunks checked against the MoveIt scene."""

from copy import deepcopy
from pathlib import Path
import time

from .chunks import timed_chunk
from .protocol import JOINTS, canonical_json, digest, number
from fr3_ai_supervisor import ExecutionContext, PlanInputRejected, PreparedPlan


def check_collisions(planner, positions, proposal, *, gripper_widths=None, deadline_override=None):
    from moveit_msgs.srv import GetStateValidity

    client = getattr(planner, "chunk_validity_client", None)
    if client is None:
        client = planner.node.create_client(GetStateValidity, "/check_state_validity")
        planner.chunk_validity_client = client
    deadline = min(time.time() + planner.args.result_timeout,
                   proposal["prediction"]["observation_created_at_unix_ms"] / 1000 + planner.limits.max_observation_age_seconds)
    if deadline_override is not None:
        deadline = min(time.time() + planner.args.result_timeout, deadline_override)
    if not client.wait_for_service(timeout_sec=min(planner.args.server_timeout, max(0.0, deadline - time.time()))):
        raise PlanInputRejected("MoveIt /check_state_validity service unavailable")
    for index, q in enumerate(positions):
        remaining = deadline - time.time()
        if remaining <= 0:
            raise PlanInputRejected("Chunk collision validation exceeded the original observation deadline")
        request = GetStateValidity.Request()
        request.group_name = planner.args.group if gripper_widths is None else ""
        request.robot_state.is_diff = True
        request.robot_state.joint_state.name = list(JOINTS)
        request.robot_state.joint_state.position = list(q)
        if gripper_widths is not None:
            request.robot_state.joint_state.name += ["fr3_finger_joint1", "fr3_finger_joint2"]
            request.robot_state.joint_state.position += [gripper_widths[index] / 2] * 2
        future = client.call_async(request)
        planner.rclpy.spin_until_future_complete(planner.node, future, timeout_sec=min(remaining, planner.args.server_timeout))
        if not future.done():
            future.cancel()
            raise PlanInputRejected("Chunk collision validation timed out")
        response = future.result()
        if response is None or not response.valid:
            raise PlanInputRejected(f"MoveIt rejected chunk collision/bounds sample {index}")


def prepare_chunk(planner, envelope):
    from builtin_interfaces.msg import Duration
    from moveit_msgs.msg import RobotTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint
    import yaml
    from .planning import trajectory_payload

    proposal = envelope["cosmos_proposal"]
    chunk = proposal["action_chunk"]
    try:
        snapshot = planner.node.wait_for_joint_state_snapshot(
            timeout=planner.args.state_timeout, after_sequence=planner.node.joint_state_sequence,
            require_velocities=True)
        current = dict(snapshot.positions)
        deltas = dict(envelope["action"]["deltas"])
        target = planner.resolve_target(envelope, current, deltas)
        config = Path(__file__).resolve().parents[1] / "src/franka_fr3_moveit_config/config/joint_limits.yaml"
        joint_limits = yaml.safe_load(config.read_text())["joint_limits"]
        velocity, acceleration = [], []
        for joint in JOINTS:
            limits = joint_limits[joint]
            if limits.get("has_velocity_limits") is not True or limits.get("has_acceleration_limits") is not True:
                raise ValueError(f"Explicit motion limits missing for {joint}")
            velocity.append(number(limits["max_velocity"], "max_velocity") * planner.args.velocity_scaling)
            acceleration.append(number(limits["max_acceleration"], "max_acceleration") * planner.args.acceleration_scaling)
        samples, metadata = timed_chunk([current[j] for j in JOINTS], chunk["joint_positions_rad"],
                                        chunk["action_period_seconds"], velocity, acceleration)
        if samples[-1][0] + planner.args.settle_time >= planner.args.result_timeout:
            raise ValueError("Chunk duration exceeds execution timeout; request fewer actions")
        trajectory = RobotTrajectory()
        trajectory.joint_trajectory.joint_names = list(JOINTS)
        for seconds, positions, velocities, accelerations in samples:
            planner.validate_target_limits(dict(zip(JOINTS, positions)), planner.args)
            point = JointTrajectoryPoint()
            point.positions, point.velocities, point.accelerations = positions, velocities, accelerations
            ns = round(seconds * 1e9)
            point.time_from_start = Duration(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)
            trajectory.joint_trajectory.points.append(point)
        reviewed = trajectory_payload(trajectory.joint_trajectory, current, target, planner.limits, planner.args.goal_tolerance)
        check_collisions(planner, [p[1] for p in samples], proposal)
    except (RuntimeError, ValueError, KeyError, OSError) as error:
        raise PlanInputRejected(str(error)) from error
    action_digest = digest(canonical_json({"action": envelope["action"], "chunk": chunk}))
    payload = {
        "schema_version": 2, "request_id": envelope["request_id"], "status": "PLAN_READY",
        "plan_only": True, "execution_available": False, "plan_request_executed": False,
        "remote_execution_available": False, "action_sha256": action_digest,
        "action_label": "joint_chunk", "validated_deltas": deltas,
        "current_joint_positions": current, "target_joint_positions": target,
        "chunk": metadata, "trajectory": reviewed,
        "planning": {"method": "DENSE_MOVEIT_STATE_VALIDITY", "result": "SUCCESS", "error_code": 1,
                     "point_count": len(samples), "duration_seconds": samples[-1][0], "joints": list(JOINTS)},
    }
    context = ExecutionContext(trajectory, current, target, {"cosmos_proposal": deepcopy(proposal)}) if planner.args.enable_local_execution else None
    return PreparedPlan(200, payload, context)
