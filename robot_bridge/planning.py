"""Validated absolute targets to the existing MoveIt/local-approval supervisor.

Imports of ROS remain inside RosMoveItPlanner initialization. HTTP plans by
default. ExecuteOnceServer explicitly adds one automatic execution attempt;
the separate local-approval mode uses the existing mode-0600 Ubuntu socket.
"""

from __future__ import annotations

from copy import deepcopy
from pathlib import Path
import sys
import threading
import time
import uuid

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from fr3_ai_supervisor import (
    ExecutionRejected, PlanInputRejected, RequestFailure, RosMoveItPlanner,
    SupervisorCore, validate_joint_delta_action_contract,
)
from .protocol import (
    JOINTS, PLAN_MODE, PLAN_PATH, EXECUTE_MODE, EXECUTE_PATH, Limits, ProposalError, canonical_json,
    evaluate_proposal, joint_vector, number,
)
from .server import DryRunHandler, DryRunServer

STATUS_PATH = "/v1/action-proposals/status/"
MAX_PLAN_RESPONSE_BYTES = 2 * 1024 * 1024


def check_live_state(proposal, snapshot, limits, *, at_submission=True, mode=PLAN_MODE):
    status, state = snapshot()
    if status != 200:
        raise ProposalError("STATE_UNAVAILABLE", str(state.get("reason", "Live state unavailable")))
    if at_submission:
        return evaluate_proposal(proposal, state, limits, int(time.time() * 1000), mode=mode)

    # At approval, request/basis timestamps describe the original submission;
    # they must never be refreshed. Check a current independent ROS sample,
    # original observation deadline, source, pose, gripper and target bounds.
    now = int(time.time() * 1000)
    from .protocol import state_reference

    live = state_reference(state)
    observed = proposal["observation_state"]
    age = (now - proposal["prediction"]["observation_created_at_unix_ms"]) / 1000
    if not 0 <= age <= limits.max_observation_age_seconds:
        raise ProposalError("STALE_OBSERVATION", "Original observation expired; obtain a new inference")
    if live["joint_state_publisher"] != observed["joint_state_publisher"]:
        raise ProposalError("SOURCE_CHANGED", "ROS publisher changed since inference")
    receipt_age = (now - live["joint_state_received_unix_ms"]) / 1000
    if not -limits.max_future_skew_seconds <= receipt_age <= limits.max_state_age_seconds:
        raise ProposalError("STALE_STATE", "Ubuntu state receipt time is stale")
    for field in ("joint_state_age_seconds", "gripper_state_age_seconds"):
        if live[field] > limits.max_state_age_seconds:
            raise ProposalError("STALE_STATE", f"{field} is stale")
    drift = max(abs(a - b) for a, b in zip(live["joint_positions_rad"], observed["joint_positions_rad"]))
    if drift > limits.max_start_drift_rad or abs(live["gripper_width_m"] - observed["gripper_width_m"]) > limits.max_gripper_drift_m:
        raise ProposalError("START_STATE_CHANGED", "Arm or gripper changed since inference")
    deltas = [a - b for a, b in zip(proposal["target_joint_positions_rad"], live["joint_positions_rad"])]
    if max(abs(v) for v in deltas) > limits.max_joint_delta_rad or sum(abs(v) for v in deltas) > limits.max_total_delta_rad:
        raise ProposalError("DELTA_LIMIT_EXCEEDED", "Target exceeds movement bounds from current joints")
    return live


def trajectory_payload(trajectory, current, target, limits, tolerance):
    """Check every waypoint, joint order, timestamps and final absolute target."""
    if list(trajectory.joint_names) != JOINTS or not 1 <= len(trajectory.points) <= 2048:
        raise PlanInputRejected("MoveIt trajectory must contain ordered FR3 joints 1-7")
    points = []
    previous_time = -1.0
    for point in trajectory.points:
        positions = joint_vector(list(point.positions), "trajectory positions")
        seconds = number(point.time_from_start.sec, "time.sec") + number(point.time_from_start.nanosec, "time.nanosec") * 1e-9
        if seconds < 0 or seconds <= previous_time:
            raise PlanInputRejected("MoveIt trajectory timestamps must increase")
        previous_time = seconds
        change = [abs(positions[index] - current[joint]) for index, joint in enumerate(JOINTS)]
        if max(change) > limits.max_joint_delta_rad + 1e-9 or sum(change) > limits.max_total_delta_rad + 1e-9:
            raise PlanInputRejected("MoveIt trajectory leaves the bounded joint-motion region")
        entry = {"positions_rad": positions, "time_from_start_seconds": seconds}
        for field in ("velocities", "accelerations"):
            values = list(getattr(point, field, []))
            if values:
                entry[field] = joint_vector(values, "trajectory " + field)
        points.append(entry)
    if max(abs(points[0]["positions_rad"][i] - current[joint]) for i, joint in enumerate(JOINTS)) > tolerance:
        raise PlanInputRejected("Trajectory does not start at the frozen start pose")
    if max(abs(points[-1]["positions_rad"][i] - target[joint]) for i, joint in enumerate(JOINTS)) > tolerance:
        raise PlanInputRejected("Trajectory does not reach the unchanged Cosmos target")
    result = {"joint_names": list(JOINTS), "points": points}
    if len(canonical_json(result)) > MAX_PLAN_RESPONSE_BYTES - 65536:
        raise PlanInputRejected("MoveIt trajectory is too large to return for review")
    return result


class CosmosMoveItPlanner(RosMoveItPlanner):
    def __init__(self, args, *, snapshot, limits):
        self.snapshot = snapshot
        self.limits = limits
        super().__init__(args, action_validator=validate_joint_delta_action_contract)

    def plan(self, envelope):
        prepared = super().plan(envelope)
        if prepared.execution_context is not None:
            prepared.execution_context.metadata = {"cosmos_proposal": deepcopy(envelope["cosmos_proposal"])}
        return prepared

    def resolve_target(self, envelope, current, deltas):
        proposal = envelope["cosmos_proposal"]
        try:
            live = check_live_state(proposal, self.snapshot, self.limits, at_submission=False)
            if max(abs(current[j] - live["joint_positions_rad"][i]) for i, j in enumerate(JOINTS)) > self.limits.max_basis_drift_rad:
                raise ProposalError("STATE_DISAGREEMENT", "MoveIt and independent ROS state disagree")
            target = dict(zip(JOINTS, proposal["target_joint_positions_rad"]))
            deltas.clear()
            deltas.update({j: target[j] - current[j] for j in JOINTS})
            self.validate_deltas(deltas, self.args)
            return target
        except ProposalError as error:
            raise PlanInputRejected(f"{error.code}: {error}") from error

    def inspect_trajectory(self, trajectory, current, target):
        return {"trajectory": trajectory_payload(trajectory, current, target, self.limits, self.args.goal_tolerance)}

    def before_execute(self, context):
        try:
            proposal = context.metadata["cosmos_proposal"]
            sequence = self.node.joint_state_sequence
            measured = self.node.wait_for_joint_state_snapshot(
                timeout=self.args.state_timeout, after_sequence=sequence, require_velocities=True)
            live = check_live_state(proposal, self.snapshot, self.limits, at_submission=False)
            for index, joint in enumerate(JOINTS):
                if abs(measured.positions[joint] - live["joint_positions_rad"][index]) > self.limits.max_basis_drift_rad:
                    raise ProposalError("STATE_DISAGREEMENT", "MoveIt and independent ROS state disagree at execution")
                if abs(measured.positions[joint] - context.start_positions[joint]) > self.args.start_state_tolerance:
                    raise ProposalError("START_STATE_CHANGED", "Robot moved since the frozen plan")
                if abs(measured.velocities[joint]) > self.args.max_start_velocity:
                    raise ProposalError("ROBOT_NOT_SETTLED", "Robot is moving at execution")
        except (ProposalError, RuntimeError, KeyError, TypeError) as error:
            raise ExecutionRejected(getattr(error, "code", "EXECUTION_REVALIDATION_FAILED"), str(error)) from error


class CosmosSupervisor(SupervisorCore):
    def __init__(self, planner, *, snapshot, limits=None, execution_enabled=False, plan_ttl_seconds=30.0, audit=None):
        self.snapshot = snapshot
        self.limits = limits or Limits()
        self.audit = audit or (lambda event: None)
        self.proposals = {}
        # No legacy HTTP server is exposed and no network credential is used.
        super().__init__(planner, b"", max_request_age_ms=5000, max_future_skew_ms=1000,
                         replay_retention_seconds=300, execution_enabled=execution_enabled,
                         plan_ttl_seconds=plan_ttl_seconds)

    def prepare_proposal(self, proposal, *, mode=PLAN_MODE):
        selected = deepcopy(proposal)
        validation = check_live_state(selected, self.snapshot, self.limits, mode=mode)
        state_status, state = self.snapshot()
        if state_status != 200:
            raise ProposalError("STATE_UNAVAILABLE", "Live state unavailable")
        action = {
            "action": "joint_deltas",
            "deltas": {j: selected["target_joint_positions_rad"][i] - state["joint_positions_rad"][i] for i, j in enumerate(JOINTS)},
            # Required legacy schema field, not a model-provided confidence.
            "confidence": 1.0,
        }
        envelope = {"request_id": selected["request_id"], "action": action, "cosmos_proposal": selected}
        try:
            status, response = self.prepare_validated_plan(envelope)
        except RequestFailure as error:
            raise ProposalError(error.code, str(error)) from error
        if status != 200:
            return status, {"decision": "REJECTED", "code": "PLAN_REJECTED",
                            "reason": "MoveIt rejected the requested target", "plan": response}
        if any(abs(response["target_joint_positions"][joint] - selected["target_joint_positions_rad"][i]) > 1e-9
               for i, joint in enumerate(JOINTS)):
            raise ProposalError("TARGET_CHANGED", "Planner changed the absolute Cosmos target")
        return 200, {"decision": "PLAN_READY", "code": "PLAN_READY", "validation": validation,
                     "reason": "MoveIt planned the target; no execution requested", "plan": response}

    def on_store_plan(self, record, envelope):
        proposal = deepcopy(envelope["cosmos_proposal"])
        if any(abs(record.execution_context.target_positions[joint] - proposal["target_joint_positions_rad"][i]) > 1e-9
               for i, joint in enumerate(JOINTS)):
            raise ProposalError("TARGET_CHANGED", "Cannot freeze a changed Cosmos target")
        remaining = (proposal["prediction"]["observation_created_at_unix_ms"] / 1000
                     + self.limits.max_observation_age_seconds - time.time())
        if remaining <= 0:
            raise ProposalError("STALE_OBSERVATION", "Observation expired during planning")
        record.expires_monotonic = min(record.expires_monotonic, time.monotonic() + remaining)
        record.expires_unix_ms = min(record.expires_unix_ms, int((time.time() + remaining) * 1000))
        self.proposals = {key: value for key, value in self.proposals.items() if key in self.records}
        self.proposals[record.request_id] = proposal

    def execute_context(self, context, request_id):
        try:
            proposal = self.proposals[request_id]
            check_live_state(proposal, self.snapshot, self.limits, at_submission=False)
            self.audit({"event": "LOCAL_EXECUTION_REQUESTED", "request_id": request_id})
        except (ProposalError, OSError, KeyError) as error:
            raise ExecutionRejected(getattr(error, "code", "APPROVAL_GUARD_FAILED"), str(error)) from error
        try:
            result = self.planner.execute(context)
        except ExecutionRejected as error:
            try:
                self.audit({"event": "LOCAL_EXECUTION_RESULT", "request_id": request_id,
                            "result": {"status": "FAULT", "code": error.code, "reason": str(error), **error.details}})
            except OSError:
                pass
            raise
        try:
            self.audit({"event": "LOCAL_EXECUTION_RESULT", "request_id": request_id, "result": result})
        except OSError:
            result = {**result, "audit_error": "Execution completed but its audit record could not be written"}
        return result


class PlanHandler(DryRunHandler):
    def do_GET(self):
        if not self.path.startswith(STATUS_PATH):
            return super().do_GET()
        if not self._allowed():
            return
        request_id = self.path[len(STATUS_PATH):]
        try:
            if str(uuid.UUID(request_id)) != request_id:
                raise ValueError()
            status, result = self.server.core.handle_local({"operation": "status", "request_id": request_id})
        except ValueError:
            self._send(400, {"code": "INVALID_REQUEST_ID", "mode": PLAN_MODE})
            return
        except RequestFailure as error:
            self._send(error.status, {"code": error.code, "reason": str(error), "mode": PLAN_MODE})
            return
        self._send(status, {"mode": PLAN_MODE, "remote_execution_available": False, **result})


class PlanServer(DryRunServer):
    mode = PLAN_MODE
    proposal_path = PLAN_PATH
    handler_class = PlanHandler

    def __init__(self, address, *, core, **kwargs):
        self.core = core
        super().__init__(address, snapshot=core.snapshot, limits=core.limits, **kwargs)
        core.audit = self.record_event

    def health(self):
        return {**super().health(), "local_approval_available": self.core.execution_enabled,
                "remote_execution_available": False}

    def record_event(self, event):
        self.record(None, {"created_at_unix_ms": int(time.time() * 1000), **event})

    def process_proposal(self, proposal, state):
        return self.core.prepare_proposal(proposal)

    def discard_plan(self, proposal):
        if not isinstance(proposal, dict):
            return
        with self.core.record_lock:
            record = self.core.records.get(proposal.get("request_id"))
            if record is not None and record.status == "READY_FOR_LOCAL_APPROVAL":
                record.status = "FAULT"
                record.result = {"status": "FAULT", "code": "AUDIT_UNAVAILABLE"}


class ExecuteOnceServer(PlanServer):
    """Opt-in automatic execution: one attempt per server lifetime, then disarm."""

    def __init__(self, address, *, local_approval_enabled=False, **kwargs):
        self.auto_execution_armed = True
        self.local_approval_enabled = local_approval_enabled
        self._step_lock = threading.Lock()
        super().__init__(address, **kwargs)
        if not self.core.execution_enabled:
            raise ValueError("Automatic execution requires the execution backend")

    def request_mode(self, path):
        if path == EXECUTE_PATH:
            return EXECUTE_MODE
        return super().request_mode(path)

    def health(self):
        return {**super().health(), "mode": EXECUTE_MODE, "auto_execution_armed": self.auto_execution_armed,
                "execution_allowed": self.auto_execution_armed, "remote_execution_available": self.auto_execution_armed,
                "local_approval_available": self.local_approval_enabled,
                "execute_once_endpoint": EXECUTE_PATH}

    def process_proposal(self, proposal, state):
        if not self._step_lock.acquire(blocking=False):
            raise ProposalError("SUPERVISOR_BUSY", "Another planning or execution request is in progress")
        try:
            if proposal["mode"] != EXECUTE_MODE:
                status, receipt = super().process_proposal(proposal, state)
                if "plan" in receipt:
                    receipt["plan"]["local_approval_available"] = self.local_approval_enabled
                return status, receipt
            if not self.auto_execution_armed:
                raise ProposalError("AUTO_EXECUTION_DISARMED", "One execution attempt was consumed; restart with the startup flag to rearm")
            status, receipt = self.core.prepare_proposal(proposal, mode=EXECUTE_MODE)
            if status != 200:
                return status, receipt
            # Consume permission before recording/sending an execution request.
            # A guard failure, timeout or verification fault cannot rearm it.
            self.auto_execution_armed = False
            receipt["plan"]["local_approval_available"] = False
            self.record(proposal, {"event": "AUTO_EXECUTION_PLAN_READY", **receipt})
            try:
                result_status, result = self.core.handle_local({
                    "operation": "execute", "request_id": proposal["request_id"],
                    "action_sha256": receipt["plan"]["action_sha256"],
                })
            except RequestFailure as error:
                return error.status, {**receipt, "decision": "REJECTED", "code": error.code,
                                      "reason": str(error), "auto_execution_armed": False}
            except Exception as error:
                return 503, {**receipt, "decision": "EXECUTION_FAILED", "code": "EXECUTION_OUTCOME_UNKNOWN",
                             "reason": f"Execution outcome unknown ({type(error).__name__}); do not retry automatically",
                             "execution_attempted": True, "robot_execution": None, "auto_execution_armed": False}
            success = result.get("status") == "VERIFIED_SUCCESS"
            warning = result.get("status") == "EXECUTED_WITH_WARNINGS"
            completed = success or warning
            reason = ("One movement executed and measured verification passed" if success
                      else "Controller execution completed; final accuracy verification failed (report-only)" if warning
                      else "Execution failed or could not be verified; do not retry automatically")
            return (200 if completed else 500), {
                **receipt, "decision": result["status"] if completed else "EXECUTION_FAILED",
                "code": result.get("code", result.get("status", "EXECUTION_FAILED")),
                "reason": reason,
                "execution": result, "execution_attempted": True,
                "robot_execution": True if completed else None, "auto_execution_armed": False,
            }
        finally:
            self._step_lock.release()
