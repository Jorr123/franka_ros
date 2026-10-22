"""IP-restricted dry-run proposals; deliberately distinct from execution messages."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import json
import math
import time
import uuid

MODE = "VALIDATION_ONLY"
PATH = "/v1/action-proposals/dry-run"
PLAN_MODE = "PLAN_ONLY"
PLAN_PATH = "/v1/action-proposals/plan"
EXECUTE_MODE = "EXECUTE_ONCE"
EXECUTE_PATH = "/v1/action-proposals/execute-once"
MAX_BODY_BYTES = 64 * 1024
JOINTS = [f"fr3_joint{index}" for index in range(1, 8)]
STATE_FIELDS = (
    "joint_names", "joint_positions_rad", "joint_state_publisher", "joint_state_stamp_ns",
    "joint_state_sequence", "joint_state_received_unix_ms", "joint_state_age_seconds",
    "gripper_width_m", "gripper_state_age_seconds",
)


class ProposalError(ValueError):
    def __init__(self, code: str, message: str):
        self.code = code
        super().__init__(message)


def number(value, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ProposalError("INVALID_NUMBER", f"{field} must be a finite number")
    try:
        converted = float(value)
    except OverflowError as error:
        raise ProposalError("INVALID_NUMBER", f"{field} must be finite") from error
    if not math.isfinite(converted):
        raise ProposalError("INVALID_NUMBER", f"{field} must be finite")
    return converted


def canonical_json(value) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")


def digest(body: bytes) -> str:
    return hashlib.sha256(body).hexdigest()


def decode_json(body: bytes):
    def unique_object(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError(f"Duplicate JSON key: {key}")
            result[key] = value
        return result

    def invalid_constant(value):
        raise ValueError(f"Non-finite JSON number: {value}")

    try:
        return json.loads(body.decode("utf-8"), object_pairs_hook=unique_object, parse_constant=invalid_constant)
    except (ValueError, UnicodeError, RecursionError) as error:
        raise ProposalError("INVALID_JSON", "Expected finite JSON with unique keys") from error


def positive_int(value, field: str) -> None:
    if type(value) is not int or value <= 0:
        raise ProposalError("INVALID_PROPOSAL", f"{field} must be a positive integer")


def joint_vector(values, field: str) -> list[float]:
    if not isinstance(values, list) or len(values) != 7:
        raise ProposalError("INVALID_JOINTS", f"{field} must contain seven positions in joint order 1-7")
    return [number(value, field) for value in values]


def state_reference(state: dict) -> dict:
    """Extract raw state provenance without depending on a camera or VLA package."""
    if not isinstance(state, dict) or not set(STATE_FIELDS).issubset(state):
        raise ProposalError("INVALID_STATE", "Incomplete measured state reference")
    result = {field: state[field] for field in STATE_FIELDS}
    # Include transport delay when called with the Windows HTTP client's result.
    for channel in ("joint", "gripper"):
        effective = f"effective_{channel}_state_age_seconds"
        if effective in state:
            result[f"{channel}_state_age_seconds"] = state[effective]
    if result["joint_names"] != JOINTS:
        raise ProposalError("INVALID_JOINTS", "Expected named FR3 joints in order 1-7")
    joint_vector(result["joint_positions_rad"], "joint_positions_rad")
    for field in ("joint_state_stamp_ns", "joint_state_sequence", "joint_state_received_unix_ms"):
        positive_int(result[field], field)
    for field in ("joint_state_age_seconds", "gripper_width_m", "gripper_state_age_seconds"):
        if number(result[field], field) < 0.0:
            raise ProposalError("INVALID_STATE", f"{field} must be nonnegative")
    publisher = result["joint_state_publisher"]
    if not isinstance(publisher, dict) or set(publisher) != {"node_name", "node_namespace", "gid"}:
        raise ProposalError("INVALID_SOURCE", "Missing joint-state publisher identity")
    if not all(isinstance(value, str) and value for value in publisher.values()):
        raise ProposalError("INVALID_SOURCE", "Missing joint-state publisher identity")
    try:
        gid = bytes.fromhex(publisher["gid"])
    except ValueError as error:
        raise ProposalError("INVALID_SOURCE", "Invalid joint-state publisher GID") from error
    if not gid or not any(gid):
        raise ProposalError("INVALID_SOURCE", "Invalid joint-state publisher GID")
    return result


@dataclass(frozen=True)
class Limits:
    max_joint_delta_rad: float = 0.1
    max_total_delta_rad: float = 0.4
    max_start_drift_rad: float = 0.005
    max_basis_drift_rad: float = 0.002
    max_gripper_drift_m: float = 0.001
    max_state_age_seconds: float = 1.0
    max_observation_age_seconds: float = 30.0
    max_request_age_seconds: float = 5.0
    max_future_skew_seconds: float = 1.0

    def __post_init__(self):
        for field, value in asdict(self).items():
            if number(value, field) <= 0.0:
                raise ValueError(f"{field} must be positive")


def make_proposal(
    *, source: str, target_joint_positions_rad: list[float], observation_state: dict,
    basis_state: dict, observation_created_at_unix_ms: int,
    inference_completed_at_unix_ms: int, action_index: int = 0, mode: str = MODE,
) -> dict:
    observed = state_reference(observation_state)
    basis = state_reference(basis_state)
    target = joint_vector(target_joint_positions_rad, "target_joint_positions_rad")
    proposal = {
        "schema_version": 1, "mode": mode, "execution_allowed": mode == EXECUTE_MODE,
        "request_id": str(uuid.uuid4()), "created_at_unix_ms": int(time.time() * 1000),
        "source": source,
        "prediction": {
            "observation_created_at_unix_ms": observation_created_at_unix_ms,
            "inference_completed_at_unix_ms": inference_completed_at_unix_ms,
            "action_index": action_index,
        },
        "observation_state": observed, "basis_state": basis,
        "target_joint_positions_rad": target,
        "joint_deltas_rad": {joint: target[index] - basis["joint_positions_rad"][index]
                             for index, joint in enumerate(JOINTS)},
        "gripper_mode": "hold",
    }
    validate_shape(proposal, expected_mode=mode)
    return proposal


def validate_shape(proposal: dict, *, expected_mode: str = MODE) -> None:
    keys = {
        "schema_version", "mode", "execution_allowed", "request_id", "created_at_unix_ms",
        "source", "prediction", "observation_state", "basis_state",
        "target_joint_positions_rad", "joint_deltas_rad", "gripper_mode",
    }
    if not isinstance(proposal, dict) or set(proposal) != keys:
        raise ProposalError("INVALID_PROPOSAL", "Unexpected dry-run proposal fields")
    if type(proposal["schema_version"]) is not int or proposal["schema_version"] != 1:
        raise ProposalError("INVALID_PROPOSAL", "Unsupported dry-run schema")
    if expected_mode not in {MODE, PLAN_MODE, EXECUTE_MODE} or proposal["mode"] != expected_mode or proposal["execution_allowed"] is not (expected_mode == EXECUTE_MODE) or proposal["gripper_mode"] != "hold":
        raise ProposalError("EXECUTION_FORBIDDEN", "This endpoint only accepts validation-only arm proposals with gripper held")
    try:
        request_id = uuid.UUID(proposal["request_id"])
        if request_id.version != 4 or str(request_id) != proposal["request_id"]:
            raise ValueError()
    except (ValueError, TypeError, AttributeError) as error:
        raise ProposalError("INVALID_PROPOSAL", "request_id must be a canonical UUIDv4") from error
    if not isinstance(proposal["source"], str) or not 1 <= len(proposal["source"]) <= 128:
        raise ProposalError("INVALID_SOURCE", "source must identify the model adapter")
    positive_int(proposal["created_at_unix_ms"], "created_at_unix_ms")
    prediction = proposal["prediction"]
    if not isinstance(prediction, dict) or set(prediction) != {
        "observation_created_at_unix_ms", "inference_completed_at_unix_ms", "action_index"
    }:
        raise ProposalError("INVALID_PROPOSAL", "Invalid prediction provenance")
    for field in ("observation_created_at_unix_ms", "inference_completed_at_unix_ms"):
        positive_int(prediction[field], field)
    if type(prediction["action_index"]) is not int or prediction["action_index"] < 0:
        raise ProposalError("INVALID_PROPOSAL", "action_index must be nonnegative")
    state_reference(proposal["observation_state"])
    basis = state_reference(proposal["basis_state"])
    target = joint_vector(proposal["target_joint_positions_rad"], "target_joint_positions_rad")
    deltas = proposal["joint_deltas_rad"]
    if not isinstance(deltas, dict) or set(deltas) != set(JOINTS):
        raise ProposalError("INVALID_JOINTS", "Deltas must name exactly FR3 joints 1-7")
    for index, joint in enumerate(JOINTS):
        delta = number(deltas[joint], joint)
        if abs(delta - (target[index] - basis["joint_positions_rad"][index])) > 1e-9:
            raise ProposalError("DELTA_MISMATCH", f"{joint} delta does not equal target minus measured basis")


def evaluate_proposal(proposal: dict, current_state: dict, limits: Limits, now_unix_ms: int, *, mode: str = MODE) -> dict:
    """Validate against Ubuntu's own measurements; never plan or execute anything."""
    validate_shape(proposal, expected_mode=mode)
    observed = state_reference(proposal["observation_state"])
    basis = state_reference(proposal["basis_state"])
    current = state_reference(current_state)
    prediction = proposal["prediction"]
    future_ms = limits.max_future_skew_seconds * 1000
    for field, timestamp, maximum in (
        ("request", proposal["created_at_unix_ms"], limits.max_request_age_seconds),
        ("observation", prediction["observation_created_at_unix_ms"], limits.max_observation_age_seconds),
        ("prediction", prediction["inference_completed_at_unix_ms"], limits.max_request_age_seconds),
    ):
        age = now_unix_ms - timestamp
        if age < -future_ms or age > maximum * 1000:
            raise ProposalError("STALE_" + field.upper(), f"{field} age {age / 1000:.3f}s is outside the permitted window")
    if not prediction["observation_created_at_unix_ms"] <= prediction["inference_completed_at_unix_ms"] <= proposal["created_at_unix_ms"]:
        raise ProposalError("INVALID_TIMELINE", "Expected observation, inference completion, then proposal creation")
    for state, maximum in ((observed, limits.max_observation_age_seconds), (basis, limits.max_state_age_seconds), (current, limits.max_state_age_seconds)):
        received_age = (now_unix_ms - state["joint_state_received_unix_ms"]) / 1000
        if received_age < -limits.max_future_skew_seconds or received_age > maximum:
            raise ProposalError("STALE_STATE", "Measured state receipt time is stale or in the future")
        for field in ("joint_state_age_seconds", "gripper_state_age_seconds"):
            # The basis ages were measured before the HTTP proposal was sent.
            # Add a conservative delay bound from its Ubuntu receipt timestamp.
            age = state[field] + (max(0.0, received_age) if state is basis else 0.0)
            if age > limits.max_state_age_seconds:
                raise ProposalError("STALE_STATE", f"{field} exceeds the freshness limit")
    if not observed["joint_state_publisher"] == basis["joint_state_publisher"] == current["joint_state_publisher"]:
        raise ProposalError("SOURCE_CHANGED", "Joint-state publisher changed since observation")
    if basis["joint_state_stamp_ns"] < observed["joint_state_stamp_ns"]:
        raise ProposalError("STATE_REWOUND", "Post-inference state precedes the observation")
    drift = max(abs(a - b) for a, b in zip(observed["joint_positions_rad"], current["joint_positions_rad"]))
    basis_drift = max(abs(a - b) for a, b in zip(basis["joint_positions_rad"], current["joint_positions_rad"]))
    grip_drift = abs(observed["gripper_width_m"] - current["gripper_width_m"])
    if drift > limits.max_start_drift_rad or grip_drift > limits.max_gripper_drift_m:
        raise ProposalError("START_STATE_CHANGED", f"Robot moved since observation: {drift:.6f} rad, {grip_drift:.6f} m")
    if basis_drift > limits.max_basis_drift_rad:
        raise ProposalError("BASIS_STATE_CHANGED", f"Robot moved since proposal basis: {basis_drift:.6f} rad")
    target = proposal["target_joint_positions_rad"]
    current_deltas = [a - b for a, b in zip(target, current["joint_positions_rad"])]
    declared = list(proposal["joint_deltas_rad"].values())
    max_delta = max(abs(value) for value in current_deltas + declared)
    total = max(sum(abs(value) for value in current_deltas), sum(abs(value) for value in declared))
    if max_delta > limits.max_joint_delta_rad or total > limits.max_total_delta_rad:
        raise ProposalError("DELTA_LIMIT_EXCEEDED", f"Proposed maximum delta {max_delta:.6f} rad / L1 total {total:.6f} rad exceeds limits")
    return {
        "max_joint_delta_rad": max_delta, "total_joint_delta_rad": total,
        "max_start_drift_rad": drift, "max_basis_drift_rad": basis_drift,
        "gripper_drift_m": grip_drift,
        "observation_age_seconds": (now_unix_ms - prediction["observation_created_at_unix_ms"]) / 1000,
    }
