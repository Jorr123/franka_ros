"""Binary policy events and bounded Franka Hand commands; ROS imports are lazy."""

from dataclasses import dataclass

from .protocol import ProposalError, number

OPEN_WIDTH_M = 0.08
OPEN_THRESHOLD = 0.3
CLOSE_THRESHOLD = 0.7


def select_gripper_prefix(signals, measured_width_m, *, grasp_confirmed=False):
    """Stop the prefix at the first open/close transition; dead band means hold."""
    width = number(measured_width_m, "gripper width")
    if not 0 <= width <= OPEN_WIDTH_M:
        raise ProposalError("INVALID_GRIPPER", "Measured width is outside the standard Hand travel")
    if not isinstance(signals, list) or not 1 <= len(signals) <= 32:
        raise ProposalError("INVALID_GRIPPER", "Expected 1..32 gripper predictions")
    current = "open" if width >= OPEN_WIDTH_M - 0.005 else "grasp" if grasp_confirmed else None
    for index, value in enumerate(signals):
        value = number(value, "policy gripper signal")
        desired = "open" if value <= OPEN_THRESHOLD else "grasp" if value >= CLOSE_THRESHOLD else current
        if desired != current:
            return index + 1, desired
    return len(signals), "hold"


def validate_gripper_command(proposal):
    command = proposal.get("gripper_command")
    if not isinstance(command, dict) or set(command) != {"operation", "policy_closed_fractions", "grasp_confirmed"}:
        raise ProposalError("INVALID_GRIPPER", "Missing gripper command/provenance")
    signals = command["policy_closed_fractions"]
    count = len(proposal.get("action_chunk", {}).get("joint_positions_rad", [None]))
    if not isinstance(signals, list) or len(signals) != count:
        raise ProposalError("INVALID_GRIPPER", "Gripper predictions must match selected arm actions")
    if type(command["grasp_confirmed"]) is not bool:
        raise ProposalError("INVALID_GRIPPER", "Grasp confirmation must be Boolean")
    prefix, operation = select_gripper_prefix(signals, proposal["basis_state"]["gripper_width_m"],
                                             grasp_confirmed=command["grasp_confirmed"])
    if prefix != count or command["operation"] != operation:
        raise ProposalError("INVALID_GRIPPER", "Chunk must end at the first gripper transition")
    return command


@dataclass(frozen=True)
class GripperSettings:
    grasp_width_m: float
    force_n: float
    speed_m_s: float = 0.02
    epsilon_inner_m: float = 0.005
    epsilon_outer_m: float = 0.005
    open_tolerance_m: float = 0.005
    timeout_seconds: float = 10.0

    def __post_init__(self):
        for name, value in vars(self).items():
            number(value, name)
        if not 0.005 < self.grasp_width_m < 0.075:
            raise ValueError("Specify a tested contact width between 0.005 and 0.075 m")
        if not 30 <= self.force_n <= 70:
            raise ValueError("Grasp force must be explicitly configured within the existing 30..70 N bounds")
        if not 0 < self.speed_m_s <= 0.02:
            raise ValueError("Gripper speed must be positive and at most 0.02 m/s")
        if not 0 <= self.epsilon_inner_m <= 0.005 or self.epsilon_inner_m >= self.grasp_width_m or not 0 <= self.epsilon_outer_m <= 0.005:
            raise ValueError("Invalid grasp epsilon")
        if not 0 < self.open_tolerance_m <= 0.005 or not 0 < self.timeout_seconds <= 30:
            raise ValueError("Invalid gripper measurement tolerance or timeout")

    def plan(self, operation):
        if operation not in {"open", "grasp", "hold"}:
            raise ValueError("Unsupported gripper operation")
        return {"operation": operation, "target_width_m": OPEN_WIDTH_M if operation == "open" else self.grasp_width_m if operation == "grasp" else None,
                "force_n": self.force_n if operation == "grasp" else None,
                "speed_m_s": self.speed_m_s, "epsilon_inner_m": self.epsilon_inner_m,
                "epsilon_outer_m": self.epsilon_outer_m, "timeout_seconds": self.timeout_seconds}


def verify_gripper_width(operation, width, config):
    width = number(width, "measured final gripper width")
    if not 0 <= width <= OPEN_WIDTH_M:
        raise RuntimeError("Final gripper width outside physical travel")
    if operation == "open" and abs(width - OPEN_WIDTH_M) > config.open_tolerance_m:
        raise RuntimeError("Gripper did not reach the configured open width")
    if operation == "grasp" and not config.grasp_width_m - config.epsilon_inner_m <= width <= config.grasp_width_m + config.epsilon_outer_m:
        raise RuntimeError("Grasp width is outside the configured object-contact interval")


def validate_gripper_receipt(proposal, receipt):
    operation = proposal["gripper_command"]["operation"]
    result = receipt.get("execution", {}).get("gripper", {})
    plan = receipt.get("plan", {}).get("gripper", {})
    if not isinstance(result, dict) or not isinstance(plan, dict) or result.get("status") != "PASSED" or result.get("operation") != operation or plan.get("operation") != operation:
        raise ValueError("Ubuntu did not confirm the requested gripper result")
    if operation == "hold":
        if result.get("command_sent") is not False:
            raise ValueError("Held gripper receipt unexpectedly claims a command")
        return
    if result.get("command_sent") is not True:
        raise ValueError("Missing gripper action completion")
    width = number(result.get("measured_width_m"), "measured gripper width")
    target = number(plan.get("target_width_m"), "planned gripper width")
    if not 0 <= width <= OPEN_WIDTH_M or result.get("target_width_m") != target:
        raise ValueError("Invalid gripper measurement or changed target")
    if operation == "open":
        if target != OPEN_WIDTH_M or abs(width - target) > 0.005:
            raise ValueError("Opening width was not verified")
    else:
        inner = number(plan.get("epsilon_inner_m"), "inner epsilon")
        outer = number(plan.get("epsilon_outer_m"), "outer epsilon")
        force = number(plan.get("force_n"), "grasp force")
        if not 0 < target < OPEN_WIDTH_M or not 0 <= inner <= 0.005 or not 0 <= outer <= 0.005 or not 30 <= force <= 70 or result.get("force_n") != force or not target - inner <= width <= target + outer:
            raise ValueError("Grasp contact width/force were not verified")


class FrankaPolicyGripper:
    def __init__(self, node, config, *, state_topic):
        from fr3_keyboard_joint_jog import KeyboardGripperClient
        self.config = config
        self.confirmed_width_m = None
        self.client = KeyboardGripperClient(node, state_topic=state_topic,
                                           move_action="/franka_gripper/move", grasp_action="/franka_gripper/grasp")

    def preflight(self):
        self.client.wait_for_servers(self.config.timeout_seconds)

    def grasp_confirmed_at(self, width):
        return self.confirmed_width_m is not None and abs(width - self.confirmed_width_m) <= 0.001

    def execute(self, operation):
        config = self.config
        if operation == "hold":
            return {"status": "PASSED", "operation": "hold", "command_sent": False}
        if operation == "open":
            snapshot = self.client.open(OPEN_WIDTH_M, config.speed_m_s, config.timeout_seconds)
        else:
            snapshot = self.client.grasp(config.grasp_width_m, config.speed_m_s, config.force_n,
                                         config.epsilon_inner_m, config.epsilon_outer_m, config.timeout_seconds)
        # Require another measurement after receiving the action result.
        final = self.client.wait_for_snapshot(config.timeout_seconds, after_sequence=snapshot.sequence)
        verify_gripper_width(operation, final.width, config)
        self.confirmed_width_m = final.width if operation == "grasp" else None
        return {"status": "PASSED", "operation": operation, "command_sent": True,
                "measured_width_m": final.width, "target_width_m": config.plan(operation)["target_width_m"],
                "force_n": config.force_n if operation == "grasp" else None,
                "task_success": "NOT_ASSESSED"}
