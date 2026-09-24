#!/usr/bin/env python3
"""Plan, optionally confirm, execute, and verify one FR3 joint jog at a time."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
import os
import select
import sys
import termios
import time
import tty
from typing import Mapping

from franka_msgs.action import Grasp, Move
import rclpy
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from moveit_msgs.msg import MoveItErrorCodes

from moveit_joint_goal_test import (
    JOINTS,
    MoveItJointGoalClient,
    add_post_execution_verification_arguments,
    error_name,
    format_joint_table,
    make_plan_request,
    run_post_execution_verification,
    trajectory_summary,
    validate_post_execution_verification_args,
)
from vla_joint_command_test import JOINT_LIMITS


STEP_SIZES = {
    "fine": 0.010,
    "normal": 0.030,
    "coarse": 0.060,
}

FINGER_JOINTS = ("fr3_finger_joint1", "fr3_finger_joint2")


@dataclass(frozen=True)
class KeyDecision:
    kind: str
    value: object | None = None


def interpret_key(key: str) -> KeyDecision:
    if key in "1234567":
        return KeyDecision("select_joint", int(key) - 1)
    if key == "LEFT":
        return KeyDecision("move", -1.0)
    if key == "RIGHT":
        return KeyDecision("move", 1.0)
    if key.lower() == "f":
        return KeyDecision("set_mode", "fine")
    if key.lower() == "n":
        return KeyDecision("set_mode", "normal")
    if key.lower() == "c":
        return KeyDecision("set_mode", "coarse")
    if key.lower() == "h":
        return KeyDecision("help")
    if key.lower() == "o":
        return KeyDecision("gripper_open")
    if key.lower() == "g":
        return KeyDecision("gripper_grasp")
    if key.lower() == "q" or key == "SPACE":
        return KeyDecision("stop")
    return KeyDecision("ignore")


def measured_gripper_width(
    message: JointState, finger_joints: tuple[str, str] = FINGER_JOINTS
) -> float:
    if len(message.name) != len(message.position):
        raise RuntimeError("Gripper joint-state names and positions have different lengths.")
    positions = dict(zip(message.name, message.position))
    missing = [joint for joint in finger_joints if joint not in positions]
    if missing:
        raise RuntimeError(f"Gripper joint state is missing positions: {missing}")
    width = sum(float(positions[joint]) for joint in finger_joints)
    if not math.isfinite(width) or width < 0.0:
        raise RuntimeError(f"Measured gripper width is invalid: {width}")
    return width


def validate_gripper_configuration(args: argparse.Namespace) -> None:
    if not args.enable_gripper:
        return
    finite_positive = {
        "gripper_open_width": args.gripper_open_width,
        "gripper_speed": args.gripper_speed,
        "gripper_force": args.gripper_force,
        "gripper_min_force": args.gripper_min_force,
        "gripper_max_width": args.gripper_max_width,
        "gripper_max_force": args.gripper_max_force,
        "gripper_state_timeout": args.gripper_state_timeout,
        "gripper_open_tolerance": args.gripper_open_tolerance,
    }
    for name, value in finite_positive.items():
        if value is None or not math.isfinite(value) or value <= 0.0:
            raise RuntimeError(
                f"--{name.replace('_', '-')} must be supplied as a finite positive value "
                "when --enable-gripper is used."
            )
    if args.gripper_grasp_width is None:
        raise RuntimeError("--gripper-grasp-width is required with --enable-gripper.")
    if not math.isfinite(args.gripper_grasp_width) or args.gripper_grasp_width < 0.0:
        raise RuntimeError("--gripper-grasp-width must be finite and nonnegative.")
    for name in ("gripper_epsilon_inner", "gripper_epsilon_outer"):
        value = getattr(args, name)
        if not math.isfinite(value) or value < 0.0:
            raise RuntimeError(f"--{name.replace('_', '-')} must be finite and nonnegative.")
    if args.gripper_open_width > args.gripper_max_width:
        raise RuntimeError("--gripper-open-width exceeds --gripper-max-width.")
    if args.gripper_grasp_width > args.gripper_max_width:
        raise RuntimeError("--gripper-grasp-width exceeds --gripper-max-width.")
    if args.gripper_force < args.gripper_min_force:
        raise RuntimeError("--gripper-force is below --gripper-min-force.")
    if args.gripper_force > args.gripper_max_force:
        raise RuntimeError("--gripper-force exceeds --gripper-max-force.")


@dataclass(frozen=True)
class GripperSnapshot:
    sequence: int
    width: float


class KeyboardGripperClient:
    def __init__(
        self,
        node: MoveItJointGoalClient,
        *,
        state_topic: str,
        move_action: str,
        grasp_action: str,
    ) -> None:
        self.node = node
        self.sequence = 0
        self.width: float | None = None
        self.subscription = node.create_subscription(
            JointState, state_topic, self._state_callback, 10
        )
        self.move_client = ActionClient(node, Move, move_action)
        self.grasp_client = ActionClient(node, Grasp, grasp_action)

    def _state_callback(self, message: JointState) -> None:
        try:
            width = measured_gripper_width(message)
        except RuntimeError:
            return
        self.width = width
        self.sequence += 1

    def wait_for_servers(self, timeout: float) -> None:
        if not self.move_client.wait_for_server(timeout_sec=timeout):
            raise RuntimeError("Timed out waiting for the gripper move action server.")
        if not self.grasp_client.wait_for_server(timeout_sec=timeout):
            raise RuntimeError("Timed out waiting for the gripper grasp action server.")

    def wait_for_snapshot(
        self, timeout: float, *, after_sequence: int | None = None
    ) -> GripperSnapshot:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if (
                self.width is not None
                and (after_sequence is None or self.sequence > after_sequence)
            ):
                return GripperSnapshot(self.sequence, self.width)
            rclpy.spin_once(
                self.node,
                timeout_sec=min(0.05, max(0.0, deadline - time.monotonic())),
            )
        raise RuntimeError("Timed out waiting for a fresh measured gripper joint state.")

    def _wait_for_future(self, future, timeout: float, description: str):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(
                self.node,
                timeout_sec=min(0.05, max(0.0, deadline - time.monotonic())),
            )
        if not future.done():
            raise RuntimeError(f"Timed out waiting for {description}.")
        if future.exception() is not None:
            raise RuntimeError(f"{description} failed: {future.exception()}")
        return future.result()

    def _execute_action(self, client: ActionClient, goal, timeout: float, name: str):
        baseline_sequence = self.sequence
        goal_handle = self._wait_for_future(
            client.send_goal_async(goal), timeout, f"the gripper {name} goal response"
        )
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f"The gripper rejected the {name} goal.")
        wrapped_result = self._wait_for_future(
            goal_handle.get_result_async(), timeout, f"the gripper {name} result"
        )
        result = wrapped_result.result
        snapshot = self.wait_for_snapshot(timeout, after_sequence=baseline_sequence)
        if not result.success:
            detail = result.error.strip() or "action returned success=false"
            raise RuntimeError(
                f"Gripper {name} failed: {detail}; measured final width was "
                f"{snapshot.width:.6f} m"
            )
        return snapshot

    def open(self, width: float, speed: float, timeout: float) -> GripperSnapshot:
        goal = Move.Goal()
        goal.width = width
        goal.speed = speed
        return self._execute_action(self.move_client, goal, timeout, "open")

    def grasp(
        self,
        width: float,
        speed: float,
        force: float,
        epsilon_inner: float,
        epsilon_outer: float,
        timeout: float,
    ) -> GripperSnapshot:
        goal = Grasp.Goal()
        goal.width = width
        goal.speed = speed
        goal.force = force
        goal.epsilon.inner = epsilon_inner
        goal.epsilon.outer = epsilon_outer
        try:
            return self._execute_action(self.grasp_client, goal, timeout, "grasp")
        except RuntimeError as error:
            lower = width - epsilon_inner
            upper = width + epsilon_outer
            raise RuntimeError(
                f"{error}. Successful grasp interval is [{lower:.6f}, {upper:.6f}] m; "
                "verify that an object is present and that its measured contact width "
                "matches --gripper-grasp-width."
            ) from error


def requires_per_jog_confirmation(*, execute: bool, auto_execute: bool) -> bool:
    return execute and not auto_execute


def build_jog_target(
    current: Mapping[str, float], joint: str, direction: float, step: float
) -> dict[str, float]:
    if joint not in JOINTS:
        raise RuntimeError(f"Unknown joint: {joint}")
    if direction not in (-1.0, 1.0):
        raise RuntimeError("Jog direction must be -1 or +1.")
    if step not in STEP_SIZES.values():
        raise RuntimeError("Jog step is not one of the configured bounded increments.")
    target = dict(current)
    target[joint] += direction * step
    return target


def validate_target_limit(
    target: Mapping[str, float], joint: str, margin: float
) -> None:
    lower, upper = JOINT_LIMITS[joint]
    position = target[joint]
    if position < lower + margin or position > upper - margin:
        raise RuntimeError(
            f"Target rejected: {joint}={position:.6f} rad is outside the guarded "
            f"range [{lower + margin:.6f}, {upper - margin:.6f}] rad."
        )


def validate_session_budget(
    completed_commands: int,
    cumulative_delta: float,
    step: float,
    *,
    max_commands: int,
    max_cumulative_delta: float,
) -> None:
    if completed_commands >= max_commands:
        raise RuntimeError("Session command limit reached.")
    if cumulative_delta + step > max_cumulative_delta + 1e-12:
        raise RuntimeError("Session cumulative-delta limit would be exceeded.")


def maximum_state_error(
    expected: Mapping[str, float], measured: Mapping[str, float]
) -> tuple[str, float]:
    errors = {joint: abs(measured[joint] - expected[joint]) for joint in JOINTS}
    joint = max(errors, key=errors.get)
    return joint, errors[joint]


def maximum_absolute_velocity(velocities: Mapping[str, float]) -> tuple[str, float]:
    missing = [joint for joint in JOINTS if joint not in velocities]
    if missing:
        raise RuntimeError(f"Measured joint velocities are incomplete: {missing}")
    joint = max(JOINTS, key=lambda name: abs(velocities[name]))
    return joint, abs(velocities[joint])


def wait_for_fresh_snapshot(
    node: MoveItJointGoalClient, timeout: float
):
    # Process any samples queued while the operator was reading the plan, then
    # require one newer sample before using the robot state.
    node.spin_for(0.05)
    baseline_sequence = node.joint_state_sequence
    return node.wait_for_joint_state_snapshot(
        timeout=timeout,
        after_sequence=baseline_sequence,
        require_velocities=True,
    )


def validate_trajectory_start(trajectory, current: Mapping[str, float], limit: float) -> float:
    joint_trajectory = trajectory.joint_trajectory
    if not joint_trajectory.points:
        raise RuntimeError("MoveIt returned an empty trajectory.")
    names = list(joint_trajectory.joint_names)
    first_positions = list(joint_trajectory.points[0].positions)
    if len(first_positions) != len(names):
        raise RuntimeError("MoveIt trajectory start positions are incomplete.")
    by_name = dict(zip(names, first_positions))
    missing = [joint for joint in JOINTS if joint not in by_name]
    if missing:
        raise RuntimeError(f"MoveIt trajectory is missing joints: {missing}")
    if any(not math.isfinite(by_name[joint]) for joint in JOINTS):
        raise RuntimeError("MoveIt trajectory contains a non-finite start position.")
    maximum = max(abs(by_name[joint] - current[joint]) for joint in JOINTS)
    if maximum > limit:
        raise RuntimeError(
            f"MoveIt trajectory start error {maximum:.6f} rad exceeds "
            f"{limit:.6f} rad."
        )
    return maximum


def read_key() -> str:
    if not sys.stdin.isatty():
        raise RuntimeError("Keyboard jogging requires an interactive terminal.")

    descriptor = sys.stdin.fileno()
    previous = termios.tcgetattr(descriptor)
    try:
        tty.setraw(descriptor)
        first = os.read(descriptor, 1)
        if first == b"\x03":
            raise KeyboardInterrupt
        if first == b" ":
            return "SPACE"
        if first != b"\x1b":
            return first.decode("utf-8", errors="ignore")

        sequence = b""
        for _ in range(2):
            ready, _, _ = select.select([descriptor], [], [], 0.05)
            if not ready:
                break
            sequence += os.read(descriptor, 1)
        return {
            b"[D": "LEFT",
            b"[C": "RIGHT",
            b"OD": "LEFT",
            b"OC": "RIGHT",
        }.get(sequence, "ESCAPE")
    finally:
        termios.tcsetattr(descriptor, termios.TCSADRAIN, previous)


def discard_pending_keys() -> None:
    if sys.stdin.isatty():
        termios.tcflush(sys.stdin.fileno(), termios.TCIFLUSH)


def print_controls(gripper_enabled: bool = False) -> None:
    gripper_controls = (
        "  O           open gripper to configured width\n"
        "  G           grasp using configured width and force\n"
        if gripper_enabled
        else ""
    )
    print(
        "\nControls\n"
        "  1-7         select fr3_joint1 through fr3_joint7\n"
        "  Left/Right  plan a negative/positive jog\n"
        "  F/N/C       fine 0.010 / normal 0.030 / coarse 0.060 rad\n"
        f"{gripper_controls}"
        "  Space or Q  end this keyboard session while idle\n"
        "  H           show these controls\n"
        "\nSpace is not an emergency stop during execution. Keep the Franka user stop "
        "within reach.\n"
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Interactively jog one FR3 joint at a time through MoveIt."
    )
    parser.add_argument("--state-topic", default="/franka/joint_states")
    parser.add_argument("--plan-service", default="/plan_kinematic_path")
    parser.add_argument("--execute-action", default="/execute_trajectory")
    parser.add_argument("--group", default="fr3_arm")
    parser.add_argument("--planning-frame", default="base")
    parser.add_argument("--pipeline", default="")
    parser.add_argument("--planner", default="")
    parser.add_argument("--planning-attempts", type=int, default=5)
    parser.add_argument("--planning-time", type=float, default=5.0)
    parser.add_argument("--velocity-scaling", type=float, default=0.20)
    parser.add_argument("--acceleration-scaling", type=float, default=0.20)
    parser.add_argument("--goal-tolerance", type=float, default=0.002)
    parser.add_argument("--workspace-size", type=float, default=2.0)
    parser.add_argument("--joint-limit-margin", type=float, default=0.02)
    parser.add_argument("--server-timeout", type=float, default=10.0)
    parser.add_argument("--result-timeout", type=float, default=60.0)
    parser.add_argument("--state-timeout", type=float, default=3.0)
    parser.add_argument("--start-state-tolerance", type=float, default=0.01)
    parser.add_argument("--max-start-velocity", type=float, default=0.01)
    parser.add_argument("--max-commands", type=int, default=20)
    parser.add_argument("--max-cumulative-delta", type=float, default=0.20)
    parser.add_argument(
        "--enable-gripper",
        action="store_true",
        help="Enable explicit O=open and G=grasp keyboard commands.",
    )
    parser.add_argument("--gripper-state-topic", default="/franka_gripper/joint_states")
    parser.add_argument("--gripper-move-action", default="/franka_gripper/move")
    parser.add_argument("--gripper-grasp-action", default="/franka_gripper/grasp")
    parser.add_argument("--gripper-open-width", type=float, default=0.07)
    parser.add_argument("--gripper-grasp-width", type=float)
    parser.add_argument("--gripper-speed", type=float, default=0.02)
    parser.add_argument("--gripper-force", type=float)
    parser.add_argument("--gripper-epsilon-inner", type=float, default=0.01)
    parser.add_argument("--gripper-epsilon-outer", type=float, default=0.01)
    parser.add_argument("--gripper-max-width", type=float, default=0.08)
    parser.add_argument("--gripper-min-force", type=float, default=30.0)
    parser.add_argument("--gripper-max-force", type=float, default=70.0)
    parser.add_argument("--gripper-state-timeout", type=float, default=3.0)
    parser.add_argument("--gripper-open-tolerance", type=float, default=0.005)
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Allow confirmed trajectories to execute. The default is plan-only.",
    )
    parser.add_argument(
        "--auto-execute",
        action="store_true",
        help=(
            "With --execute, execute each successfully validated arrow-key jog "
            "without asking for Enter. Keys pressed while a jog is active are discarded."
        ),
    )
    add_post_execution_verification_arguments(parser)
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    positive = {
        "planning_time": args.planning_time,
        "velocity_scaling": args.velocity_scaling,
        "acceleration_scaling": args.acceleration_scaling,
        "goal_tolerance": args.goal_tolerance,
        "workspace_size": args.workspace_size,
        "joint_limit_margin": args.joint_limit_margin,
        "server_timeout": args.server_timeout,
        "result_timeout": args.result_timeout,
        "state_timeout": args.state_timeout,
        "start_state_tolerance": args.start_state_tolerance,
        "max_start_velocity": args.max_start_velocity,
        "max_cumulative_delta": args.max_cumulative_delta,
    }
    for name, value in positive.items():
        if not math.isfinite(value) or value <= 0.0:
            raise RuntimeError(f"--{name.replace('_', '-')} must be finite and positive.")
    if not 0.0 < args.velocity_scaling <= 1.0:
        raise RuntimeError("--velocity-scaling must be in (0, 1].")
    if not 0.0 < args.acceleration_scaling <= 1.0:
        raise RuntimeError("--acceleration-scaling must be in (0, 1].")
    if args.planning_attempts <= 0:
        raise RuntimeError("--planning-attempts must be positive.")
    if args.max_commands <= 0:
        raise RuntimeError("--max-commands must be positive.")
    if args.auto_execute and not args.execute:
        raise RuntimeError("--auto-execute requires --execute.")
    validate_gripper_configuration(args)
    validate_post_execution_verification_args(args)


def run_jog(node: MoveItJointGoalClient, args: argparse.Namespace) -> int:
    selected_index = 0
    mode = "normal"
    completed_commands = 0
    cumulative_delta = 0.0

    node.wait_for_plan_service(timeout=args.server_timeout)
    if args.execute:
        node.wait_for_execute_action(timeout=args.server_timeout)

    gripper = None
    if args.enable_gripper:
        gripper = KeyboardGripperClient(
            node,
            state_topic=args.gripper_state_topic,
            move_action=args.gripper_move_action,
            grasp_action=args.gripper_grasp_action,
        )
        initial_gripper = gripper.wait_for_snapshot(args.gripper_state_timeout)
        if args.execute:
            gripper.wait_for_servers(args.server_timeout)
        print(f"Initial measured gripper width: {initial_gripper.width:.6f} m")

    wait_for_fresh_snapshot(node, args.state_timeout)

    print("FR3 discrete keyboard joint jog")
    print(f"Mode: {'EXECUTION ENABLED' if args.execute else 'PLAN ONLY'}")
    if args.execute:
        confirmation = (
            "DISABLED (--auto-execute; one fresh arrow press executes one jog)"
            if args.auto_execute
            else "REQUIRED before every jog"
        )
        print(f"Per-jog confirmation: {confirmation}")
    print(f"Maximum commands: {args.max_commands}")
    print(f"Maximum cumulative delta: {args.max_cumulative_delta:.3f} rad")
    print_controls(args.enable_gripper)

    while rclpy.ok():
        selected_joint = JOINTS[selected_index]
        step = STEP_SIZES[mode]
        print(
            f"[{completed_commands}/{args.max_commands}, cumulative "
            f"{cumulative_delta:.3f}/{args.max_cumulative_delta:.3f} rad] "
            f"{selected_joint}, {mode} {step:.3f} rad > ",
            end="",
            flush=True,
        )
        key = read_key()
        print(key if key not in ("LEFT", "RIGHT") else f"{key} arrow")
        decision = interpret_key(key)

        if decision.kind == "select_joint":
            selected_index = int(decision.value)
            continue
        if decision.kind == "set_mode":
            mode = str(decision.value)
            continue
        if decision.kind == "help":
            print_controls(args.enable_gripper)
            continue
        if decision.kind == "stop":
            print("Keyboard session ended while idle. No command is queued.")
            return 0
        if decision.kind in ("gripper_open", "gripper_grasp"):
            if gripper is None:
                print("Gripper key ignored; restart with --enable-gripper to enable it.")
                continue
            validate_session_budget(
                completed_commands,
                cumulative_delta,
                0.0,
                max_commands=args.max_commands,
                max_cumulative_delta=args.max_cumulative_delta,
            )
            command_name = "open" if decision.kind == "gripper_open" else "grasp"
            target_width = (
                args.gripper_open_width
                if decision.kind == "gripper_open"
                else args.gripper_grasp_width
            )
            before = gripper.wait_for_snapshot(args.gripper_state_timeout)
            print(f"\nProposed gripper {command_name}")
            print(f"  Current width: {before.width:.6f} m")
            print(f"  Target width:  {target_width:.6f} m")
            print(f"  Speed:         {args.gripper_speed:.6f} m/s")
            if decision.kind == "gripper_grasp":
                print(f"  Force:         {args.gripper_force:.3f} N")

            if not args.execute:
                print("Plan-only result complete. The gripper was not commanded.\n")
                continue
            if requires_per_jog_confirmation(
                execute=args.execute, auto_execute=args.auto_execute
            ):
                confirmation = input(
                    f"Press Enter on an empty line to execute this gripper {command_name}; "
                    "type anything to cancel: "
                )
                if confirmation != "":
                    print("Gripper command cancelled.\n")
                    continue
            else:
                print(f"Auto-execute enabled: executing gripper {command_name}.")

            if decision.kind == "gripper_open":
                after = gripper.open(
                    args.gripper_open_width,
                    args.gripper_speed,
                    args.result_timeout,
                )
                error = abs(after.width - args.gripper_open_width)
                if error > args.gripper_open_tolerance:
                    raise RuntimeError(
                        f"Measured open width error is {error:.6f} m; limit is "
                        f"{args.gripper_open_tolerance:.6f} m."
                    )
            else:
                after = gripper.grasp(
                    args.gripper_grasp_width,
                    args.gripper_speed,
                    args.gripper_force,
                    args.gripper_epsilon_inner,
                    args.gripper_epsilon_outer,
                    args.result_timeout,
                )
                lower = args.gripper_grasp_width - args.gripper_epsilon_inner
                upper = args.gripper_grasp_width + args.gripper_epsilon_outer
                if after.width < lower or after.width > upper:
                    raise RuntimeError(
                        f"Measured grasp width {after.width:.6f} m is outside the "
                        f"configured interval [{lower:.6f}, {upper:.6f}] m."
                    )

            print(f"Measured gripper width: {after.width:.6f} m")
            print(f"Gripper {command_name}: VERIFIED SUCCESS\n")
            completed_commands += 1
            if args.auto_execute:
                discard_pending_keys()
            if completed_commands >= args.max_commands:
                print("Session command limit reached. Keyboard session ended.")
                return 0
            continue

        if decision.kind != "move":
            continue

        direction = float(decision.value)
        validate_session_budget(
            completed_commands,
            cumulative_delta,
            step,
            max_commands=args.max_commands,
            max_cumulative_delta=args.max_cumulative_delta,
        )

        snapshot = wait_for_fresh_snapshot(node, args.state_timeout)
        velocity_joint, start_velocity = maximum_absolute_velocity(snapshot.velocities)
        if start_velocity > args.max_start_velocity:
            raise RuntimeError(
                f"Robot is not settled: {velocity_joint} velocity is "
                f"{start_velocity:.6f} rad/s; limit is "
                f"{args.max_start_velocity:.6f} rad/s."
            )

        selected_joint = JOINTS[selected_index]
        target = build_jog_target(snapshot.positions, selected_joint, direction, step)
        validate_target_limit(target, selected_joint, args.joint_limit_margin)

        print("\nProposed jog")
        print(f"  Joint:     {selected_joint}")
        print(f"  Delta:     {direction * step:+.6f} rad")
        print(f"  Start vel: {start_velocity:.6f} rad/s on {velocity_joint}")
        print(format_joint_table(snapshot.positions, target))

        request = make_plan_request(node, snapshot.positions, target, args)
        response = node.plan(request, timeout=args.result_timeout)
        plan_code = response.error_code.val
        print(trajectory_summary(response.trajectory))
        print(f"Planning result: {error_name(plan_code)}")
        if plan_code != MoveItErrorCodes.SUCCESS:
            raise RuntimeError("MoveIt planning failed; keyboard session is terminated.")
        start_error = validate_trajectory_start(
            response.trajectory,
            snapshot.positions,
            args.start_state_tolerance,
        )
        print(f"Trajectory start-state error: {start_error:.6f} rad")

        if not args.execute:
            print("Plan-only result complete. The robot was not commanded.\n")
            continue

        if requires_per_jog_confirmation(
            execute=args.execute, auto_execute=args.auto_execute
        ):
            confirmation = input(
                "Press Enter on an empty line to execute this exact trajectory; "
                "type anything to cancel: "
            )
            if confirmation != "":
                print("Jog cancelled. The robot was not commanded.\n")
                continue
        else:
            print("Auto-execute enabled: executing this validated jog.")

        pre_execute = wait_for_fresh_snapshot(node, args.state_timeout)
        drift_joint, drift = maximum_state_error(snapshot.positions, pre_execute.positions)
        velocity_joint, start_velocity = maximum_absolute_velocity(pre_execute.velocities)
        print(
            f"Pre-execution state: drift={drift:.6f} rad on {drift_joint}, "
            f"velocity={start_velocity:.6f} rad/s on {velocity_joint}"
        )
        if drift > args.start_state_tolerance:
            raise RuntimeError(
                f"Robot state changed by {drift:.6f} rad after planning; execution blocked."
            )
        if start_velocity > args.max_start_velocity:
            raise RuntimeError(
                f"Robot is moving at {start_velocity:.6f} rad/s after planning; "
                "execution blocked."
            )

        execute_result = node.execute(response.trajectory, timeout=args.result_timeout)
        execute_code = execute_result.error_code.val
        print(f"Execution result: {error_name(execute_code)}")
        if execute_code != MoveItErrorCodes.SUCCESS:
            raise RuntimeError("MoveIt execution failed; keyboard session is terminated.")
        if not run_post_execution_verification(
            node, snapshot.positions, target, args
        ):
            raise RuntimeError(
                "Measured post-execution verification failed; keyboard session is terminated."
            )

        completed_commands += 1
        cumulative_delta += step
        if args.auto_execute:
            discard_pending_keys()
        print("Jog verified. The next key command is now enabled.\n")

        if completed_commands >= args.max_commands:
            print("Session command limit reached. Keyboard session ended.")
            return 0

    return 0


def main() -> int:
    args = parse_args()
    try:
        validate_args(args)
    except RuntimeError as error:
        print(f"Configuration error: {error}", file=sys.stderr)
        return 2

    rclpy.init()
    node = MoveItJointGoalClient(
        args.state_topic,
        args.plan_service,
        args.execute_action,
        node_name="fr3_keyboard_joint_jog",
    )
    try:
        return run_jog(node, args)
    except KeyboardInterrupt:
        print("\nKeyboard session interrupted. No new command will be sent.")
        return 130
    except RuntimeError as error:
        print(f"\nFAULT: {error}", file=sys.stderr)
        print("Do not send another command until the robot and MoveIt logs are checked.")
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
