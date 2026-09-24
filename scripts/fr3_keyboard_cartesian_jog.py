#!/usr/bin/env python3
"""Run one guarded Pilz LIN Cartesian jog for each fresh arrow key press."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
import os
from pathlib import Path
import select
import subprocess
import sys
import termios
import tty
from collections.abc import Callable, Sequence


STEP_SIZES = {
    "fine": 0.010,
    "normal": 0.025,
    "coarse": 0.050,
}

AXES = ("x", "y", "z")


@dataclass(frozen=True)
class KeyDecision:
    kind: str
    value: object | None = None


def interpret_key(key: str) -> KeyDecision:
    lowered = key.lower()
    if lowered in AXES:
        return KeyDecision("select_axis", lowered)
    if key == "LEFT":
        return KeyDecision("move", -1.0)
    if key == "RIGHT":
        return KeyDecision("move", 1.0)
    if lowered == "f":
        return KeyDecision("set_mode", "fine")
    if lowered == "n":
        return KeyDecision("set_mode", "normal")
    if lowered == "c":
        return KeyDecision("set_mode", "coarse")
    if lowered == "h":
        return KeyDecision("help")
    if lowered == "o":
        return KeyDecision("gripper_open")
    if lowered == "g":
        return KeyDecision("gripper_grasp")
    if lowered == "q" or key == "SPACE":
        return KeyDecision("stop")
    return KeyDecision("ignore")


def requires_per_jog_confirmation(*, execute: bool, auto_execute: bool) -> bool:
    return execute and not auto_execute


def validate_session_budget(
    completed_commands: int,
    cumulative_distance: float,
    next_distance: float,
    *,
    max_commands: int,
    max_cumulative_distance: float | None,
) -> None:
    if completed_commands >= max_commands:
        raise RuntimeError("Session command limit has been reached.")
    proposed = cumulative_distance + next_distance
    if (
        max_cumulative_distance is not None
        and proposed > max_cumulative_distance + 1e-12
    ):
        raise RuntimeError(
            "Session cumulative-distance limit would be exceeded "
            f"({proposed:.6f} m > {max_cumulative_distance:.6f} m)."
        )


def build_cartesian_command(
    args: argparse.Namespace,
    axis: str,
    direction: float,
    step: float,
) -> list[str]:
    if axis not in AXES:
        raise RuntimeError(f"Unsupported Cartesian axis: {axis}")
    if direction not in (-1.0, 1.0):
        raise RuntimeError("Cartesian direction must be -1 or +1.")
    if step not in STEP_SIZES.values():
        raise RuntimeError("Cartesian step is not one of the configured increments.")

    child = Path(__file__).with_name("moveit_cartesian_ik_joint_test.py")
    command = [
        sys.executable,
        str(child),
        "--path-mode",
        "pilz-lin",
        f"--d{axis}",
        f"{direction * step:.12g}",
        "--group",
        args.group,
        "--planning-frame",
        args.planning_frame,
        "--ee-link",
        args.ee_link,
        "--velocity-scaling",
        str(args.velocity_scaling),
        "--acceleration-scaling",
        str(args.acceleration_scaling),
        "--max-distance",
        str(args.max_distance),
        "--max-wrist-joint-delta",
        str(args.max_wrist_joint_delta),
        "--max-joint-excursion",
        str(args.max_joint_excursion),
        "--max-total-joint-delta",
        str(args.max_total_joint_delta),
        "--max-joint-step",
        str(args.max_joint_step),
        "--max-planned-velocity",
        str(args.max_planned_velocity),
        "--max-planned-acceleration",
        str(args.max_planned_acceleration),
        "--max-start-velocity",
        str(args.max_start_velocity),
        "--max-start-error",
        str(args.max_start_error),
        "--cartesian-verification-tolerance",
        str(args.cartesian_verification_tolerance),
        "--orientation-verification-tolerance",
        str(args.orientation_verification_tolerance),
        "--joint-verification-tolerance",
        str(args.joint_verification_tolerance),
        "--server-timeout",
        str(args.server_timeout),
        "--result-timeout",
        str(args.result_timeout),
    ]
    if args.execute:
        command.append("--execute")
    return command


def run_guarded_cartesian_jog(
    args: argparse.Namespace,
    axis: str,
    direction: float,
    step: float,
    *,
    runner: Callable[..., subprocess.CompletedProcess[bytes]] = subprocess.run,
) -> None:
    command = build_cartesian_command(args, axis, direction, step)
    result = runner(command, check=False)
    if result.returncode != 0:
        raise RuntimeError(
            "Guarded Cartesian command failed with exit code "
            f"{result.returncode}. Review the output above before continuing."
        )


def validate_gripper_configuration(args: argparse.Namespace) -> None:
    if not args.enable_gripper:
        return
    positive = {
        "gripper_open_width": args.gripper_open_width,
        "gripper_grasp_width": args.gripper_grasp_width,
        "gripper_speed": args.gripper_speed,
        "gripper_force": args.gripper_force,
        "gripper_max_width": args.gripper_max_width,
        "gripper_min_force": args.gripper_min_force,
        "gripper_max_force": args.gripper_max_force,
        "gripper_state_timeout": args.gripper_state_timeout,
        "gripper_open_tolerance": args.gripper_open_tolerance,
    }
    for name, value in positive.items():
        if value is None or not math.isfinite(value) or value <= 0.0:
            raise RuntimeError(
                f"--{name.replace('_', '-')} must be supplied as a finite "
                "positive value when --enable-gripper is used."
            )
    for name in ("gripper_epsilon_inner", "gripper_epsilon_outer"):
        value = getattr(args, name)
        if not math.isfinite(value) or value < 0.0:
            raise RuntimeError(
                f"--{name.replace('_', '-')} must be finite and nonnegative."
            )
    if args.gripper_open_width > args.gripper_max_width:
        raise RuntimeError("--gripper-open-width exceeds --gripper-max-width.")
    if args.gripper_grasp_width > args.gripper_max_width:
        raise RuntimeError("--gripper-grasp-width exceeds --gripper-max-width.")
    if args.gripper_force < args.gripper_min_force:
        raise RuntimeError("--gripper-force is below --gripper-min-force.")
    if args.gripper_force > args.gripper_max_force:
        raise RuntimeError("--gripper-force exceeds --gripper-max-force.")


def build_gripper_command(
    args: argparse.Namespace, command_name: str
) -> list[str]:
    if command_name not in ("open", "grasp"):
        raise RuntimeError(f"Unsupported gripper command: {command_name}")
    child = Path(__file__).with_name("fr3_gripper_command.py")
    command = [
        sys.executable,
        str(child),
        command_name,
        "--gripper-state-topic",
        args.gripper_state_topic,
        "--gripper-move-action",
        args.gripper_move_action,
        "--gripper-grasp-action",
        args.gripper_grasp_action,
        "--gripper-open-width",
        str(args.gripper_open_width),
        "--gripper-grasp-width",
        str(args.gripper_grasp_width),
        "--gripper-speed",
        str(args.gripper_speed),
        "--gripper-force",
        str(args.gripper_force),
        "--gripper-epsilon-inner",
        str(args.gripper_epsilon_inner),
        "--gripper-epsilon-outer",
        str(args.gripper_epsilon_outer),
        "--gripper-max-width",
        str(args.gripper_max_width),
        "--gripper-min-force",
        str(args.gripper_min_force),
        "--gripper-max-force",
        str(args.gripper_max_force),
        "--gripper-state-timeout",
        str(args.gripper_state_timeout),
        "--gripper-open-tolerance",
        str(args.gripper_open_tolerance),
        "--result-timeout",
        str(args.result_timeout),
    ]
    if args.execute:
        command.append("--execute")
    return command


def run_gripper_command(
    args: argparse.Namespace,
    command_name: str,
    *,
    runner: Callable[..., subprocess.CompletedProcess[bytes]] = subprocess.run,
) -> None:
    result = runner(build_gripper_command(args, command_name), check=False)
    if result.returncode != 0:
        raise RuntimeError(
            f"Gripper {command_name} command failed with exit code "
            f"{result.returncode}. Review the output above before continuing."
        )


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
        "  O           open gripper to the configured width\n"
        "  G           grasp using the configured width and force\n"
        if gripper_enabled
        else ""
    )
    print(
        "\nControls\n"
        "  X/Y/Z       select the base-frame translation axis\n"
        "  Left/Right  negative/positive Cartesian jog\n"
        "  F/N/C       fine 0.010 / normal 0.025 / coarse 0.050 m\n"
        f"{gripper_controls}"
        "  Space or Q  end this keyboard session while idle\n"
        "  H           show these controls\n"
        "\nOrientation is held at its current value. Space is not an emergency "
        "stop during planning or execution. Keep the Franka user stop within "
        "reach.\n"
    )


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Interactively request one guarded Pilz LIN Cartesian translation "
            "at a time."
        )
    )
    parser.add_argument("--group", default="fr3_arm")
    parser.add_argument("--planning-frame", default="base")
    parser.add_argument("--ee-link", default="fr3_hand_tcp")
    parser.add_argument("--velocity-scaling", type=float, default=0.20)
    parser.add_argument("--acceleration-scaling", type=float, default=0.20)
    parser.add_argument("--max-distance", type=float, default=0.050)
    parser.add_argument("--max-wrist-joint-delta", type=float, default=0.05)
    parser.add_argument("--max-joint-excursion", type=float, default=0.12)
    parser.add_argument("--max-total-joint-delta", type=float, default=0.30)
    parser.add_argument("--max-joint-step", type=float, default=0.04)
    parser.add_argument("--max-planned-velocity", type=float, default=0.02)
    parser.add_argument("--max-planned-acceleration", type=float, default=0.05)
    parser.add_argument("--max-start-velocity", type=float, default=0.01)
    parser.add_argument("--max-start-error", type=float, default=0.003)
    parser.add_argument(
        "--cartesian-verification-tolerance", type=float, default=0.003
    )
    parser.add_argument(
        "--orientation-verification-tolerance", type=float, default=0.03
    )
    parser.add_argument("--joint-verification-tolerance", type=float, default=0.01)
    parser.add_argument("--server-timeout", type=float, default=10.0)
    parser.add_argument("--result-timeout", type=float, default=60.0)
    parser.add_argument("--max-commands", type=int, default=20)
    parser.add_argument("--max-cumulative-distance", type=float, default=0.20)
    parser.add_argument(
        "--no-cumulative-distance-limit",
        action="store_true",
        help=(
            "Disable only the keyboard session's cumulative translation "
            "budget. The command-count and per-motion safety checks remain."
        ),
    )
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
    parser.add_argument("--gripper-epsilon-inner", type=float, default=0.005)
    parser.add_argument("--gripper-epsilon-outer", type=float, default=0.005)
    parser.add_argument("--gripper-max-width", type=float, default=0.08)
    parser.add_argument("--gripper-min-force", type=float, default=30.0)
    parser.add_argument("--gripper-max-force", type=float, default=70.0)
    parser.add_argument("--gripper-state-timeout", type=float, default=3.0)
    parser.add_argument("--gripper-open-tolerance", type=float, default=0.005)
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Allow confirmed and fully validated Cartesian plans to execute.",
    )
    parser.add_argument(
        "--auto-execute",
        action="store_true",
        help=(
            "With --execute, skip the Enter prompt. Each fresh arrow press "
            "still plans, validates, executes, and verifies one complete jog."
        ),
    )
    return parser.parse_args(argv)


def validate_args(args: argparse.Namespace) -> None:
    positive = {
        "velocity_scaling": args.velocity_scaling,
        "acceleration_scaling": args.acceleration_scaling,
        "max_distance": args.max_distance,
        "max_wrist_joint_delta": args.max_wrist_joint_delta,
        "max_joint_excursion": args.max_joint_excursion,
        "max_total_joint_delta": args.max_total_joint_delta,
        "max_joint_step": args.max_joint_step,
        "max_planned_velocity": args.max_planned_velocity,
        "max_planned_acceleration": args.max_planned_acceleration,
        "max_start_velocity": args.max_start_velocity,
        "max_start_error": args.max_start_error,
        "cartesian_verification_tolerance": args.cartesian_verification_tolerance,
        "orientation_verification_tolerance": args.orientation_verification_tolerance,
        "joint_verification_tolerance": args.joint_verification_tolerance,
        "server_timeout": args.server_timeout,
        "result_timeout": args.result_timeout,
        "max_cumulative_distance": args.max_cumulative_distance,
    }
    for name, value in positive.items():
        if not math.isfinite(value) or value <= 0.0:
            raise RuntimeError(f"--{name.replace('_', '-')} must be finite and positive.")
    if args.velocity_scaling > 1.0:
        raise RuntimeError("--velocity-scaling must not exceed 1.0.")
    if args.acceleration_scaling > 1.0:
        raise RuntimeError("--acceleration-scaling must not exceed 1.0.")
    if args.max_commands <= 0:
        raise RuntimeError("--max-commands must be positive.")
    if max(STEP_SIZES.values()) > args.max_distance:
        raise RuntimeError(
            "--max-distance must be at least the configured coarse step "
            f"({max(STEP_SIZES.values()):.3f} m)."
        )
    if args.auto_execute and not args.execute:
        raise RuntimeError("--auto-execute requires --execute.")
    validate_gripper_configuration(args)


def run_keyboard_session(args: argparse.Namespace) -> int:
    selected_axis = "y"
    mode = "normal"
    completed_commands = 0
    cumulative_distance = 0.0
    cumulative_limit = (
        None
        if args.no_cumulative_distance_limit
        else args.max_cumulative_distance
    )

    print("FR3 discrete keyboard Cartesian jog")
    print("Path: Pilz industrial LIN in the base frame")
    print(f"Mode: {'EXECUTION ENABLED' if args.execute else 'PLAN ONLY'}")
    if args.execute:
        confirmation = (
            "DISABLED (--auto-execute; one fresh arrow press starts one jog)"
            if args.auto_execute
            else "REQUIRED before every jog"
        )
        print(f"Per-jog confirmation: {confirmation}")
    print(f"Maximum commands: {args.max_commands}")
    if cumulative_limit is None:
        print("Maximum cumulative translation: DISABLED")
    else:
        print(f"Maximum cumulative translation: {cumulative_limit:.3f} m")
    print_controls(args.enable_gripper)

    while True:
        step = STEP_SIZES[mode]
        cumulative_display = (
            "unlimited"
            if cumulative_limit is None
            else f"{cumulative_limit:.3f}"
        )
        print(
            f"[{completed_commands}/{args.max_commands}, cumulative "
            f"{cumulative_distance:.3f}/{cumulative_display} m] "
            f"{selected_axis.upper()} axis, {mode} {step:.3f} m > ",
            end="",
            flush=True,
        )
        key = read_key()
        print(key if key not in ("LEFT", "RIGHT") else f"{key} arrow")
        decision = interpret_key(key)

        if decision.kind == "select_axis":
            selected_axis = str(decision.value)
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
            if not args.enable_gripper:
                print("Gripper key ignored; restart with --enable-gripper to enable it.")
                continue
            validate_session_budget(
                completed_commands,
                cumulative_distance,
                0.0,
                max_commands=args.max_commands,
                max_cumulative_distance=cumulative_limit,
            )
            command_name = "open" if decision.kind == "gripper_open" else "grasp"
            target_width = (
                args.gripper_open_width
                if command_name == "open"
                else args.gripper_grasp_width
            )
            print(f"\nProposed gripper {command_name}")
            print(f"  Target width: {target_width:.6f} m")
            print(f"  Speed:        {args.gripper_speed:.6f} m/s")
            if command_name == "grasp":
                print(f"  Force:        {args.gripper_force:.3f} N")

            if not args.execute:
                print("Plan-only result complete. The gripper was not commanded.\n")
                continue
            if requires_per_jog_confirmation(
                execute=args.execute, auto_execute=args.auto_execute
            ):
                confirmation = input(
                    f"Press Enter on an empty line to execute this gripper "
                    f"{command_name}; type anything to cancel: "
                )
                if confirmation != "":
                    print("Gripper command cancelled.\n")
                    continue
            else:
                print(f"Auto-execute enabled: executing gripper {command_name}.")

            run_gripper_command(args, command_name)
            completed_commands += 1
            discard_pending_keys()
            print(
                f"Gripper {command_name} verified. The next key command is now "
                "enabled.\n"
            )
            continue
        if decision.kind != "move":
            continue

        direction = float(decision.value)
        validate_session_budget(
            completed_commands,
            cumulative_distance,
            step,
            max_commands=args.max_commands,
            max_cumulative_distance=cumulative_limit,
        )
        signed_delta = direction * step
        print("\nProposed Cartesian jog")
        print(f"  Frame:       {args.planning_frame}")
        print(f"  TCP:         {args.ee_link}")
        print(f"  Axis:        {selected_axis.upper()}")
        print(f"  Translation: {signed_delta:+.6f} m")
        print("  Orientation: HOLD CURRENT")

        if requires_per_jog_confirmation(
            execute=args.execute, auto_execute=args.auto_execute
        ):
            confirmation = input(
                "Press Enter on an empty line to plan, validate, and execute "
                "this jog; type anything to cancel: "
            )
            if confirmation != "":
                print("Cartesian jog cancelled.\n")
                continue
        elif args.execute:
            print("Auto-execute enabled: starting guarded plan and execution.")

        run_guarded_cartesian_jog(
            args,
            selected_axis,
            direction,
            step,
        )
        discard_pending_keys()
        if args.execute:
            completed_commands += 1
            cumulative_distance += step
            print("Cartesian jog verified. The next key command is now enabled.\n")
        else:
            print("Plan-only jog complete. The robot was not commanded.\n")


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        validate_args(args)
        return run_keyboard_session(args)
    except KeyboardInterrupt:
        print("\nKeyboard session interrupted while idle or in the foreground.")
        return 130
    except RuntimeError as exc:
        print(f"\nFAULT: {exc}", file=sys.stderr)
        print(
            "Do not send another command until the robot and MoveIt logs are checked.",
            file=sys.stderr,
        )
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
