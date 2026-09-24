#!/usr/bin/env python3
"""Execute and verify one explicit FR3 gripper open or grasp command."""

from __future__ import annotations

import argparse
from collections.abc import Sequence
import sys

import rclpy
from rclpy.node import Node

from fr3_keyboard_joint_jog import (
    KeyboardGripperClient,
    validate_gripper_configuration,
)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("open", "grasp"))
    parser.add_argument("--gripper-state-topic", default="/franka_gripper/joint_states")
    parser.add_argument("--gripper-move-action", default="/franka_gripper/move")
    parser.add_argument("--gripper-grasp-action", default="/franka_gripper/grasp")
    parser.add_argument("--gripper-open-width", type=float, default=0.07)
    parser.add_argument("--gripper-grasp-width", type=float, required=True)
    parser.add_argument("--gripper-speed", type=float, default=0.02)
    parser.add_argument("--gripper-force", type=float, required=True)
    parser.add_argument("--gripper-epsilon-inner", type=float, default=0.005)
    parser.add_argument("--gripper-epsilon-outer", type=float, default=0.005)
    parser.add_argument("--gripper-max-width", type=float, default=0.08)
    parser.add_argument("--gripper-min-force", type=float, default=30.0)
    parser.add_argument("--gripper-max-force", type=float, default=70.0)
    parser.add_argument("--gripper-state-timeout", type=float, default=3.0)
    parser.add_argument("--gripper-open-tolerance", type=float, default=0.005)
    parser.add_argument("--result-timeout", type=float, default=60.0)
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Actually send the selected gripper action.",
    )
    args = parser.parse_args(argv)
    args.enable_gripper = True
    return args


def run_command(node: Node, args: argparse.Namespace) -> None:
    validate_gripper_configuration(args)
    gripper = KeyboardGripperClient(
        node,
        state_topic=args.gripper_state_topic,
        move_action=args.gripper_move_action,
        grasp_action=args.gripper_grasp_action,
    )
    before = gripper.wait_for_snapshot(args.gripper_state_timeout)
    target_width = (
        args.gripper_open_width
        if args.command == "open"
        else args.gripper_grasp_width
    )
    print(f"Initial measured gripper width: {before.width:.6f} m")
    print(f"Gripper command: {args.command}")
    print(f"Target width:    {target_width:.6f} m")
    print(f"Speed:           {args.gripper_speed:.6f} m/s")
    if args.command == "grasp":
        print(f"Force:           {args.gripper_force:.3f} N")

    if not args.execute:
        print("Plan-only gripper check complete. No action goal was sent.")
        return

    gripper.wait_for_servers(args.gripper_state_timeout)
    if args.command == "open":
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
    print(f"Gripper {args.command}: VERIFIED SUCCESS")


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    rclpy.init()
    node = Node("fr3_gripper_command")
    try:
        run_command(node, args)
        return 0
    except KeyboardInterrupt:
        print("\nGripper command interrupted.")
        return 130
    except RuntimeError as exc:
        print(f"Gripper command failed: {exc}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
