#!/usr/bin/env python3
"""Stream safe intermediate JointState targets to IsaacJointStreamController.

This is a cautious test helper for larger joint-space moves. It does not talk
to libfranka or ros2_control directly. It only publishes small, name-complete
sensor_msgs/JointState targets to /isaac_joint_targets.
"""

import argparse
import math
import sys
import time
from typing import Dict, List, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


JOINTS = [
    "fr3_joint1",
    "fr3_joint2",
    "fr3_joint3",
    "fr3_joint4",
    "fr3_joint5",
    "fr3_joint6",
    "fr3_joint7",
]


class JointGoalStreamer(Node):
    def __init__(self, state_topic: str, target_topic: str, rate_hz: float) -> None:
        super().__init__("isaac_joint_goal_streamer")
        self.current: Dict[str, float] = {}
        self.publisher = self.create_publisher(JointState, target_topic, 10)
        self.subscription = self.create_subscription(
            JointState, state_topic, self._state_callback, 10
        )
        self.period = 1.0 / rate_hz
        self.target_topic = target_topic
        self.state_topic = state_topic

    def _state_callback(self, msg: JointState) -> None:
        for name, position in zip(msg.name, msg.position):
            if name in JOINTS and math.isfinite(position):
                self.current[name] = position

    def wait_for_state(self, timeout: float) -> Dict[str, float]:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if all(joint in self.current for joint in JOINTS):
                return {joint: self.current[joint] for joint in JOINTS}
        missing = [joint for joint in JOINTS if joint not in self.current]
        raise RuntimeError(
            f"Timed out waiting for all FR3 joints on {self.state_topic}. Missing: {missing}"
        )

    def publish_target(self, target: Dict[str, float]) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINTS)
        msg.position = [target[joint] for joint in JOINTS]
        self.publisher.publish(msg)


def parse_assignment(text: str) -> Tuple[str, float]:
    if ":=" not in text:
        raise argparse.ArgumentTypeError(
            f"Expected JOINT:=VALUE, for example fr3_joint7:=0.05. Got: {text}"
        )
    joint, value_text = text.split(":=", 1)
    joint = joint.strip()
    if joint not in JOINTS:
        raise argparse.ArgumentTypeError(f"Unknown joint '{joint}'. Expected one of: {JOINTS}")
    try:
        value = float(value_text)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"Invalid number in '{text}'") from exc
    if not math.isfinite(value):
        raise argparse.ArgumentTypeError(f"Value must be finite in '{text}'")
    return joint, value


def build_goal(
    start: Dict[str, float],
    absolute_targets: List[Tuple[str, float]],
    deltas: List[Tuple[str, float]],
) -> Dict[str, float]:
    goal = dict(start)
    for joint, value in absolute_targets:
        goal[joint] = value
    for joint, delta in deltas:
        goal[joint] += delta
    return goal


def next_waypoint(
    current: Dict[str, float],
    goal: Dict[str, float],
    lookahead: float,
) -> Dict[str, float]:
    waypoint = {}
    for joint in JOINTS:
        error = goal[joint] - current[joint]
        step = max(-lookahead, min(lookahead, error))
        waypoint[joint] = current[joint] + step
    return waypoint


def max_abs_delta(a: Dict[str, float], b: Dict[str, float]) -> float:
    return max(abs(a[joint] - b[joint]) for joint in JOINTS)


def format_joint_table(start: Dict[str, float], goal: Dict[str, float]) -> str:
    lines = ["joint        start(rad)    goal(rad)    delta(rad)"]
    for joint in JOINTS:
        lines.append(
            f"{joint:<11} {start[joint]:>10.6f}  {goal[joint]:>10.6f}  "
            f"{goal[joint] - start[joint]:>10.6f}"
        )
    return "\n".join(lines)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Stream small JointState waypoints to /isaac_joint_targets."
    )
    parser.add_argument(
        "--state-topic",
        default="/joint_states",
        help="JointState topic to read current FR3 positions from.",
    )
    parser.add_argument(
        "--target-topic",
        default="/isaac_joint_targets",
        help="JointState target topic consumed by IsaacJointStreamController.",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=20.0,
        help="Publish rate in Hz. Keep this above the controller target_timeout.",
    )
    parser.add_argument(
        "--lookahead",
        type=float,
        default=0.02,
        help="Maximum target distance ahead of measured position per joint, in rad.",
    )
    parser.add_argument(
        "--tolerance",
        type=float,
        default=0.003,
        help="Stop when every joint is this close to the goal, in rad.",
    )
    parser.add_argument(
        "--max-total-delta",
        type=float,
        default=0.25,
        help="Reject any requested per-joint move larger than this, in rad.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=120.0,
        help="Maximum execution time in seconds.",
    )
    parser.add_argument(
        "--target",
        action="append",
        type=parse_assignment,
        default=[],
        metavar="JOINT:=POSITION",
        help="Absolute joint target in rad. Can be used multiple times.",
    )
    parser.add_argument(
        "--delta",
        action="append",
        type=parse_assignment,
        default=[],
        metavar="JOINT:=DELTA",
        help="Relative joint move in rad. Can be used multiple times.",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Actually publish targets. Without this flag, the script only prints the plan.",
    )
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    if not args.target and not args.delta:
        raise RuntimeError("Provide at least one --target or --delta assignment.")
    if args.rate <= 0.0:
        raise RuntimeError("--rate must be positive.")
    if args.lookahead <= 0.0 or args.lookahead > 0.09:
        raise RuntimeError("--lookahead must be in (0, 0.09] rad to stay below max_target_jump.")
    if args.tolerance <= 0.0:
        raise RuntimeError("--tolerance must be positive.")
    if args.max_total_delta <= 0.0:
        raise RuntimeError("--max-total-delta must be positive.")
    if args.timeout <= 0.0:
        raise RuntimeError("--timeout must be positive.")


def main() -> int:
    args = parse_args()
    try:
        validate_args(args)
    except RuntimeError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 2

    rclpy.init()
    node = JointGoalStreamer(args.state_topic, args.target_topic, args.rate)

    try:
        start = node.wait_for_state(timeout=5.0)
        goal = build_goal(start, args.target, args.delta)

        too_large = [
            joint
            for joint in JOINTS
            if abs(goal[joint] - start[joint]) > args.max_total_delta
        ]
        if too_large:
            print(format_joint_table(start, goal))
            raise RuntimeError(
                "Requested move exceeds --max-total-delta for joints: "
                f"{too_large}. Split the move into smaller commands."
            )

        print(format_joint_table(start, goal))
        print(
            f"\nState topic: {args.state_topic}\n"
            f"Target topic: {args.target_topic}\n"
            f"Rate: {args.rate:.1f} Hz\n"
            f"Lookahead: {args.lookahead:.4f} rad\n"
            f"Tolerance: {args.tolerance:.4f} rad\n"
        )

        if not args.execute:
            print("Dry run only. Add --execute to publish waypoints.")
            return 0

        print("Publishing waypoints. Press Ctrl+C to stop.")
        deadline = time.monotonic() + args.timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.0)
            current = node.wait_for_state(timeout=1.0)
            remaining = max_abs_delta(current, goal)
            if remaining <= args.tolerance:
                node.publish_target(goal)
                print(f"Reached goal within {remaining:.6f} rad.")
                return 0

            waypoint = next_waypoint(current, goal, args.lookahead)
            node.publish_target(waypoint)
            time.sleep(node.period)

        raise RuntimeError("Timed out before reaching the goal.")

    except KeyboardInterrupt:
        print("\nStopped by user.")
        return 130
    except RuntimeError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
