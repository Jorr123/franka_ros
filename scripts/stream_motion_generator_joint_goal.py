#!/usr/bin/env python3
"""Publish MotionGenerator-style JointState targets to IsaacJointStreamController.

This script is intentionally outside the controller. It is a test tool for
larger joint-space goals:

1. Read the current FR3 joint state.
2. Build a final absolute/relative joint goal.
3. Generate a synchronized smooth trajectory similar to Franka's C++
   MotionGenerator example.
4. Publish trajectory samples as sensor_msgs/JointState messages to
   /isaac_joint_targets.

The IsaacJointStreamController still performs the final validation and filtering.
"""

import argparse
import math
import sys
import time
from typing import Dict, List, Sequence, Tuple

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

K_JOINTS = 7
K_DELTA_Q_MOTION_FINISHED = 1e-6


def sign(value: float) -> int:
    return (value > 0.0) - (value < 0.0)


class MotionGenerator:
    """Python port of franka_example_controllers::MotionGenerator."""

    def __init__(self, speed_factor: float, q_start: Sequence[float], q_goal: Sequence[float]):
        if speed_factor <= 0.0 or speed_factor > 1.0:
            raise ValueError("speed_factor must be in (0, 1].")
        if len(q_start) != K_JOINTS or len(q_goal) != K_JOINTS:
            raise ValueError("q_start and q_goal must each contain 7 joints.")

        self.q_start = list(q_start)
        self.delta_q = [q_goal[i] - q_start[i] for i in range(K_JOINTS)]

        self.dq_max = [2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5]
        self.ddq_max_start = [5.0] * K_JOINTS
        self.ddq_max_goal = [5.0] * K_JOINTS

        self.dq_max = [value * speed_factor for value in self.dq_max]
        self.ddq_max_start = [value * speed_factor for value in self.ddq_max_start]
        self.ddq_max_goal = [value * speed_factor for value in self.ddq_max_goal]

        self.dq_max_sync = [0.0] * K_JOINTS
        self.t_1_sync = [0.0] * K_JOINTS
        self.t_2_sync = [0.0] * K_JOINTS
        self.t_f_sync = [0.0] * K_JOINTS
        self.q_1 = [0.0] * K_JOINTS

        self._calculate_synchronized_values()

    @property
    def duration(self) -> float:
        return max(self.t_f_sync)

    def desired_joint_positions(self, trajectory_time: float) -> Tuple[List[float], bool]:
        delta_q_d = [0.0] * K_JOINTS
        finished = self._calculate_desired_values(max(0.0, trajectory_time), delta_q_d)
        return [self.q_start[i] + delta_q_d[i] for i in range(K_JOINTS)], finished

    def _calculate_desired_values(self, trajectory_time: float, delta_q_d: List[float]) -> bool:
        sign_delta_q = [sign(value) for value in self.delta_q]
        joint_motion_finished = [False] * K_JOINTS

        for i in range(K_JOINTS):
            if abs(self.delta_q[i]) < K_DELTA_Q_MOTION_FINISHED:
                delta_q_d[i] = 0.0
                joint_motion_finished[i] = True
                continue

            t_d = self.t_2_sync[i] - self.t_1_sync[i]
            delta_t_2_sync = self.t_f_sync[i] - self.t_2_sync[i]

            if trajectory_time < self.t_1_sync[i]:
                delta_q_d[i] = (
                    -1.0
                    / math.pow(self.t_1_sync[i], 3.0)
                    * self.dq_max_sync[i]
                    * sign_delta_q[i]
                    * (0.5 * trajectory_time - self.t_1_sync[i])
                    * math.pow(trajectory_time, 3.0)
                )
            elif trajectory_time < self.t_2_sync[i]:
                delta_q_d[i] = (
                    self.q_1[i]
                    + (trajectory_time - self.t_1_sync[i])
                    * self.dq_max_sync[i]
                    * sign_delta_q[i]
                )
            elif trajectory_time < self.t_f_sync[i]:
                delta_q_d[i] = self.delta_q[i] + 0.5 * (
                    (
                        1.0
                        / math.pow(delta_t_2_sync, 3.0)
                        * (
                            trajectory_time
                            - self.t_1_sync[i]
                            - 2.0 * delta_t_2_sync
                            - t_d
                        )
                        * math.pow((trajectory_time - self.t_1_sync[i] - t_d), 3.0)
                    )
                    + (
                        2.0 * trajectory_time
                        - 2.0 * self.t_1_sync[i]
                        - delta_t_2_sync
                        - 2.0 * t_d
                    )
                ) * self.dq_max_sync[i] * sign_delta_q[i]
            else:
                delta_q_d[i] = self.delta_q[i]
                joint_motion_finished[i] = True

        return all(joint_motion_finished)

    def _calculate_synchronized_values(self) -> None:
        dq_max_reach = list(self.dq_max)
        t_f = [0.0] * K_JOINTS
        sign_delta_q = [sign(value) for value in self.delta_q]

        for i in range(K_JOINTS):
            if abs(self.delta_q[i]) <= K_DELTA_Q_MOTION_FINISHED:
                continue

            triangular_limit = 0.75 * (
                math.pow(self.dq_max[i], 2.0) / self.ddq_max_start[i]
                + math.pow(self.dq_max[i], 2.0) / self.ddq_max_goal[i]
            )
            if abs(self.delta_q[i]) < triangular_limit:
                dq_max_reach[i] = math.sqrt(
                    4.0
                    / 3.0
                    * self.delta_q[i]
                    * sign_delta_q[i]
                    * (self.ddq_max_start[i] * self.ddq_max_goal[i])
                    / (self.ddq_max_start[i] + self.ddq_max_goal[i])
                )

            t_1 = 1.5 * dq_max_reach[i] / self.ddq_max_start[i]
            delta_t_2 = 1.5 * dq_max_reach[i] / self.ddq_max_goal[i]
            t_f[i] = t_1 / 2.0 + delta_t_2 / 2.0 + abs(self.delta_q[i]) / dq_max_reach[i]

        max_t_f = max(t_f)

        for i in range(K_JOINTS):
            if abs(self.delta_q[i]) <= K_DELTA_Q_MOTION_FINISHED:
                continue

            param_a = 0.75 * (self.ddq_max_goal[i] + self.ddq_max_start[i])
            param_b = -1.0 * max_t_f * self.ddq_max_goal[i] * self.ddq_max_start[i]
            param_c = abs(self.delta_q[i]) * self.ddq_max_goal[i] * self.ddq_max_start[i]
            discriminant = max(0.0, param_b * param_b - 4.0 * param_a * param_c)

            self.dq_max_sync[i] = (-param_b - math.sqrt(discriminant)) / (2.0 * param_a)
            self.t_1_sync[i] = 1.5 * self.dq_max_sync[i] / self.ddq_max_start[i]
            delta_t_2_sync = 1.5 * self.dq_max_sync[i] / self.ddq_max_goal[i]
            self.t_f_sync[i] = (
                self.t_1_sync[i] / 2.0
                + delta_t_2_sync / 2.0
                + abs(self.delta_q[i] / self.dq_max_sync[i])
            )
            self.t_2_sync[i] = self.t_f_sync[i] - delta_t_2_sync
            self.q_1[i] = self.dq_max_sync[i] * sign_delta_q[i] * 0.5 * self.t_1_sync[i]


class JointTargetPublisher(Node):
    def __init__(self, state_topic: str, target_topic: str) -> None:
        super().__init__("motion_generator_isaac_joint_goal_streamer")
        self.current: Dict[str, float] = {}
        self.publisher = self.create_publisher(JointState, target_topic, 10)
        self.subscription = self.create_subscription(
            JointState, state_topic, self._state_callback, 10
        )
        self.state_topic = state_topic
        self.target_topic = target_topic

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

    def publish_target(self, positions: Sequence[float]) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINTS)
        msg.position = list(positions)
        self.publisher.publish(msg)


def parse_assignment(text: str) -> Tuple[str, float]:
    if ":=" not in text:
        raise argparse.ArgumentTypeError(
            f"Expected JOINT:=VALUE, for example fr3_joint7:=0.10. Got: {text}"
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


def dict_to_list(values: Dict[str, float]) -> List[float]:
    return [values[joint] for joint in JOINTS]


def list_to_dict(values: Sequence[float]) -> Dict[str, float]:
    return {joint: values[i] for i, joint in enumerate(JOINTS)}


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


def limit_to_measured_lookahead(
    desired: Dict[str, float],
    measured: Dict[str, float],
    lookahead: float,
) -> List[float]:
    limited = []
    for joint in JOINTS:
        error = desired[joint] - measured[joint]
        step = max(-lookahead, min(lookahead, error))
        limited.append(measured[joint] + step)
    return limited


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
        description="Publish MotionGenerator-style targets to IsaacJointStreamController."
    )
    parser.add_argument("--state-topic", default="/joint_states")
    parser.add_argument("--target-topic", default="/isaac_joint_targets")
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument(
        "--speed-factor",
        type=float,
        default=0.02,
        help="MotionGenerator speed factor in (0, 1]. Start very small.",
    )
    parser.add_argument(
        "--lookahead",
        type=float,
        default=0.02,
        help="Maximum published target distance ahead of measured position, rad.",
    )
    parser.add_argument("--tolerance", type=float, default=0.003)
    parser.add_argument(
        "--max-total-delta",
        type=float,
        default=0.30,
        help="Reject any requested per-joint move larger than this, rad.",
    )
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--hold-after", type=float, default=1.0)
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
        help="Actually publish targets. Without this flag, only print the plan.",
    )
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    if not args.target and not args.delta:
        raise RuntimeError("Provide at least one --target or --delta assignment.")
    if args.rate <= 0.0:
        raise RuntimeError("--rate must be positive.")
    if args.speed_factor <= 0.0 or args.speed_factor > 1.0:
        raise RuntimeError("--speed-factor must be in (0, 1].")
    if args.lookahead <= 0.0 or args.lookahead > 0.09:
        raise RuntimeError("--lookahead must be in (0, 0.09] rad.")
    if args.tolerance <= 0.0:
        raise RuntimeError("--tolerance must be positive.")
    if args.max_total_delta <= 0.0:
        raise RuntimeError("--max-total-delta must be positive.")
    if args.timeout <= 0.0:
        raise RuntimeError("--timeout must be positive.")
    if args.hold_after < 0.0:
        raise RuntimeError("--hold-after must be non-negative.")


def main() -> int:
    args = parse_args()
    try:
        validate_args(args)
    except RuntimeError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 2

    rclpy.init()
    node = JointTargetPublisher(args.state_topic, args.target_topic)

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
                f"{too_large}. Split it into smaller goals."
            )

        generator = MotionGenerator(args.speed_factor, dict_to_list(start), dict_to_list(goal))

        print(format_joint_table(start, goal))
        print(
            f"\nState topic: {args.state_topic}\n"
            f"Target topic: {args.target_topic}\n"
            f"Rate: {args.rate:.1f} Hz\n"
            f"Speed factor: {args.speed_factor:.4f}\n"
            f"Generated duration: {generator.duration:.2f} s\n"
            f"Measured-state lookahead: {args.lookahead:.4f} rad\n"
            f"Tolerance: {args.tolerance:.4f} rad\n"
        )

        if not args.execute:
            print("Dry run only. Add --execute to publish MotionGenerator targets.")
            return 0

        print("Publishing MotionGenerator targets. Press Ctrl+C to stop.")
        start_time = time.monotonic()
        deadline = start_time + args.timeout
        next_status_time = start_time
        period = 1.0 / args.rate

        while rclpy.ok() and time.monotonic() < deadline:
            now = time.monotonic()
            rclpy.spin_once(node, timeout_sec=0.0)
            measured = node.wait_for_state(timeout=1.0)

            elapsed = now - start_time
            desired_list, trajectory_finished = generator.desired_joint_positions(elapsed)
            desired = list_to_dict(desired_list)
            target_list = limit_to_measured_lookahead(desired, measured, args.lookahead)
            node.publish_target(target_list)

            remaining = max_abs_delta(measured, goal)
            if now >= next_status_time:
                print(f"elapsed={elapsed:7.2f}s  max_remaining={remaining:.5f} rad")
                next_status_time = now + 2.0

            if trajectory_finished and remaining <= args.tolerance:
                hold_deadline = time.monotonic() + args.hold_after
                while rclpy.ok() and time.monotonic() < hold_deadline:
                    node.publish_target(dict_to_list(goal))
                    time.sleep(period)
                print(f"Reached goal within {remaining:.6f} rad.")
                return 0

            time.sleep(period)

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
