#!/usr/bin/env python3
"""Stream guarded base-frame Cartesian keyboard commands to MoveIt Servo."""

from __future__ import annotations

import argparse
from collections.abc import Sequence
from contextlib import AbstractContextManager
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import select
import sys
import termios
import time
import tty

import rclpy
from control_msgs.msg import JointTrajectoryControllerState
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import TwistStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectory

from cartesian_execution_diagnostics import (
    controller_state_sample_from_message,
)
from fr3_keyboard_joint_jog import (
    KeyboardGripperClient,
    validate_gripper_configuration,
)


FR3_JOINTS = tuple(f"fr3_joint{index}" for index in range(1, 8))
AXES = ("x", "y", "z")
SERVO_STATUS = {
    -1: "INVALID",
    0: "NO_WARNING",
    1: "DECELERATE_FOR_APPROACHING_SINGULARITY",
    2: "HALT_FOR_SINGULARITY",
    3: "DECELERATE_FOR_COLLISION",
    4: "HALT_FOR_COLLISION",
    5: "JOINT_BOUND",
    6: "DECELERATE_FOR_LEAVING_SINGULARITY",
}
HALTING_SERVO_STATUSES = {2, 4, 5}


@dataclass(frozen=True)
class KeyDecision:
    kind: str
    value: object | None = None


@dataclass
class MotionState:
    axis: str = "y"
    mode: str = "normal"
    direction: float = 0.0
    active_until: float = 0.0

    def halt(self) -> None:
        self.direction = 0.0
        self.active_until = 0.0

    def refresh(self, direction: float, now: float, timeout: float) -> None:
        if direction not in (-1.0, 1.0):
            raise RuntimeError("Motion direction must be -1 or +1.")
        self.direction = direction
        self.active_until = now + timeout

    def linear_velocity(
        self, now: float, speeds: dict[str, float]
    ) -> tuple[float, float, float]:
        if self.direction == 0.0 or now >= self.active_until:
            self.halt()
            return (0.0, 0.0, 0.0)
        values = [0.0, 0.0, 0.0]
        values[AXES.index(self.axis)] = self.direction * speeds[self.mode]
        return tuple(values)


@dataclass(frozen=True)
class TrackingAssessment:
    commanded_distance: float
    measured_projection: float
    measured_distance: float
    progress_ratio: float
    cartesian_tracking_error: float
    failure_reason: str | None


@dataclass
class CartesianProgressGuard:
    grace_time: float
    minimum_command_distance: float
    minimum_progress_ratio: float
    maximum_tracking_error: float
    direction: tuple[float, float, float] | None = None
    start_time: float | None = None
    start_position: tuple[float, float, float] | None = None
    commanded_distance: float = 0.0

    def reset(self) -> None:
        self.direction = None
        self.start_time = None
        self.start_position = None
        self.commanded_distance = 0.0

    def update(
        self,
        *,
        now: float,
        velocity: Sequence[float],
        duration: float,
        tcp_position: Sequence[float],
        servo_status: int | None,
    ) -> TrackingAssessment | None:
        speed = vector_norm(velocity)
        if speed <= 1e-12:
            self.reset()
            return None

        unit_direction = tuple(float(value) / speed for value in velocity)
        position = tuple(float(value) for value in tcp_position)
        same_direction = (
            self.direction is not None
            and sum(
                left * right
                for left, right in zip(self.direction, unit_direction)
            )
            >= 1.0 - 1e-9
        )
        if not same_direction:
            self.direction = unit_direction
            self.start_time = float(now)
            self.start_position = position
            self.commanded_distance = 0.0

        self.commanded_distance += speed * max(float(duration), 0.0)
        assert self.direction is not None
        assert self.start_position is not None
        assert self.start_time is not None

        measured_offset = tuple(
            current - start
            for current, start in zip(position, self.start_position)
        )
        measured_projection = sum(
            value * direction
            for value, direction in zip(measured_offset, self.direction)
        )
        measured_distance = vector_norm(measured_offset)
        progress_ratio = (
            measured_projection / self.commanded_distance
            if self.commanded_distance > 1e-12
            else 0.0
        )
        expected_offset = tuple(
            self.commanded_distance * direction for direction in self.direction
        )
        tracking_error = vector_norm(
            tuple(
                expected - measured
                for expected, measured in zip(expected_offset, measured_offset)
            )
        )

        failure_reason: str | None = None
        eligible = (
            now - self.start_time >= self.grace_time
            and self.commanded_distance >= self.minimum_command_distance
            and servo_status in (None, 0)
        )
        if eligible and progress_ratio < self.minimum_progress_ratio:
            failure_reason = (
                "Measured TCP progress is too small: "
                f"{measured_projection:.6f} m along the command after "
                f"{self.commanded_distance:.6f} m was requested "
                f"(ratio={progress_ratio:.3f}, minimum="
                f"{self.minimum_progress_ratio:.3f})."
            )
        elif eligible and tracking_error > self.maximum_tracking_error:
            failure_reason = (
                "Cartesian command tracking error is too large: "
                f"{tracking_error:.6f} m (limit "
                f"{self.maximum_tracking_error:.6f} m)."
            )

        return TrackingAssessment(
            commanded_distance=self.commanded_distance,
            measured_projection=measured_projection,
            measured_distance=measured_distance,
            progress_ratio=progress_ratio,
            cartesian_tracking_error=tracking_error,
            failure_reason=failure_reason,
        )


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
    if lowered == "q":
        return KeyDecision("quit")
    if key == "SPACE":
        return KeyDecision("halt")
    return KeyDecision("ignore")


def delta_from_velocity(
    velocity: tuple[float, float, float], duration: float
) -> tuple[float, float, float]:
    return tuple(component * duration for component in velocity)


def vector_norm(values: Sequence[float]) -> float:
    return math.sqrt(sum(value * value for value in values))


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--planning-frame", default="base")
    parser.add_argument("--ee-link", default="fr3_hand_tcp")
    parser.add_argument(
        "--twist-topic", default="/servo_node/delta_twist_cmds"
    )
    parser.add_argument("--servo-status-topic", default="/servo_node/status")
    parser.add_argument(
        "--start-servo-service", default="/servo_node/start_servo"
    )
    parser.add_argument("--controller-manager", default="/controller_manager")
    parser.add_argument("--controller-name", default="fr3_arm_controller")
    parser.add_argument(
        "--controller-state-topic",
        default="/fr3_arm_controller/controller_state",
    )
    parser.add_argument(
        "--servo-output-topic",
        default="/fr3_arm_controller/joint_trajectory",
    )
    parser.add_argument("--joint-state-topic", default="/joint_states")
    parser.add_argument(
        "--gripper-state-topic", default="/franka_gripper/joint_states"
    )
    parser.add_argument(
        "--enable-gripper",
        action="store_true",
        help="Enable O=open and G=grasp keyboard commands.",
    )
    parser.add_argument("--gripper-move-action", default="/franka_gripper/move")
    parser.add_argument("--gripper-grasp-action", default="/franka_gripper/grasp")
    parser.add_argument("--gripper-open-width", type=float, default=0.08)
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
    parser.add_argument("--gripper-result-timeout", type=float, default=60.0)
    parser.add_argument("--command-rate", type=float, default=20.0)
    parser.add_argument("--record-rate", type=float, default=10.0)
    parser.add_argument("--fine-speed", type=float, default=0.005)
    parser.add_argument("--normal-speed", type=float, default=0.010)
    parser.add_argument("--coarse-speed", type=float, default=0.020)
    parser.add_argument("--max-linear-speed", type=float, default=0.020)
    parser.add_argument("--key-timeout", type=float, default=0.25)
    parser.add_argument("--state-timeout", type=float, default=0.50)
    parser.add_argument("--controller-state-timeout", type=float, default=0.50)
    parser.add_argument("--servo-output-timeout", type=float, default=0.25)
    parser.add_argument("--tracking-grace-time", type=float, default=1.0)
    parser.add_argument(
        "--tracking-min-command-distance", type=float, default=0.003
    )
    parser.add_argument("--minimum-progress-ratio", type=float, default=0.10)
    parser.add_argument(
        "--max-cartesian-tracking-error", type=float, default=0.010
    )
    parser.add_argument("--startup-timeout", type=float, default=30.0)
    parser.add_argument("--max-start-velocity", type=float, default=0.01)
    parser.add_argument("--max-session-seconds", type=float, default=120.0)
    parser.add_argument(
        "--max-cumulative-translation", type=float, default=0.50
    )
    parser.add_argument(
        "--record-jsonl",
        type=Path,
        help="Write synchronized 10 Hz pilot records to this JSONL file.",
    )
    parser.add_argument(
        "--task",
        default="",
        help="Natural-language task stored in each pilot record.",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Publish twists after one explicit session-level arm confirmation.",
    )
    parser.add_argument(
        "--no-start-servo",
        action="store_true",
        help="Do not call the Servo start service; require it to be started elsewhere.",
    )
    return parser.parse_args(argv)


def speed_presets(args: argparse.Namespace) -> dict[str, float]:
    return {
        "fine": args.fine_speed,
        "normal": args.normal_speed,
        "coarse": args.coarse_speed,
    }


def validate_args(args: argparse.Namespace) -> None:
    positive = {
        "command_rate": args.command_rate,
        "record_rate": args.record_rate,
        "fine_speed": args.fine_speed,
        "normal_speed": args.normal_speed,
        "coarse_speed": args.coarse_speed,
        "max_linear_speed": args.max_linear_speed,
        "key_timeout": args.key_timeout,
        "state_timeout": args.state_timeout,
        "controller_state_timeout": args.controller_state_timeout,
        "servo_output_timeout": args.servo_output_timeout,
        "tracking_grace_time": args.tracking_grace_time,
        "tracking_min_command_distance": args.tracking_min_command_distance,
        "max_cartesian_tracking_error": args.max_cartesian_tracking_error,
        "startup_timeout": args.startup_timeout,
        "max_start_velocity": args.max_start_velocity,
        "max_session_seconds": args.max_session_seconds,
        "max_cumulative_translation": args.max_cumulative_translation,
    }
    for name, value in positive.items():
        if not math.isfinite(value) or value <= 0.0:
            raise RuntimeError(
                f"--{name.replace('_', '-')} must be finite and positive."
            )
    if args.record_rate > args.command_rate:
        raise RuntimeError("--record-rate must not exceed --command-rate.")
    if not (
        args.fine_speed <= args.normal_speed <= args.coarse_speed
    ):
        raise RuntimeError(
            "Speed presets must satisfy fine <= normal <= coarse."
        )
    if args.coarse_speed > args.max_linear_speed:
        raise RuntimeError("--coarse-speed exceeds --max-linear-speed.")
    if args.key_timeout >= 1.0:
        raise RuntimeError("--key-timeout must remain below 1.0 second.")
    if not math.isfinite(args.minimum_progress_ratio) or not (
        0.0 < args.minimum_progress_ratio <= 1.0
    ):
        raise RuntimeError(
            "--minimum-progress-ratio must be finite and in (0, 1]."
        )
    if args.record_jsonl is not None and not args.task.strip():
        raise RuntimeError("--task is required when --record-jsonl is used.")
    validate_gripper_configuration(args)


def print_controls(args: argparse.Namespace) -> None:
    gripper_controls = (
        "  O           halt arm and open gripper\n"
        "  G           halt arm and grasp with configured width/force\n"
        if args.enable_gripper
        else ""
    )
    print(
        "\nControls\n"
        "  X/Y/Z       select the base-frame translation axis\n"
        "  Left/Right  move negative/positive while key repeat is active\n"
        "  F/N/C       fine/normal/coarse velocity\n"
        f"{gripper_controls}"
        "  Space       immediately command zero velocity\n"
        "  Q           command zero velocity and end the session\n"
        "  H           show these controls\n"
    )
    print(
        "Velocity presets: "
        f"fine={args.fine_speed:.3f}, normal={args.normal_speed:.3f}, "
        f"coarse={args.coarse_speed:.3f} m/s"
    )
    print(
        f"Command watchdog: {args.key_timeout:.3f} s. Releasing an arrow "
        "stops movement when terminal key-repeat messages cease."
    )
    print(
        "Orientation is held. Space is not a hardware emergency stop. "
        "Keep the Franka user stop within reach.\n"
    )


class RawTerminal(AbstractContextManager["RawTerminal"]):
    def __init__(self) -> None:
        if not sys.stdin.isatty():
            raise RuntimeError("Continuous keyboard control requires a terminal.")
        self.descriptor = sys.stdin.fileno()
        self.previous = termios.tcgetattr(self.descriptor)

    def __enter__(self) -> "RawTerminal":
        tty.setraw(self.descriptor)
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        termios.tcsetattr(
            self.descriptor, termios.TCSADRAIN, self.previous
        )

    def read_available(self) -> list[str]:
        keys: list[str] = []
        while select.select([self.descriptor], [], [], 0.0)[0]:
            first = os.read(self.descriptor, 1)
            if first == b"\x03":
                raise KeyboardInterrupt
            if first == b" ":
                keys.append("SPACE")
                continue
            if first != b"\x1b":
                keys.append(first.decode("utf-8", errors="ignore"))
                continue

            sequence = b""
            for _ in range(2):
                if not select.select([self.descriptor], [], [], 0.01)[0]:
                    break
                sequence += os.read(self.descriptor, 1)
            key = {
                b"[D": "LEFT",
                b"[C": "RIGHT",
                b"OD": "LEFT",
                b"OC": "RIGHT",
            }.get(sequence)
            if key is not None:
                keys.append(key)
        return keys

    def discard_pending(self) -> None:
        termios.tcflush(self.descriptor, termios.TCIFLUSH)


class PilotRecorder:
    def __init__(self, path: Path, args: argparse.Namespace) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        self.stream = path.open("x", encoding="ascii")
        self.sequence = 0
        self.action_frame = args.planning_frame
        self.write(
            {
                "schema_version": 1,
                "record_type": "session",
                "created_unix_ns": time.time_ns(),
                "task": args.task,
                "action_space": (
                    "base_relative_cartesian_translation_with_gripper"
                    if args.enable_gripper
                    else "base_relative_cartesian_translation"
                ),
                "action_frame": self.action_frame,
                "action_units": "meters_per_record_interval",
                "record_rate_hz": args.record_rate,
                "command_rate_hz": args.command_rate,
                "robot": "fr3",
            }
        )

    def write(self, record: dict[str, object]) -> None:
        self.stream.write(json.dumps(record, sort_keys=True) + "\n")
        self.stream.flush()

    def add_frame(
        self,
        *,
        observation: dict[str, object] | None,
        translation_delta: Sequence[float],
        servo_status: int | None,
        gripper_command: str | None = None,
    ) -> None:
        self.write(
            {
                "schema_version": 1,
                "record_type": "frame",
                "sequence": self.sequence,
                "time_unix_ns": time.time_ns(),
                "observation": observation,
                "action": {
                    "translation_delta": list(translation_delta),
                    "rotation_delta": [0.0, 0.0, 0.0],
                    "frame": self.action_frame,
                    "gripper_command": gripper_command,
                },
                "servo_status": servo_status,
                "servo_status_name": SERVO_STATUS.get(
                    servo_status, "NOT_AVAILABLE"
                ),
            }
        )
        self.sequence += 1

    def close(self, status: str) -> None:
        self.write(
            {
                "schema_version": 1,
                "record_type": "session_end",
                "time_unix_ns": time.time_ns(),
                "status": status,
                "frames": self.sequence,
            }
        )
        self.stream.close()


class ServoKeyboardNode(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("fr3_keyboard_cartesian_servo")
        self.args = args
        self.twist_publisher = self.create_publisher(
            TwistStamped, args.twist_topic, 10
        )
        self.create_subscription(
            JointState,
            args.joint_state_topic,
            self._joint_state_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            JointState,
            args.gripper_state_topic,
            self._gripper_state_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Int8, args.servo_status_topic, self._servo_status_callback, 10
        )
        self.create_subscription(
            JointTrajectoryControllerState,
            args.controller_state_topic,
            self._controller_state_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            JointTrajectory,
            args.servo_output_topic,
            self._servo_output_callback,
            10,
        )
        self.start_client = self.create_client(
            Trigger, args.start_servo_service
        )
        self.controller_client = self.create_client(
            ListControllers,
            f"{args.controller_manager.rstrip('/')}/list_controllers",
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.joint_positions: dict[str, float] = {}
        self.joint_velocities: dict[str, float] = {}
        self.joint_state_monotonic: float | None = None
        self.gripper_width: float | None = None
        self.servo_status: int | None = None
        self.controller_state = None
        self.controller_state_monotonic: float | None = None
        self.controller_state_error: str | None = None
        self.servo_output: dict[str, object] | None = None
        self.servo_output_monotonic: float | None = None
        self.gripper = (
            KeyboardGripperClient(
                self,
                state_topic=args.gripper_state_topic,
                move_action=args.gripper_move_action,
                grasp_action=args.gripper_grasp_action,
            )
            if args.enable_gripper
            else None
        )

    def _joint_state_callback(self, message: JointState) -> None:
        positions = dict(zip(message.name, message.position))
        velocities = dict(zip(message.name, message.velocity))
        if all(joint in positions for joint in FR3_JOINTS):
            self.joint_positions = {
                joint: float(positions[joint]) for joint in FR3_JOINTS
            }
            self.joint_velocities = {
                joint: float(velocities.get(joint, 0.0)) for joint in FR3_JOINTS
            }
            self.joint_state_monotonic = time.monotonic()

    def _gripper_state_callback(self, message: JointState) -> None:
        positions = dict(zip(message.name, message.position))
        finger_names = ("fr3_finger_joint1", "fr3_finger_joint2")
        if all(name in positions for name in finger_names):
            self.gripper_width = sum(float(positions[name]) for name in finger_names)

    def _servo_status_callback(self, message: Int8) -> None:
        self.servo_status = int(message.data)

    def _controller_state_callback(
        self, message: JointTrajectoryControllerState
    ) -> None:
        try:
            self.controller_state = controller_state_sample_from_message(
                message,
                receipt_time=time.monotonic(),
            )
        except RuntimeError as exc:
            self.controller_state_error = str(exc)
            return
        self.controller_state_error = None
        self.controller_state_monotonic = time.monotonic()

    def _servo_output_callback(self, message: JointTrajectory) -> None:
        summary: dict[str, object] = {
            "joint_names": list(message.joint_names),
            "point_count": len(message.points),
        }
        if message.points:
            point = message.points[-1]
            summary.update(
                {
                    "positions": list(point.positions),
                    "velocities": list(point.velocities),
                    "time_from_start": (
                        float(point.time_from_start.sec)
                        + float(point.time_from_start.nanosec) * 1e-9
                    ),
                }
            )
        self.servo_output = summary
        self.servo_output_monotonic = time.monotonic()

    def joint_state_age(self) -> float:
        if self.joint_state_monotonic is None:
            return math.inf
        return time.monotonic() - self.joint_state_monotonic

    def controller_state_age(self) -> float:
        if self.controller_state_monotonic is None:
            return math.inf
        return time.monotonic() - self.controller_state_monotonic

    def servo_output_age(self) -> float:
        if self.servo_output_monotonic is None:
            return math.inf
        return time.monotonic() - self.servo_output_monotonic

    def maximum_joint_velocity(self) -> tuple[str, float]:
        if not self.joint_velocities:
            return ("n/a", math.inf)
        return max(
            self.joint_velocities.items(), key=lambda item: abs(item[1])
        )

    def publish_velocity(self, velocity: Sequence[float]) -> None:
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.args.planning_frame
        message.twist.linear.x = float(velocity[0])
        message.twist.linear.y = float(velocity[1])
        message.twist.linear.z = float(velocity[2])
        self.twist_publisher.publish(message)

    def publish_halt(self, count: int = 8) -> None:
        for _ in range(count):
            self.publish_velocity((0.0, 0.0, 0.0))
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.02)

    def _wait_future(self, future, timeout: float, operation: str):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            raise RuntimeError(
                f"Timed out after {timeout:.1f} s waiting for {operation}."
            )
        if future.exception() is not None:
            raise RuntimeError(str(future.exception()))
        return future.result()

    def verify_controller(self) -> None:
        if not self.controller_client.wait_for_service(
            timeout_sec=self.args.startup_timeout
        ):
            raise RuntimeError(
                "Controller-manager service is unavailable. Start the FR3 "
                "MoveIt launch first."
            )
        response = self._wait_future(
            self.controller_client.call_async(ListControllers.Request()),
            self.args.startup_timeout,
            f"{self.args.controller_manager}/list_controllers",
        )
        states = {
            controller.name: controller.state
            for controller in response.controller
        }
        state = states.get(self.args.controller_name)
        if state != "active":
            raise RuntimeError(
                f"Controller {self.args.controller_name!r} is not active "
                f"(state={state!r})."
            )

    def start_servo(self) -> None:
        if not self.start_client.wait_for_service(
            timeout_sec=self.args.startup_timeout
        ):
            raise RuntimeError(
                "MoveIt Servo start service is unavailable. Start "
                "servo.launch.py first."
            )
        response = self._wait_future(
            self.start_client.call_async(Trigger.Request()),
            self.args.startup_timeout,
            self.args.start_servo_service,
        )
        if not response.success:
            raise RuntimeError(
                f"MoveIt Servo refused to start: {response.message}"
            )

    def wait_for_readiness(self) -> None:
        deadline = time.monotonic() + self.args.startup_timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            output_publishers = self.get_publishers_info_by_topic(
                self.args.servo_output_topic
            )
            output_subscribers = [
                endpoint
                for endpoint in self.get_subscriptions_info_by_topic(
                    self.args.servo_output_topic
                )
                if endpoint.node_name != self.get_name()
            ]
            if (
                self.twist_publisher.get_subscription_count() > 0
                and self.joint_state_age() <= self.args.state_timeout
                and self.controller_state_age()
                <= self.args.controller_state_timeout
                and output_publishers
                and output_subscribers
                and self.tcp_position() is not None
            ):
                break
        else:
            raise RuntimeError(
                "Timed out waiting for the complete Servo chain: twist "
                "subscriber, Servo output publisher, controller output "
                "subscriber, controller state, joint state, and TCP TF."
            )

        if self.controller_state_error is not None:
            raise RuntimeError(
                "Controller-state message is invalid: "
                f"{self.controller_state_error}"
            )

        joint, velocity = self.maximum_joint_velocity()
        if abs(velocity) > self.args.max_start_velocity:
            raise RuntimeError(
                f"Robot is not settled: {joint} velocity is {velocity:+.6f} "
                f"rad/s (limit {self.args.max_start_velocity:.6f})."
            )

    def _tcp_pose(self) -> dict[str, object] | None:
        tcp_pose: dict[str, object] | None = None
        try:
            transform = self.tf_buffer.lookup_transform(
                self.args.planning_frame,
                self.args.ee_link,
                Time(),
                timeout=Duration(seconds=0.02),
            )
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            tcp_pose = {
                "frame": self.args.planning_frame,
                "position": [translation.x, translation.y, translation.z],
                "quaternion_xyzw": [
                    rotation.x,
                    rotation.y,
                    rotation.z,
                    rotation.w,
                ],
            }
        except TransformException:
            pass
        return tcp_pose

    def tcp_position(self) -> tuple[float, float, float] | None:
        pose = self._tcp_pose()
        if pose is None:
            return None
        return tuple(float(value) for value in pose["position"])

    def controller_diagnostics(self) -> dict[str, object] | None:
        sample = self.controller_state
        if sample is None:
            return None
        maximum_error_joint = max(
            FR3_JOINTS,
            key=lambda joint: abs(sample.position_errors[joint]),
        )
        return {
            "age_seconds": self.controller_state_age(),
            "desired_positions": [
                sample.desired_positions[joint] for joint in FR3_JOINTS
            ],
            "actual_positions": [
                sample.actual_positions[joint] for joint in FR3_JOINTS
            ],
            "position_errors": [
                sample.position_errors[joint] for joint in FR3_JOINTS
            ],
            "desired_velocities": [
                sample.desired_velocities[joint] for joint in FR3_JOINTS
            ],
            "actual_velocities": [
                sample.actual_velocities[joint] for joint in FR3_JOINTS
            ],
            "output_efforts": [
                sample.output_efforts.get(joint) for joint in FR3_JOINTS
            ],
            "maximum_position_error_joint": maximum_error_joint,
            "maximum_position_error": abs(
                sample.position_errors[maximum_error_joint]
            ),
        }

    def servo_output_diagnostics(self) -> dict[str, object] | None:
        if self.servo_output is None:
            return None
        return {
            **self.servo_output,
            "age_seconds": self.servo_output_age(),
        }

    def observation(self) -> dict[str, object]:
        tcp_pose = self._tcp_pose()

        return {
            "joint_names": list(FR3_JOINTS),
            "joint_positions": [
                self.joint_positions.get(joint) for joint in FR3_JOINTS
            ],
            "joint_velocities": [
                self.joint_velocities.get(joint) for joint in FR3_JOINTS
            ],
            "tcp_pose": tcp_pose,
            "gripper_width": self.gripper_width,
            "controller_state": self.controller_diagnostics(),
            "servo_output": self.servo_output_diagnostics(),
        }


def apply_decision(
    state: MotionState,
    decision: KeyDecision,
    *,
    now: float,
    key_timeout: float,
) -> str:
    if decision.kind == "select_axis":
        state.halt()
        state.axis = str(decision.value)
        return f"Axis selected: {state.axis.upper()}"
    if decision.kind == "set_mode":
        state.halt()
        state.mode = str(decision.value)
        return f"Velocity mode: {state.mode}"
    if decision.kind == "move":
        state.refresh(float(decision.value), now, key_timeout)
        sign = "+" if state.direction > 0.0 else "-"
        return f"Streaming {state.axis.upper()}{sign}"
    if decision.kind == "halt":
        state.halt()
        return "HALT: zero Cartesian velocity"
    return ""


def execute_gripper_command(
    args: argparse.Namespace,
    node: ServoKeyboardNode | None,
    decision: KeyDecision,
) -> str:
    if decision.kind not in ("gripper_open", "gripper_grasp"):
        raise RuntimeError("Unsupported gripper keyboard decision.")
    if not args.enable_gripper:
        raise RuntimeError("Restart with --enable-gripper to use O/G commands.")

    command = "open" if decision.kind == "gripper_open" else "grasp"
    target_width = (
        args.gripper_open_width
        if command == "open"
        else args.gripper_grasp_width
    )
    print(f"\nGripper command: {command}")
    print(f"  Target width: {target_width:.6f} m")
    print(f"  Speed:       {args.gripper_speed:.6f} m/s")
    if command == "grasp":
        print(f"  Force:       {args.gripper_force:.3f} N")

    if node is None:
        print("  DRY RUN: no gripper action was sent.")
        return command
    if node.gripper is None:
        raise RuntimeError("Gripper client was not initialized.")

    node.publish_halt()
    before = node.gripper.wait_for_snapshot(args.gripper_state_timeout)
    print(f"  Initial measured width: {before.width:.6f} m")
    if command == "open":
        after = node.gripper.open(
            args.gripper_open_width,
            args.gripper_speed,
            args.gripper_result_timeout,
        )
        error = abs(after.width - args.gripper_open_width)
        if error > args.gripper_open_tolerance:
            raise RuntimeError(
                f"Measured open width error is {error:.6f} m; limit is "
                f"{args.gripper_open_tolerance:.6f} m."
            )
    else:
        after = node.gripper.grasp(
            args.gripper_grasp_width,
            args.gripper_speed,
            args.gripper_force,
            args.gripper_epsilon_inner,
            args.gripper_epsilon_outer,
            args.gripper_result_timeout,
        )
        lower = args.gripper_grasp_width - args.gripper_epsilon_inner
        upper = args.gripper_grasp_width + args.gripper_epsilon_outer
        if after.width < lower or after.width > upper:
            raise RuntimeError(
                f"Measured grasp width {after.width:.6f} m is outside the "
                f"configured interval [{lower:.6f}, {upper:.6f}] m."
            )

    node.publish_halt()
    print(f"  Measured width: {after.width:.6f} m")
    print(f"Gripper {command}: VERIFIED SUCCESS")
    return command


def print_session_diagnostics(
    *,
    cumulative_translation: float,
    commanded_vector: Sequence[float],
    session_start_tcp: Sequence[float] | None,
    latest_tcp: Sequence[float] | None,
    maximum_controller_errors: dict[str, float],
    servo_output_start: dict[str, float] | None,
    servo_output_end: dict[str, float] | None,
) -> None:
    print(
        "\nServo session diagnostics\n"
        f"  Integrated commanded distance: {cumulative_translation:.6f} m\n"
        "  Integrated commanded vector:   "
        f"x={commanded_vector[0]:+.6f}, y={commanded_vector[1]:+.6f}, "
        f"z={commanded_vector[2]:+.6f} m"
    )
    if session_start_tcp is not None and latest_tcp is not None:
        measured_vector = tuple(
            final - initial
            for final, initial in zip(latest_tcp, session_start_tcp)
        )
        print(
            "  Measured net TCP vector:       "
            f"x={measured_vector[0]:+.6f}, y={measured_vector[1]:+.6f}, "
            f"z={measured_vector[2]:+.6f} m\n"
            f"  Measured net TCP distance:     "
            f"{vector_norm(measured_vector):.6f} m"
        )

    if servo_output_start is not None and servo_output_end is not None:
        servo_deltas = {
            joint: servo_output_end[joint] - servo_output_start[joint]
            for joint in FR3_JOINTS
        }
        maximum_servo_joint = max(
            FR3_JOINTS, key=lambda joint: abs(servo_deltas[joint])
        )
        print(
            "  Servo output endpoint change:  "
            f"{servo_deltas[maximum_servo_joint]:+.6f} rad on "
            f"{maximum_servo_joint}"
        )
    else:
        print("  Servo output endpoint change:  not available")

    maximum_error_joint = max(
        FR3_JOINTS, key=lambda joint: maximum_controller_errors[joint]
    )
    print(
        "  Maximum controller error:       "
        f"{maximum_controller_errors[maximum_error_joint]:.6f} rad on "
        f"{maximum_error_joint}"
    )


def run_session(
    args: argparse.Namespace, node: ServoKeyboardNode | None
) -> int:
    state = MotionState()
    speeds = speed_presets(args)
    command_period = 1.0 / args.command_rate
    record_period = 1.0 / args.record_rate
    start = time.monotonic()
    last_tick = start
    next_tick = start
    next_record = start + record_period
    accumulated_delta = [0.0, 0.0, 0.0]
    cumulative_translation = 0.0
    last_velocity = (0.0, 0.0, 0.0)
    last_status: int | None = None
    pending_gripper_command: str | None = None
    exit_requested = False
    progress_guard = CartesianProgressGuard(
        grace_time=args.tracking_grace_time,
        minimum_command_distance=args.tracking_min_command_distance,
        minimum_progress_ratio=args.minimum_progress_ratio,
        maximum_tracking_error=args.max_cartesian_tracking_error,
    )
    commanded_vector = [0.0, 0.0, 0.0]
    session_start_tcp = node.tcp_position() if node is not None else None
    latest_tcp = session_start_tcp
    nonzero_command_started: float | None = None
    maximum_controller_errors = {joint: 0.0 for joint in FR3_JOINTS}
    servo_output_start: dict[str, float] | None = None
    servo_output_end: dict[str, float] | None = None
    recorder = (
        PilotRecorder(args.record_jsonl, args)
        if args.record_jsonl is not None
        else None
    )
    final_status = "COMPLETED"

    print_controls(args)
    print(
        f"Session mode: {'EXECUTION ENABLED' if node else 'DRY RUN'}; "
        f"command={args.command_rate:.1f} Hz, record={args.record_rate:.1f} Hz"
    )
    if node is not None:
        print(
            "Tracking guard: require at least "
            f"{args.minimum_progress_ratio:.0%} measured TCP progress after "
            f"{args.tracking_grace_time:.2f} s and "
            f"{args.tracking_min_command_distance:.3f} m of command."
        )

    try:
        with RawTerminal() as terminal:
            while not exit_requested:
                now = time.monotonic()
                elapsed = now - start
                if elapsed >= args.max_session_seconds:
                    print("\nSession duration reached; commanding halt.")
                    break

                if node is not None:
                    rclpy.spin_once(node, timeout_sec=0.0)
                    sample = node.controller_state
                    if sample is not None:
                        for joint in FR3_JOINTS:
                            maximum_controller_errors[joint] = max(
                                maximum_controller_errors[joint],
                                abs(sample.position_errors[joint]),
                            )
                    output = node.servo_output
                    if output is not None:
                        names = output.get("joint_names", [])
                        positions = output.get("positions", [])
                        if len(names) == len(positions) and all(
                            joint in names for joint in FR3_JOINTS
                        ):
                            output_positions = {
                                joint: float(positions[names.index(joint)])
                                for joint in FR3_JOINTS
                            }
                            if servo_output_start is None:
                                servo_output_start = output_positions
                            servo_output_end = output_positions

                for key in terminal.read_available():
                    decision = interpret_key(key)
                    if decision.kind == "quit":
                        state.halt()
                        exit_requested = True
                        print("\nQuit requested; commanding halt.")
                        break
                    if decision.kind == "help":
                        print_controls(args)
                        continue
                    if decision.kind in ("gripper_open", "gripper_grasp"):
                        state.halt()
                        if node is not None:
                            node.publish_halt()
                        if not args.enable_gripper:
                            print(
                                "\rGripper key ignored; restart with "
                                "--enable-gripper."
                            )
                        else:
                            pending_gripper_command = execute_gripper_command(
                                args, node, decision
                            )
                        terminal.discard_pending()
                        now = time.monotonic()
                        last_tick = now
                        next_tick = now
                        break
                    message = apply_decision(
                        state,
                        decision,
                        now=now,
                        key_timeout=args.key_timeout,
                    )
                    if message:
                        print(f"\r{message:<72}")

                if now < next_tick:
                    time.sleep(min(next_tick - now, command_period / 2.0))
                    continue

                dt = min(max(now - last_tick, 0.0), command_period * 2.0)
                velocity = state.linear_velocity(now, speeds)
                if vector_norm(velocity) > args.max_linear_speed + 1e-12:
                    raise RuntimeError("Generated velocity exceeds the configured cap.")

                step_delta = delta_from_velocity(velocity, dt)
                prospective = cumulative_translation + vector_norm(step_delta)
                if prospective > args.max_cumulative_translation + 1e-12:
                    raise RuntimeError(
                        "Session cumulative translation would exceed "
                        f"{args.max_cumulative_translation:.3f} m."
                    )
                cumulative_translation = prospective
                for index, value in enumerate(step_delta):
                    accumulated_delta[index] += value
                    commanded_vector[index] += value

                if node is not None:
                    if node.joint_state_age() > args.state_timeout:
                        raise RuntimeError(
                            "FR3 joint state became stale; movement is halted."
                        )
                    if (
                        node.controller_state_age()
                        > args.controller_state_timeout
                    ):
                        raise RuntimeError(
                            "FR3 controller state became stale; movement is "
                            "halted."
                        )
                    status = node.servo_status
                    if status != last_status and status is not None:
                        print(
                            f"\rServo status: {SERVO_STATUS.get(status, status)}"
                            f"{'':<32}"
                        )
                        last_status = status
                    if status in HALTING_SERVO_STATUSES and vector_norm(velocity) > 0.0:
                        raise RuntimeError(
                            "MoveIt Servo halted motion: "
                            f"{SERVO_STATUS.get(status, status)}."
                        )
                    node.publish_velocity(velocity)

                    if vector_norm(velocity) > 0.0:
                        if nonzero_command_started is None:
                            nonzero_command_started = now
                        if (
                            now - nonzero_command_started
                            >= args.servo_output_timeout
                            and node.servo_output_age()
                            > args.servo_output_timeout
                        ):
                            raise RuntimeError(
                                "MoveIt Servo output became stale while a "
                                "nonzero Cartesian command was active."
                            )
                    else:
                        nonzero_command_started = None

                    latest_tcp = node.tcp_position()
                    if latest_tcp is None:
                        raise RuntimeError(
                            "TCP transform became unavailable during the "
                            "Servo session."
                        )
                    assessment = progress_guard.update(
                        now=now,
                        velocity=velocity,
                        duration=dt,
                        tcp_position=latest_tcp,
                        servo_status=status,
                    )
                    if (
                        assessment is not None
                        and assessment.failure_reason is not None
                    ):
                        raise RuntimeError(
                            "Servo downstream tracking guard stopped the "
                            f"session. {assessment.failure_reason}"
                        )

                if velocity != last_velocity:
                    if vector_norm(velocity) == 0.0:
                        print("\rWatchdog/command halt: zero Cartesian velocity        ")
                    else:
                        print(
                            "\rVelocity [m/s]: "
                            f"x={velocity[0]:+.3f}, y={velocity[1]:+.3f}, "
                            f"z={velocity[2]:+.3f}"
                        )
                    last_velocity = velocity

                if recorder is not None and now >= next_record:
                    recorder.add_frame(
                        observation=node.observation() if node is not None else None,
                        translation_delta=accumulated_delta,
                        servo_status=node.servo_status if node is not None else None,
                        gripper_command=pending_gripper_command,
                    )
                    accumulated_delta = [0.0, 0.0, 0.0]
                    pending_gripper_command = None
                    while next_record <= now:
                        next_record += record_period

                last_tick = now
                next_tick += command_period
                if next_tick <= now:
                    next_tick = now + command_period
    except BaseException:
        final_status = "FAULT"
        raise
    finally:
        state.halt()
        if node is not None:
            node.publish_halt()
        if recorder is not None:
            recorder.close(final_status)
        print_session_diagnostics(
            cumulative_translation=cumulative_translation,
            commanded_vector=commanded_vector,
            session_start_tcp=session_start_tcp,
            latest_tcp=latest_tcp,
            maximum_controller_errors=maximum_controller_errors,
            servo_output_start=servo_output_start,
            servo_output_end=servo_output_end,
        )

    print("Session ended with zero velocity.")
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    node: ServoKeyboardNode | None = None
    ros_initialized = False
    try:
        validate_args(args)
        if args.execute:
            print("FR3 CONTINUOUS CARTESIAN SERVO EXECUTION REQUESTED")
            print(
                "This is one bounded streaming session. Arrow-key repeat can "
                "move the real robot without per-step confirmation."
            )
            confirmation = input("Type exactly 'START SERVO' to continue: ")
            if confirmation != "START SERVO":
                raise RuntimeError("Servo session was not armed.")

            rclpy.init()
            ros_initialized = True
            node = ServoKeyboardNode(args)
            node.verify_controller()
            node.wait_for_readiness()
            if node.gripper is not None:
                initial_gripper = node.gripper.wait_for_snapshot(
                    args.gripper_state_timeout
                )
                node.gripper.wait_for_servers(args.startup_timeout)
                print(
                    "Gripper readiness: PASS; measured width="
                    f"{initial_gripper.width:.6f} m"
                )
            if not args.no_start_servo:
                node.start_servo()
            joint, velocity = node.maximum_joint_velocity()
            print(
                "Readiness checks: PASS; "
                f"maximum start velocity={abs(velocity):.6f} rad/s on {joint}"
            )

        return run_session(args, node)
    except KeyboardInterrupt:
        print("\nInterrupted; zero velocity was requested.")
        return 130
    except (RuntimeError, OSError) as exc:
        print(f"\nFAULT: {exc}", file=sys.stderr)
        print(
            "Continuous Cartesian output is disabled. Check Servo, controller, "
            "joint-state, and robot logs before retrying.",
            file=sys.stderr,
        )
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if ros_initialized and rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
