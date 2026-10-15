#!/usr/bin/env python3
"""Serve measured FR3 state through an IP-restricted, read-only HTTP endpoint."""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import ipaddress
import json
import math
import threading
import time
from typing import Any


JOINTS = tuple(f"fr3_joint{index}" for index in range(1, 8))
FINGER_JOINTS = ("fr3_finger_joint1", "fr3_finger_joint2")
STATE_PATH = "/v1/robot-state"
DEFAULT_JOINT_STATE_TOPIC = "/franka/joint_states"
GRAPH_MAX_AGE_SECONDS = 1.0
FUTURE_STAMP_TOLERANCE_SECONDS = 0.1


def _named_positions(message: Any, required: tuple[str, ...]) -> tuple[float, ...]:
    names = list(message.name)
    positions = list(message.position)
    if any(not isinstance(name, str) or not name for name in names):
        raise ValueError("JointState contains invalid joint names")
    if len(names) != len(positions) or len(names) != len(set(names)):
        raise ValueError("JointState names and positions are incomplete or duplicated")
    values = dict(zip(names, positions))
    if not set(required).issubset(values):
        raise ValueError("JointState is missing required joints")
    selected = tuple(float(values[joint]) for joint in required)
    if any(isinstance(values[joint], bool) for joint in required) or not all(
        math.isfinite(value) for value in selected
    ):
        raise ValueError("JointState contains invalid positions")
    return selected


@dataclass
class StateChannel:
    topic: str
    required_joints: tuple[str, ...]
    publishers: list[dict[str, str]] = field(default_factory=list)
    graph_checked_at: float | None = None
    positions: tuple[float, ...] | None = None
    source_stamp_ns: int = 0
    received_at: float = 0.0
    received_unix_ms: int = 0
    age_at_receipt: float = 0.0
    sequence: int = 0
    error: str = "No complete state received"


class StateCache:
    def __init__(
        self,
        max_age_seconds: float = 1.0,
        joint_topic: str = DEFAULT_JOINT_STATE_TOPIC,
        gripper_topic: str = "/franka_gripper/joint_states",
    ) -> None:
        if not math.isfinite(max_age_seconds) or max_age_seconds <= 0.0:
            raise ValueError("Maximum state age must be finite and positive")
        self.max_age_seconds = max_age_seconds
        self._lock = threading.Lock()
        self._channels = {
            "joints": StateChannel(joint_topic, JOINTS),
            "gripper": StateChannel(gripper_topic, FINGER_JOINTS),
        }

    def update_publishers(self, kind: str, endpoints: list[Any]) -> None:
        publishers = sorted(
            [
                {
                    "node_name": endpoint.node_name,
                    "node_namespace": endpoint.node_namespace,
                    "gid": bytes(endpoint.endpoint_gid).hex(),
                }
                for endpoint in endpoints
            ],
            key=lambda publisher: publisher["gid"],
        )
        with self._lock:
            channel = self._channels[kind]
            # Require a new sample after a publisher appears, disappears, or restarts.
            if publishers != channel.publishers:
                channel.positions = None
                channel.error = "Waiting for a new sample after a publisher change"
            channel.publishers = publishers
            channel.graph_checked_at = time.monotonic()

    def update(self, kind: str, message: Any, now_ros_ns: int) -> None:
        with self._lock:
            channel = self._channels[kind]
            now = time.monotonic()
            reason = self._source_error(channel, now)
            try:
                if reason:
                    raise ValueError(reason)
                positions = _named_positions(message, channel.required_joints)
                stamp = message.header.stamp
                if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
                    raise ValueError("Invalid JointState header timestamp")
                stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
                age = (now_ros_ns - stamp_ns) / 1_000_000_000
                if stamp_ns <= 0 or age < -FUTURE_STAMP_TOLERANCE_SECONDS:
                    raise ValueError("JointState header timestamp is zero or in the future")
                if age > self.max_age_seconds:
                    raise ValueError("JointState was already stale when received")
                if kind == "gripper" and (
                    any(position < 0.0 for position in positions)
                    or not math.isfinite(sum(positions))
                ):
                    raise ValueError("Invalid measured gripper width")
            except (AttributeError, TypeError, ValueError, OverflowError) as error:
                channel.positions = None
                channel.error = str(error)
                return
            channel.positions = positions
            channel.source_stamp_ns = stamp_ns
            channel.received_at = now
            channel.received_unix_ms = int(time.time() * 1000)
            channel.age_at_receipt = max(0.0, age)
            channel.sequence += 1
            channel.error = ""

    @staticmethod
    def _source_error(channel: StateChannel, now: float) -> str | None:
        if channel.graph_checked_at is None or now - channel.graph_checked_at > GRAPH_MAX_AGE_SECONDS:
            return f"Publisher discovery is unavailable or stale for {channel.topic}"
        if len(channel.publishers) != 1:
            return f"Expected exactly one publisher on {channel.topic}; found {len(channel.publishers)}"
        if not channel.publishers[0]["gid"] or not any(bytes.fromhex(channel.publishers[0]["gid"])):
            return f"Publisher identity is unavailable for {channel.topic}"
        return None

    def _state_error(self, channel: StateChannel, now: float) -> str | None:
        reason = self._source_error(channel, now)
        if reason:
            return reason
        if channel.positions is None:
            return channel.error
        if channel.age_at_receipt + now - channel.received_at > self.max_age_seconds:
            return f"State is stale for {channel.topic}"
        return None

    def snapshot(self) -> tuple[int, dict[str, Any]]:
        with self._lock:
            now = time.monotonic()
            joints = self._channels["joints"]
            gripper = self._channels["gripper"]
            reason = self._state_error(joints, now)
            if reason:
                return 503, {"status": "unavailable", "reason": reason}
            gripper_reason = self._state_error(gripper, now)
            return 200, {
                "schema_version": 1,
                "read_only": True,
                "status": "ok",
                "joint_names": list(JOINTS),
                "joint_positions_rad": list(joints.positions),
                "joint_state_sequence": joints.sequence,
                "joint_state_stamp_ns": joints.source_stamp_ns,
                "joint_state_received_unix_ms": joints.received_unix_ms,
                "joint_state_age_seconds": joints.age_at_receipt + now - joints.received_at,
                "joint_state_topic": joints.topic,
                "joint_state_publisher": dict(joints.publishers[0]),
                "gripper_width_m": None if gripper_reason else sum(gripper.positions),
                "gripper_state_age_seconds": (
                    None if gripper_reason else gripper.age_at_receipt + now - gripper.received_at
                ),
                "gripper_unavailable_reason": gripper_reason,
            }


class StateHttpServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address: tuple[str, int], cache: StateCache, allowed_clients: set[str]):
        self.cache = cache
        self.allowed_clients = allowed_clients
        super().__init__(address, StateRequestHandler)


class StateRequestHandler(BaseHTTPRequestHandler):
    server: StateHttpServer

    def do_GET(self) -> None:
        if self.client_address[0] not in self.server.allowed_clients:
            self._send_json(403, {"status": "error", "reason": "Client IP is not allowed"})
            return
        if self.path != STATE_PATH:
            self._send_json(404, {"status": "error", "reason": "Not found"})
            return
        status, payload = self.server.cache.snapshot()
        self._send_json(status, payload)

    def _send_json(self, status: int, payload: dict[str, Any]) -> None:
        body = json.dumps(payload, allow_nan=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def log_message(self, message_format: str, *args: Any) -> None:
        pass


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind-host", default="10.10.10.1")
    parser.add_argument("--port", type=int, default=8082)
    parser.add_argument("--allowed-client", action="append", default=None)
    parser.add_argument(
        "--joint-state-topic",
        default=DEFAULT_JOINT_STATE_TOPIC,
        help="Measured hardware joint-state topic; avoid republished /joint_states",
    )
    parser.add_argument("--gripper-state-topic", default="/franka_gripper/joint_states")
    parser.add_argument("--max-age-seconds", type=float, default=1.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    bind_address = ipaddress.IPv4Address(args.bind_host)
    if bind_address.is_unspecified or bind_address.is_multicast:
        raise ValueError("--bind-host must identify a specific local IPv4 interface")
    allowed_clients = set(args.allowed_client or ["10.10.10.2"])
    for client in allowed_clients:
        ipaddress.IPv4Address(client)
    if not 1 <= args.port <= 65535:
        raise ValueError("--port must be between 1 and 65535")
    cache = StateCache(args.max_age_seconds, args.joint_state_topic, args.gripper_state_topic)

    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState

    rclpy.init()
    node = rclpy.create_node("fr3_readonly_state_server")
    server = None
    thread = None
    try:
        def receive_joints(message: Any) -> None:
            cache.update("joints", message, node.get_clock().now().nanoseconds)

        def receive_gripper(message: Any) -> None:
            cache.update("gripper", message, node.get_clock().now().nanoseconds)

        def refresh_publishers() -> None:
            cache.update_publishers("joints", node.get_publishers_info_by_topic(args.joint_state_topic))
            cache.update_publishers("gripper", node.get_publishers_info_by_topic(args.gripper_state_topic))

        node.create_subscription(JointState, args.joint_state_topic, receive_joints, qos_profile_sensor_data)
        node.create_subscription(JointState, args.gripper_state_topic, receive_gripper, qos_profile_sensor_data)
        node.create_timer(0.25, refresh_publishers)
        refresh_publishers()
        server = StateHttpServer((args.bind_host, args.port), cache, allowed_clients)
        thread = threading.Thread(target=server.serve_forever, kwargs={"poll_interval": 0.2}, daemon=True)
        thread.start()
        print(f"Read-only FR3 state: http://{args.bind_host}:{args.port}{STATE_PATH}", flush=True)
        print(f"Allowed clients: {sorted(allowed_clients)}", flush=True)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if server is not None:
            if thread is not None and thread.is_alive():
                server.shutdown()
                thread.join(timeout=2.0)
            server.server_close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
