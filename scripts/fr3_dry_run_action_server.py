#!/usr/bin/env python3
"""Validate Windows action proposals against live ROS state; execution is absent."""

from __future__ import annotations

import argparse
import ipaddress
from pathlib import Path
import sys
import threading

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from robot_bridge.protocol import Limits, MODE, PATH
from robot_bridge.server import DryRunServer
from fr3_readonly_state_server import StateCache


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind-host", default="10.10.10.1")
    parser.add_argument("--port", type=int, default=8083)
    parser.add_argument("--allowed-client", action="append")
    parser.add_argument("--joint-state-topic", default="/franka/joint_states")
    parser.add_argument("--gripper-state-topic", default="/franka_gripper/joint_states")
    parser.add_argument("--log-file", type=Path, default=Path("log/dry_run_action_proposals.jsonl"))
    args = parser.parse_args()
    address = ipaddress.IPv4Address(args.bind_host)
    if address.is_unspecified or address.is_multicast or not 1 <= args.port <= 65535:
        parser.error("Use a specific local IPv4 interface and a port in 1-65535")
    allowed = set(args.allowed_client or ["10.10.10.2"])
    for client in allowed:
        ipaddress.IPv4Address(client)
    limits = Limits()
    cache = StateCache(limits.max_state_age_seconds, args.joint_state_topic, args.gripper_state_topic)

    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState

    rclpy.init()
    node = rclpy.create_node("fr3_dry_run_action_server")
    server = None
    thread = None
    try:
        def receive_joints(message):
            cache.update("joints", message, node.get_clock().now().nanoseconds)

        def receive_gripper(message):
            cache.update("gripper", message, node.get_clock().now().nanoseconds)

        def refresh_publishers():
            cache.update_publishers("joints", node.get_publishers_info_by_topic(args.joint_state_topic))
            cache.update_publishers("gripper", node.get_publishers_info_by_topic(args.gripper_state_topic))

        node.create_subscription(JointState, args.joint_state_topic, receive_joints, qos_profile_sensor_data)
        node.create_subscription(JointState, args.gripper_state_topic, receive_gripper, qos_profile_sensor_data)
        node.create_timer(0.25, refresh_publishers)
        refresh_publishers()
        server = DryRunServer(
            (args.bind_host, args.port), snapshot=cache.snapshot,
            allowed_clients=allowed, audit_log=args.log_file, limits=limits,
        )
        thread = threading.Thread(target=server.serve_forever, kwargs={"poll_interval": 0.2}, daemon=True)
        thread.start()
        print(f"{MODE}: http://{args.bind_host}:{args.port}{PATH}", flush=True)
        print(f"Allowed clients: {sorted(allowed)} | Robot execution: False", flush=True)
        print(f"Audit log: {args.log_file.resolve()}", flush=True)
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
