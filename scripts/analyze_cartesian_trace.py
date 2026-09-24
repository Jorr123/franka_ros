#!/usr/bin/env python3
"""Analyze a recorded FR3 Cartesian execution without publishing commands."""

import argparse
from collections import deque
import math
from pathlib import Path
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


CONTROLLER_TOPIC = "/fr3_arm_controller/controller_state"
TF_TOPICS = {"/tf", "/tf_static"}


def transform_matrix(transform) -> np.ndarray:
    x = transform.rotation.x
    y = transform.rotation.y
    z = transform.rotation.z
    w = transform.rotation.w
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0.0:
        raise ValueError("TF contains a zero-length quaternion.")
    x, y, z, w = (value / norm for value in (x, y, z, w))

    matrix = np.eye(4)
    matrix[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    matrix[:3, 3] = [
        transform.translation.x,
        transform.translation.y,
        transform.translation.z,
    ]
    return matrix


def lookup_matrix(transforms, source: str, target: str):
    graph = {}
    for (parent, child), matrix in transforms.items():
        graph.setdefault(parent, []).append((child, matrix))
        graph.setdefault(child, []).append((parent, np.linalg.inv(matrix)))

    queue = deque([(source, np.eye(4))])
    visited = {source}
    while queue:
        frame, source_to_frame = queue.popleft()
        if frame == target:
            return source_to_frame
        for neighbor, frame_to_neighbor in graph.get(frame, []):
            if neighbor in visited:
                continue
            visited.add(neighbor)
            queue.append((neighbor, source_to_frame @ frame_to_neighbor))
    return None


def point_seconds(point) -> float:
    return point.time_from_start.sec + point.time_from_start.nanosec * 1e-9


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Analyze controller tracking and TCP motion in an ROS bag."
    )
    parser.add_argument("bag", type=Path)
    parser.add_argument("--base-frame", default="base")
    parser.add_argument("--ee-frame", default="fr3_hand_tcp")
    parser.add_argument("--motion-velocity-threshold", type=float, default=1e-5)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.bag.exists():
        print(f"Bag does not exist: {args.bag}", file=sys.stderr)
        return 2

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {
        topic.name: topic.type for topic in reader.get_all_topics_and_types()
    }
    if CONTROLLER_TOPIC not in topic_types:
        print(f"Bag is missing {CONTROLLER_TOPIC}.", file=sys.stderr)
        return 2

    message_types = {
        topic: get_message(type_name)
        for topic, type_name in topic_types.items()
        if topic == CONTROLLER_TOPIC or topic in TF_TOPICS
    }
    controller_records = []
    tcp_records = []
    transforms = {}

    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        if topic not in message_types:
            continue
        message = deserialize_message(data, message_types[topic])
        timestamp = timestamp_ns * 1e-9

        if topic == CONTROLLER_TOPIC:
            reference = (
                message.reference
                if message.reference.positions
                else message.desired
            )
            feedback = (
                message.feedback
                if message.feedback.positions
                else message.actual
            )
            controller_records.append(
                {
                    "time": timestamp,
                    "names": tuple(message.joint_names),
                    "reference": tuple(reference.positions),
                    "reference_velocity": tuple(reference.velocities),
                    "feedback": tuple(feedback.positions),
                    "feedback_velocity": tuple(feedback.velocities),
                    "error": tuple(message.error.positions),
                    "effort": tuple(message.output.effort),
                    "trajectory_time": point_seconds(reference),
                }
            )
            continue

        for stamped in message.transforms:
            parent = stamped.header.frame_id.lstrip("/")
            child = stamped.child_frame_id.lstrip("/")
            transforms[(parent, child)] = transform_matrix(stamped.transform)
        matrix = lookup_matrix(
            transforms,
            args.base_frame.lstrip("/"),
            args.ee_frame.lstrip("/"),
        )
        if matrix is not None:
            tcp_records.append((timestamp, tuple(matrix[:3, 3])))

    if not controller_records:
        print("Bag contains no controller-state messages.", file=sys.stderr)
        return 1

    names = controller_records[0]["names"]
    active_indices = []
    for index, record in enumerate(controller_records):
        velocities = record["reference_velocity"]
        if velocities and max(abs(value) for value in velocities) > (
            args.motion_velocity_threshold
        ):
            active_indices.append(index)

    if not active_indices:
        print("No moving trajectory reference was found.", file=sys.stderr)
        return 1

    start_index = active_indices[0]
    target_index = min(len(controller_records) - 1, active_indices[-1] + 1)
    start_time = controller_records[start_index]["time"]
    target_time = controller_records[target_index]["time"]

    post_indices = [
        index
        for index, record in enumerate(controller_records)
        if target_time <= record["time"] <= target_time + 3.2
    ]
    end_index = post_indices[-1] if post_indices else target_index
    analysis_records = controller_records[start_index : end_index + 1]

    start = controller_records[start_index]
    target = controller_records[target_index]
    end = controller_records[end_index]
    print("FR3 Cartesian trace analysis")
    print()
    print(f"Controller samples:       {len(controller_records)}")
    print(f"Trajectory motion start:  {start_time:.6f}")
    print(f"Trajectory nominal end:   {target_time:.6f}")
    print(f"Analyzed through:         {end['time']:.6f}")
    print(f"Nominal moving duration:  {target_time - start_time:.3f} s")
    print()
    print(
        "joint        target delta   measured delta   start error   "
        "target error   final error   max effort"
    )

    for joint_index, name in enumerate(names):
        target_delta = (
            target["reference"][joint_index] - start["reference"][joint_index]
        )
        measured_delta = end["feedback"][joint_index] - start["feedback"][joint_index]
        start_error = (
            start["reference"][joint_index] - start["feedback"][joint_index]
        )
        target_error = (
            target["reference"][joint_index]
            - target["feedback"][joint_index]
        )
        final_error = (
            target["reference"][joint_index] - end["feedback"][joint_index]
        )
        maximum_effort = max(
            abs(record["effort"][joint_index])
            for record in analysis_records
            if len(record["effort"]) == len(names)
        )
        print(
            f"{name:<11} {target_delta:+13.6f} {measured_delta:+16.6f} "
            f"{start_error:+13.6f} {target_error:+14.6f} "
            f"{final_error:+13.6f} {maximum_effort:12.6f}"
        )

    window_tcp = [
        (timestamp, position)
        for timestamp, position in tcp_records
        if start_time <= timestamp <= end["time"]
    ]
    if len(window_tcp) < 2:
        print("\nCould not reconstruct enough TCP samples from TF.")
        return 0

    start_tcp = min(window_tcp, key=lambda item: abs(item[0] - start_time))
    nominal_end_tcp = min(
        window_tcp, key=lambda item: abs(item[0] - target_time)
    )
    final_tcp = window_tcp[-1]
    minimum_z = min(window_tcp, key=lambda item: item[1][2])
    maximum_z = max(window_tcp, key=lambda item: item[1][2])

    def millimeter_delta(first, second):
        return tuple((second[i] - first[i]) * 1000.0 for i in range(3))

    nominal_delta = millimeter_delta(start_tcp[1], nominal_end_tcp[1])
    final_delta = millimeter_delta(start_tcp[1], final_tcp[1])
    dip = (minimum_z[1][2] - start_tcp[1][2]) * 1000.0
    rise = (maximum_z[1][2] - start_tcp[1][2]) * 1000.0

    print()
    print(f"TCP samples in window:    {len(window_tcp)}")
    print(
        "TCP at nominal end:      "
        f"dx={nominal_delta[0]:+.3f}, dy={nominal_delta[1]:+.3f}, "
        f"dz={nominal_delta[2]:+.3f} mm"
    )
    print(
        "TCP after goal window:   "
        f"dx={final_delta[0]:+.3f}, dy={final_delta[1]:+.3f}, "
        f"dz={final_delta[2]:+.3f} mm"
    )
    print(
        f"Lowest Z excursion:       {dip:+.3f} mm at "
        f"{minimum_z[0] - start_time:+.3f} s"
    )
    print(
        f"Highest Z excursion:      {rise:+.3f} mm at "
        f"{maximum_z[0] - start_time:+.3f} s"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
