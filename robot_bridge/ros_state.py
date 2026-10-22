"""Independent ROS state monitor sharing the strict read-only telemetry cache."""

from __future__ import annotations

import threading


class LiveStateMonitor:
    def __init__(self, *, joint_topic, gripper_topic, max_age_seconds=1.0):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import JointState
        from fr3_readonly_state_server import StateCache

        self.cache = StateCache(max_age_seconds, joint_topic, gripper_topic)
        self.node = rclpy.create_node("fr3_cosmos_state_monitor")
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

        def receive(kind, message):
            self.cache.update(kind, message, self.node.get_clock().now().nanoseconds)

        def refresh():
            self.cache.update_publishers("joints", self.node.get_publishers_info_by_topic(joint_topic))
            self.cache.update_publishers("gripper", self.node.get_publishers_info_by_topic(gripper_topic))

        self.node.create_subscription(JointState, joint_topic, lambda msg: receive("joints", msg), qos_profile_sensor_data)
        self.node.create_subscription(JointState, gripper_topic, lambda msg: receive("gripper", msg), qos_profile_sensor_data)
        self.node.create_timer(0.25, refresh)
        refresh()
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def snapshot(self):
        return self.cache.snapshot()

    def close(self):
        self.executor.shutdown(timeout_sec=2.0)
        self.thread.join(timeout=2.0)
        self.node.destroy_node()
