#!/usr/bin/env python3
"""Offline validation and HTTP tests; no ROS installation or robot required."""

from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest import mock
from urllib.error import HTTPError
from urllib.request import ProxyHandler, Request, build_opener
import json


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from fr3_readonly_state_server import (  # noqa: E402
    FINGER_JOINTS,
    JOINTS,
    StateCache,
    StateHttpServer,
    parse_args,
)


ROS_NOW = 1_000_000_000_000


def endpoint(gid: int = 1) -> SimpleNamespace:
    return SimpleNamespace(node_name="joint_state_broadcaster", node_namespace="/", endpoint_gid=[gid] * 24)


def message(names=JOINTS, positions=None, stamp_ns=ROS_NOW) -> SimpleNamespace:
    return SimpleNamespace(
        name=list(names),
        position=list(positions if positions is not None else range(len(names))),
        header=SimpleNamespace(stamp=SimpleNamespace(sec=stamp_ns // 1_000_000_000, nanosec=stamp_ns % 1_000_000_000)),
    )


class StateCacheTest(unittest.TestCase):
    def setUp(self) -> None:
        self.clock = mock.patch("fr3_readonly_state_server.time.monotonic", return_value=100.0)
        self.now = self.clock.start()
        self.addCleanup(self.clock.stop)
        self.cache = StateCache()
        self.cache.update_publishers("joints", [endpoint()])

    def test_default_cli_and_cache_use_hardware_joint_state_topic(self) -> None:
        with mock.patch.object(sys, "argv", ["fr3_readonly_state_server.py"]):
            args = parse_args()
        self.assertEqual(args.joint_state_topic, "/franka/joint_states")
        self.cache.update("joints", message(), ROS_NOW)
        status, state = self.cache.snapshot()
        self.assertEqual(status, 200)
        self.assertEqual(state["joint_state_topic"], args.joint_state_topic)

    def test_joint_state_topic_can_be_overridden_for_namespaced_hardware(self) -> None:
        topic = "/robot1/franka/joint_states"
        with mock.patch.object(
            sys, "argv", ["fr3_readonly_state_server.py", "--joint-state-topic", topic]
        ):
            args = parse_args()
        cache = StateCache(joint_topic=args.joint_state_topic)
        cache.update_publishers("joints", [endpoint()])
        cache.update("joints", message(), ROS_NOW)
        status, state = cache.snapshot()
        self.assertEqual(status, 200)
        self.assertEqual(state["joint_state_topic"], topic)

    def test_named_order_and_raw_gripper_width(self) -> None:
        self.cache.update("joints", message(JOINTS[::-1], range(7)), ROS_NOW)
        self.cache.update_publishers("gripper", [endpoint(2)])
        self.cache.update("gripper", message(FINGER_JOINTS, [0.02, 0.03]), ROS_NOW)
        status, state = self.cache.snapshot()
        self.assertEqual(status, 200)
        self.assertEqual(state["joint_names"], list(JOINTS))
        self.assertEqual(state["joint_positions_rad"], list(reversed(range(7))))
        self.assertAlmostEqual(state["gripper_width_m"], 0.05)
        self.assertEqual(state["joint_state_publisher"]["node_name"], "joint_state_broadcaster")

    def test_bad_new_message_invalidates_previous_state(self) -> None:
        invalid_messages = [
            message(positions=[float("nan")] * 7),
            message(positions=[float("inf")] * 7),
            message(positions=[True] * 7),
            message(names=JOINTS[:-1]),
            message(names=[JOINTS[0]] * 7),
            message(positions=[0.0]),
            message(stamp_ns=0),
            message(stamp_ns=ROS_NOW - 2_000_000_000),
            message(stamp_ns=ROS_NOW + 1_000_000_000),
        ]
        for invalid in invalid_messages:
            with self.subTest(message=invalid):
                self.cache.update("joints", message(), ROS_NOW)
                self.assertEqual(self.cache.snapshot()[0], 200)
                self.cache.update("joints", invalid, ROS_NOW)
                self.assertEqual(self.cache.snapshot()[0], 503)

    def test_source_age_counts_before_receipt(self) -> None:
        self.cache.update("joints", message(stamp_ns=ROS_NOW - 800_000_000), ROS_NOW)
        self.now.return_value = 100.3
        self.assertEqual(self.cache.snapshot()[0], 503)

    def test_stale_discovery_blocks_even_fresh_data(self) -> None:
        self.now.return_value = 100.9
        self.cache.update("joints", message(), ROS_NOW)
        self.now.return_value = 101.1
        status, payload = self.cache.snapshot()
        self.assertEqual(status, 503)
        self.assertIn("discovery", payload["reason"])

    def test_publisher_ambiguity_and_restart_require_new_sample(self) -> None:
        self.cache.update("joints", message(), ROS_NOW)
        for publishers in ([], [endpoint(), endpoint(2)], [endpoint(3)], [endpoint(0)]):
            with self.subTest(publishers=publishers):
                self.cache.update_publishers("joints", publishers)
                self.assertEqual(self.cache.snapshot()[0], 503)
        self.cache.update_publishers("joints", [endpoint(3)])
        self.assertEqual(self.cache.snapshot()[0], 503)
        self.cache.update("joints", message(), ROS_NOW)
        self.assertEqual(self.cache.snapshot()[0], 200)

    def test_missing_and_bad_gripper_leave_arm_state_available(self) -> None:
        self.cache.update("joints", message(), ROS_NOW)
        self.assertIsNone(self.cache.snapshot()[1]["gripper_width_m"])
        self.cache.update_publishers("gripper", [endpoint(2)])
        for positions in ([0.02, -0.01], [1e308, 1e308], [float("nan"), 0.01]):
            self.cache.update("gripper", message(FINGER_JOINTS, positions), ROS_NOW)
            status, state = self.cache.snapshot()
            self.assertEqual(status, 200)
            self.assertIsNone(state["gripper_width_m"])
            self.assertIsNotNone(state["gripper_unavailable_reason"])


class StateHttpTest(unittest.TestCase):
    def setUp(self) -> None:
        self.cache = StateCache()
        self.server = StateHttpServer(("127.0.0.1", 0), self.cache, {"127.0.0.1"})
        self.thread = threading.Thread(target=self.server.serve_forever, kwargs={"poll_interval": 0.01}, daemon=True)
        self.thread.start()
        self.url = f"http://127.0.0.1:{self.server.server_port}"
        self.opener = build_opener(ProxyHandler({}))
        self.addCleanup(self.close_server)

    def close_server(self) -> None:
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2.0)

    def test_get_snapshot_and_failures(self) -> None:
        with self.assertRaises(HTTPError) as error:
            self.opener.open(self.url + "/v1/robot-state", timeout=2.0)
        self.assertEqual(error.exception.code, 503)
        error.exception.close()
        self.cache.update_publishers("joints", [endpoint()])
        self.cache.update("joints", message(), ROS_NOW)
        with self.opener.open(self.url + "/v1/robot-state", timeout=2.0) as response:
            self.assertEqual(response.headers["Cache-Control"], "no-store")
            self.assertEqual(json.load(response)["joint_positions_rad"], list(range(7)))
        for path, method, expected in (("/missing", "GET", 404), ("/v1/robot-state", "POST", 501)):
            with self.assertRaises(HTTPError) as error:
                self.opener.open(Request(self.url + path, method=method), timeout=2.0)
            self.assertEqual(error.exception.code, expected)
            error.exception.close()

    def test_client_ip_is_restricted(self) -> None:
        self.server.allowed_clients = {"10.10.10.2"}
        with self.assertRaises(HTTPError) as error:
            self.opener.open(self.url + "/v1/robot-state", timeout=2.0)
        self.assertEqual(error.exception.code, 403)
        error.exception.close()


if __name__ == "__main__":
    unittest.main()
