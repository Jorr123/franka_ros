#!/usr/bin/env python3

from pathlib import Path
import json
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "windows_vlm_bridge"))

import bounded_joint_action_session as client  # noqa: E402


def args(tmp: Path, **overrides):
    values = {
        "action_json_file": [],
        "session_id": "",
        "ubuntu_health_url": "http://10.10.10.1:8080/health",
        "ubuntu_step_url": "http://10.10.10.1:8080/v1/sessions/step",
        "ubuntu_stop_url": "http://10.10.10.1:8080/v1/sessions/stop",
        "ubuntu_timeout": 5.0,
        "shared_secret_env": "FR3_BRIDGE_SECRET",
        "min_confidence": 0.70,
        "max_moving_joints": 2,
        "max_delta": 0.01,
        "max_total_delta": 0.02,
        "max_cumulative_delta": 0.04,
        "max_session_seconds": 50.0,
        "log_file": tmp / "session.jsonl",
        "run_armed_session": False,
    }
    values.update(overrides)
    return SimpleNamespace(**values)


def write_action(path: Path, deltas: dict[str, float]) -> Path:
    action = {"action": "joint_deltas", "deltas": deltas, "confidence": 1.0}
    path.write_text(json.dumps(action) + "\n", encoding="utf-8")
    return path


class BoundedJointActionSessionTest(unittest.TestCase):
    def test_two_action_dry_run_uses_no_network(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            tmp = Path(directory)
            forward = write_action(
                tmp / "forward.json",
                {"fr3_joint1": 0.01, "fr3_joint3": -0.01},
            )
            reverse = write_action(
                tmp / "reverse.json",
                {"fr3_joint1": -0.01, "fr3_joint3": 0.01},
            )
            options = args(tmp, action_json_file=[forward, reverse])
            with mock.patch.object(client, "get_json") as get_json:
                code, summary = client.run(options)
            self.assertEqual(code, 0)
            self.assertEqual(summary["status"], "DRY_RUN_COMPLETE")
            self.assertEqual(summary["actions_validated"], 2)
            self.assertAlmostEqual(summary["cumulative_delta"], 0.04)
            self.assertFalse(summary["ubuntu_contacted"])
            self.assertFalse(summary["robot_execution"])
            get_json.assert_not_called()

    def test_unknown_joint_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            options = args(Path(directory))
            with self.assertRaisesRegex(RuntimeError, "Unknown FR3 joint"):
                client.validate_action(
                    {
                        "action": "joint_deltas",
                        "deltas": {"not_a_joint": 0.01},
                        "confidence": 1.0,
                    },
                    options,
                )

    def test_per_joint_limit_is_enforced(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            options = args(Path(directory))
            with self.assertRaisesRegex(RuntimeError, "max-delta"):
                client.validate_action(
                    {
                        "action": "joint_deltas",
                        "deltas": {"fr3_joint1": 0.011},
                        "confidence": 1.0,
                    },
                    options,
                )

    def test_step_l1_limit_is_enforced(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            options = args(Path(directory), max_total_delta=0.015)
            with self.assertRaisesRegex(RuntimeError, "max-total-delta"):
                client.validate_action(
                    {
                        "action": "joint_deltas",
                        "deltas": {"fr3_joint1": 0.01, "fr3_joint3": -0.01},
                        "confidence": 1.0,
                    },
                    options,
                )

    def test_moving_joint_count_is_enforced(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            options = args(Path(directory), max_moving_joints=1)
            with self.assertRaisesRegex(RuntimeError, "max-moving-joints"):
                client.validate_action(
                    {
                        "action": "joint_deltas",
                        "deltas": {"fr3_joint1": 0.01, "fr3_joint3": -0.01},
                        "confidence": 1.0,
                    },
                    options,
                )

    def test_file_sequence_cumulative_l1_limit_is_enforced(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            tmp = Path(directory)
            action_files = [
                write_action(
                    tmp / f"action_{index}.json",
                    {"fr3_joint1": 0.01, "fr3_joint3": -0.01},
                )
                for index in range(3)
            ]
            options = args(tmp, action_json_file=action_files)
            with self.assertRaisesRegex(RuntimeError, "max-cumulative-delta"):
                client.run(options)


if __name__ == "__main__":
    unittest.main()
