#!/usr/bin/env python3

import builtins
import contextlib
import io
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import fr3_session_control as session_control  # noqa: E402


SESSION_ID = "aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa"


def arm_args():
    return SimpleNamespace(
        joint=["fr3_joint1", "fr3_joint3"],
        max_moving_joints=2,
        max_steps=2,
        max_duration=60.0,
        max_delta=0.01,
        max_total_delta=0.02,
        max_cumulative_delta=0.04,
        session_id=SESSION_ID,
        socket=Path("/tmp/test-fr3-session.sock"),
        timeout=5.0,
    )


class SessionControlTest(unittest.TestCase):
    def test_wrong_confirmation_creates_no_authority(self) -> None:
        with mock.patch.object(builtins, "input", return_value="ARM wrong"), mock.patch.object(
            session_control, "send_local_request"
        ) as send, contextlib.redirect_stdout(io.StringIO()):
            result = session_control.arm_session(arm_args())

        self.assertEqual(result, 1)
        send.assert_not_called()

    def test_exact_confirmation_sends_bounded_arm_request(self) -> None:
        response = {
            "local_status": 201,
            "session_id": SESSION_ID,
            "status": "ARMED",
            "next_sequence": 1,
            "completed_steps": 0,
            "cumulative_absolute_delta": 0.0,
            "remaining_seconds": 60.0,
            "stop_requested": False,
            "limits": {
                "allowed_joints": ["fr3_joint1", "fr3_joint3"],
                "max_moving_joints": 2,
                "max_steps": 2,
                "max_delta": 0.01,
                "max_total_delta": 0.02,
                "max_cumulative_delta": 0.04,
            },
        }
        expected = "ARM"
        with mock.patch.object(builtins, "input", return_value=expected), mock.patch.object(
            session_control, "send_local_request", return_value=response
        ) as send, contextlib.redirect_stdout(io.StringIO()):
            result = session_control.arm_session(arm_args())

        self.assertEqual(result, 0)
        send.assert_called_once_with(
            Path("/tmp/test-fr3-session.sock"),
            {
                "operation": "arm_session",
                "session_id": SESSION_ID,
                "allowed_joints": ["fr3_joint1", "fr3_joint3"],
                "max_moving_joints": 2,
                "max_steps": 2,
                "max_duration_seconds": 60.0,
                "max_delta": 0.01,
                "max_total_delta": 0.02,
                "max_cumulative_delta": 0.04,
            },
            timeout=5.0,
        )

    def test_status_defaults_to_current_session(self) -> None:
        response = {
            "local_status": 200,
            "session_id": SESSION_ID,
            "status": "ARMED",
            "next_sequence": 1,
            "completed_steps": 0,
            "cumulative_absolute_delta": 0.0,
            "remaining_seconds": 30.0,
            "stop_requested": False,
            "limits": {
                "allowed_joints": ["fr3_joint3"],
                "max_moving_joints": 1,
                "max_steps": 2,
                "max_delta": 0.03,
                "max_total_delta": 0.03,
                "max_cumulative_delta": 0.06,
            },
        }
        args = SimpleNamespace(
            session_id="",
            socket=Path("/tmp/test-fr3-session.sock"),
            timeout=5.0,
        )
        with mock.patch.object(
            session_control, "send_local_request", return_value=response
        ) as send, contextlib.redirect_stdout(io.StringIO()):
            result = session_control.session_status(args)

        self.assertEqual(result, 0)
        send.assert_called_once_with(
            Path("/tmp/test-fr3-session.sock"),
            {"operation": "session_status"},
            timeout=5.0,
        )

    def test_disarm_defaults_to_current_session(self) -> None:
        response = {
            "local_status": 200,
            "status": "SESSION_STOPPED",
            "session": {
                "session_id": SESSION_ID,
                "status": "DISARMED",
                "next_sequence": 1,
                "completed_steps": 0,
                "cumulative_absolute_delta": 0.0,
                "remaining_seconds": 30.0,
                "stop_requested": False,
                "limits": {
                    "allowed_joints": ["fr3_joint3"],
                    "max_moving_joints": 1,
                    "max_steps": 2,
                    "max_delta": 0.03,
                    "max_total_delta": 0.03,
                    "max_cumulative_delta": 0.06,
                },
            },
        }
        args = SimpleNamespace(
            session_id="",
            socket=Path("/tmp/test-fr3-session.sock"),
            timeout=5.0,
        )
        with mock.patch.object(
            session_control, "send_local_request", return_value=response
        ) as send, contextlib.redirect_stdout(io.StringIO()):
            result = session_control.disarm_session(args)

        self.assertEqual(result, 0)
        send.assert_called_once_with(
            Path("/tmp/test-fr3-session.sock"),
            {"operation": "disarm_session"},
            timeout=5.0,
        )


if __name__ == "__main__":
    unittest.main()
