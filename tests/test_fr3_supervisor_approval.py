#!/usr/bin/env python3

import io
import os
from pathlib import Path
import stat
import sys
import tempfile
import threading
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "scripts"
sys.path.insert(0, str(SCRIPTS))

from fr3_ai_supervisor import create_approval_server  # noqa: E402
import fr3_supervisor_approve as approval  # noqa: E402
from tests.test_fr3_action_protocol import make_core, submit_plan  # noqa: E402


class ApprovalSocketTest(unittest.TestCase):
    def test_socket_is_private_and_plan_is_single_use(self) -> None:
        core = make_core(execution_enabled=True)
        _, plan = submit_plan(core)

        with tempfile.TemporaryDirectory() as directory:
            socket_path = Path(directory) / "approval.sock"
            server = create_approval_server(socket_path, core)
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            try:
                mode = stat.S_IMODE(os.stat(socket_path).st_mode)
                self.assertEqual(mode, 0o600)

                status = approval.send_local_request(
                    socket_path,
                    {"operation": "status", "request_id": plan["request_id"]},
                    timeout=2.0,
                )
                self.assertEqual(status["local_status"], 200)
                self.assertEqual(status["status"], "READY_FOR_LOCAL_APPROVAL")

                result = approval.send_local_request(
                    socket_path,
                    {
                        "operation": "execute",
                        "request_id": plan["request_id"],
                        "action_sha256": plan["action_sha256"],
                    },
                    timeout=2.0,
                )
                self.assertEqual(result["local_status"], 200)
                self.assertEqual(result["status"], "VERIFIED_SUCCESS")

                reused = approval.send_local_request(
                    socket_path,
                    {
                        "operation": "execute",
                        "request_id": plan["request_id"],
                        "action_sha256": plan["action_sha256"],
                    },
                    timeout=2.0,
                )
                self.assertEqual(reused["local_status"], 409)
                self.assertEqual(reused["code"], "PLAN_NOT_READY")
            finally:
                server.shutdown()
                server.server_close()
                thread.join(timeout=2.0)
                socket_path.unlink(missing_ok=True)


class ApprovalCliTest(unittest.TestCase):
    REQUEST_ID = "12345678-1234-4234-8234-123456789abc"
    DIGEST = "a" * 64

    @classmethod
    def ready_status(cls) -> dict:
        return {
            "local_status": 200,
            "status": "READY_FOR_LOCAL_APPROVAL",
            "request_id": cls.REQUEST_ID,
            "action_sha256": cls.DIGEST,
            "plan_expires_at_unix_ms": 4_000_000_000_000,
            "validated_deltas": {"fr3_joint3": 0.03},
            "current_joint_positions": {"fr3_joint3": -0.3},
            "target_joint_positions": {"fr3_joint3": -0.27},
            "planning": {
                "result": "SUCCESS",
                "point_count": 6,
                "duration_seconds": 0.5,
            },
            "result": None,
        }

    def test_inspection_does_not_request_execution(self) -> None:
        argv = ["approve", "--request-id", self.REQUEST_ID]
        with mock.patch.object(sys, "argv", argv), mock.patch.object(
            approval, "send_local_request", return_value=self.ready_status()
        ) as send, mock.patch("sys.stdout", new_callable=io.StringIO):
            result = approval.main()
        self.assertEqual(result, 0)
        self.assertEqual(send.call_count, 1)
        self.assertEqual(send.call_args.args[1]["operation"], "status")

    def test_wrong_typed_confirmation_cancels_execution(self) -> None:
        argv = [
            "approve",
            "--request-id",
            self.REQUEST_ID,
            "--action-sha256",
            self.DIGEST,
            "--execute",
        ]
        with mock.patch.object(sys, "argv", argv), mock.patch.object(
            approval, "send_local_request", return_value=self.ready_status()
        ) as send, mock.patch("builtins.input", return_value="EXECUTE"), mock.patch(
            "sys.stdout", new_callable=io.StringIO
        ):
            result = approval.main()
        self.assertEqual(result, 1)
        self.assertEqual(send.call_count, 1)

    def test_exact_confirmation_requests_one_execution(self) -> None:
        argv = [
            "approve",
            "--request-id",
            self.REQUEST_ID,
            "--action-sha256",
            self.DIGEST,
            "--execute",
        ]
        responses = [
            self.ready_status(),
            {
                "local_status": 200,
                "status": "VERIFIED_SUCCESS",
                "code": "VERIFIED_SUCCESS",
            },
        ]
        with mock.patch.object(sys, "argv", argv), mock.patch.object(
            approval, "send_local_request", side_effect=responses
        ) as send, mock.patch(
            "builtins.input", return_value=f"EXECUTE {self.REQUEST_ID}"
        ), mock.patch("sys.stdout", new_callable=io.StringIO):
            result = approval.main()
        self.assertEqual(result, 0)
        self.assertEqual(send.call_count, 2)
        self.assertEqual(send.call_args_list[1].args[1]["operation"], "execute")


if __name__ == "__main__":
    unittest.main()
