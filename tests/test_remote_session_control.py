#!/usr/bin/env python3

import os
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "windows_vlm_bridge"))
sys.path.insert(0, str(ROOT / "scripts"))

from fr3_bounded_session_supervisor import BoundedSessionHttpServer  # noqa: E402
from fr3_remote_session_control import (  # noqa: E402
    arm_session,
    get_health,
    stop_session,
)
from tests.test_bounded_session_supervisor import make_core  # noqa: E402
from tests.test_fr3_action_protocol import SECRET  # noqa: E402


def arm_args() -> SimpleNamespace:
    return SimpleNamespace(
        joint=["fr3_joint1", "fr3_joint3"],
        max_moving_joints=2,
        max_steps=2,
        max_duration=60.0,
        max_delta=0.01,
        max_total_delta=0.02,
        max_cumulative_delta=0.04,
        session_id="aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa",
        timeout=2.0,
        shared_secret_env="FR3_BRIDGE_SECRET",
    )


class RemoteSessionControlIntegrationTest(unittest.TestCase):
    def test_windows_client_arms_and_stops_bounded_session(self) -> None:
        core = make_core(remote_arm_joints=("fr3_joint1", "fr3_joint3"))
        server = BoundedSessionHttpServer(("127.0.0.1", 0), core, {"127.0.0.1"})
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        base_url = f"http://127.0.0.1:{server.server_port}"
        try:
            with mock.patch.dict(
                os.environ, {"FR3_BRIDGE_SECRET": SECRET.decode("utf-8")}, clear=False
            ):
                self.assertEqual(arm_session(arm_args(), base_url), 0)
                health = get_health(base_url, 2.0)
                self.assertTrue(health["session_armed"])
                self.assertEqual(health["session"]["limits"]["max_delta"], 0.01)
                self.assertEqual(
                    health["session"]["limits"]["allowed_joints"],
                    ["fr3_joint1", "fr3_joint3"],
                )

                stop_args = SimpleNamespace(
                    session_id="",
                    timeout=2.0,
                    shared_secret_env="FR3_BRIDGE_SECRET",
                )
                self.assertEqual(stop_session(stop_args, base_url), 0)
                self.assertEqual(get_health(base_url, 2.0)["session"]["status"], "STOPPED")
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2.0)


if __name__ == "__main__":
    unittest.main()
