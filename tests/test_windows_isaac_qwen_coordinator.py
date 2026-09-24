#!/usr/bin/env python3

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "windows_vlm_bridge"))

from isaac_qwen_coordinator import NoAction, validate_action  # noqa: E402


def args():
    return SimpleNamespace(
        min_confidence=0.70,
        max_delta=0.03,
        max_total_delta=0.03,
        max_moving_joints=1,
    )


def action(delta=0.03, side="right", confidence=0.95):
    return {
        "action": "joint_deltas",
        "deltas": {"fr3_joint3": delta},
        "confidence": confidence,
        "observation": {"red_marker_side": side},
    }


class CoordinatorValidationTest(unittest.TestCase):
    def test_positive_and_negative_actions_pass(self) -> None:
        validate_action(action(), args())
        validate_action(action(-0.03, "left"), args())

    def test_unclear_is_a_safe_no_action(self) -> None:
        with self.assertRaises(NoAction):
            validate_action(action(0.0, "unclear", 0.0), args())

    def test_low_confidence_is_a_safe_no_action(self) -> None:
        with self.assertRaises(NoAction):
            validate_action(action(confidence=0.69), args())

    def test_unknown_joint_is_rejected(self) -> None:
        candidate = action()
        candidate["deltas"] = {"bad_joint": 0.03}
        with self.assertRaisesRegex(RuntimeError, "Unknown FR3 joint"):
            validate_action(candidate, args())

    def test_excessive_delta_is_rejected(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "max-delta"):
            validate_action(action(0.031), args())

    def test_sign_mismatch_is_rejected(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "non-positive"):
            validate_action(action(-0.03, "right"), args())


if __name__ == "__main__":
    unittest.main()
