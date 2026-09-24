#!/usr/bin/env python3

from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest

from PIL import Image


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "windows_vlm_bridge"))

from closed_loop_vla_shadow_session import (  # noqa: E402
    MockVisualEnvironment,
    render_frame,
    run_closed_loop,
)
from isaac_qwen_coordinator import NoAction  # noqa: E402
from static_vla_shadow_session import InferenceResult  # noqa: E402


def args(**overrides):
    values = {
        "joint": "fr3_joint3",
        "min_confidence": 0.70,
        "max_delta": 0.03,
        "max_total_delta": 0.03,
        "max_moving_joints": 1,
        "max_actions": 6,
        "max_cumulative_delta": 0.18,
        "max_session_seconds": 30.0,
        "interval": 0.0,
    }
    values.update(overrides)
    return SimpleNamespace(**values)


def environment(side="right", *, pixels_per_rad=1000.0):
    offset = -90.0 if side == "left" else 90.0
    return MockVisualEnvironment(
        width=640,
        height=480,
        marker_x=320.0 + offset,
        marker_radius=24,
        deadband_px=15.0,
        pixels_per_rad=pixels_per_rad,
    )


def action(side, delta):
    return {
        "action": "joint_deltas",
        "deltas": {"fr3_joint3": delta},
        "confidence": 0.95,
        "observation": {"red_marker_side": side},
    }


def inference(candidate):
    return InferenceResult(
        action=candidate,
        image_sha256="a" * 64,
        mime_type="image/png",
    )


def renderer(_environment, observation, _output_dir):
    return Path(f"frame_{observation:03d}.png")


class ClosedLoopStateMachineTest(unittest.TestCase):
    def test_right_marker_converges_then_requires_no_action(self) -> None:
        env = environment("right")

        def policy(_frame, _remaining):
            if env.side == "centered":
                raise NoAction("centered")
            return inference(action("right", 0.03))

        code, summary = run_closed_loop(
            args(),
            env,
            Path("unused"),
            event_sink=lambda _event: None,
            infer=policy,
            renderer=renderer,
        )
        self.assertEqual(code, 0)
        self.assertEqual(summary["status"], "GOAL_REACHED")
        self.assertEqual(summary["actions_applied"], 3)
        self.assertEqual(summary["observations"], 4)
        self.assertAlmostEqual(summary["cumulative_absolute_delta"], 0.09)
        self.assertAlmostEqual(summary["final_error_px"], 0.0)
        self.assertFalse(summary["robot_execution_enabled"])

    def test_left_marker_converges_with_negative_actions(self) -> None:
        env = environment("left")

        def policy(_frame, _remaining):
            if env.side == "centered":
                raise NoAction("centered")
            return inference(action("left", -0.03))

        code, summary = run_closed_loop(
            args(),
            env,
            Path("unused"),
            event_sink=lambda _event: None,
            infer=policy,
            renderer=renderer,
        )
        self.assertEqual(code, 0)
        self.assertEqual(summary["status"], "GOAL_REACHED")
        self.assertEqual(summary["actions_applied"], 3)
        self.assertAlmostEqual(summary["final_error_px"], 0.0)

    def test_no_action_away_from_center_is_failure(self) -> None:
        env = environment("right")
        code, summary = run_closed_loop(
            args(),
            env,
            Path("unused"),
            event_sink=lambda _event: None,
            infer=lambda _frame, _remaining: (_ for _ in ()).throw(NoAction("unclear")),
            renderer=renderer,
        )
        self.assertEqual(code, 1)
        self.assertEqual(summary["status"], "PREMATURE_NO_ACTION")
        self.assertEqual(summary["actions_applied"], 0)

    def test_perception_mismatch_does_not_update_environment(self) -> None:
        env = environment("right")
        start_x = env.marker_x
        code, summary = run_closed_loop(
            args(),
            env,
            Path("unused"),
            event_sink=lambda _event: None,
            infer=lambda _frame, _remaining: inference(action("left", -0.03)),
            renderer=renderer,
        )
        self.assertEqual(code, 1)
        self.assertEqual(summary["status"], "PERCEPTION_MISMATCH")
        self.assertEqual(env.marker_x, start_x)

    def test_action_that_increases_error_is_rejected(self) -> None:
        env = environment("right", pixels_per_rad=-1000.0)
        start_x = env.marker_x
        code, summary = run_closed_loop(
            args(),
            env,
            Path("unused"),
            event_sink=lambda _event: None,
            infer=lambda _frame, _remaining: inference(action("right", 0.03)),
            renderer=renderer,
        )
        self.assertEqual(code, 1)
        self.assertEqual(summary["status"], "ACTION_DID_NOT_IMPROVE")
        self.assertEqual(env.marker_x, start_x)

    def test_cumulative_limit_stops_before_excess_action(self) -> None:
        env = environment("right")
        code, summary = run_closed_loop(
            args(max_cumulative_delta=0.05),
            env,
            Path("unused"),
            event_sink=lambda _event: None,
            infer=lambda _frame, _remaining: inference(action("right", 0.03)),
            renderer=renderer,
        )
        self.assertEqual(code, 1)
        self.assertEqual(summary["status"], "CUMULATIVE_LIMIT_REACHED")
        self.assertEqual(summary["actions_applied"], 1)
        self.assertAlmostEqual(summary["cumulative_absolute_delta"], 0.03)

    def test_action_limit_fails_when_marker_is_not_centered(self) -> None:
        env = environment("right")
        code, summary = run_closed_loop(
            args(max_actions=2),
            env,
            Path("unused"),
            event_sink=lambda _event: None,
            infer=lambda _frame, _remaining: inference(action("right", 0.03)),
            renderer=renderer,
        )
        self.assertEqual(code, 1)
        self.assertEqual(summary["status"], "ACTION_LIMIT_REACHED")
        self.assertEqual(summary["actions_applied"], 2)
        self.assertEqual(summary["final_ground_truth_side"], "right")

    def test_each_successful_plan_precedes_one_mock_state_update(self) -> None:
        env = environment("right")
        events = []
        marker_positions_at_plan = []

        def policy(_frame, _remaining):
            if env.side == "centered":
                raise NoAction("centered")
            return inference(action("right", 0.03))

        def plan(_action, _digest, frame_id, _remaining):
            marker_positions_at_plan.append(env.marker_x)
            return {
                "request_id": f"request-{len(marker_positions_at_plan)}",
                "action_sha256": "b" * 64,
                "planning": {"result": "SUCCESS", "point_count": 6},
                "frame_id": frame_id,
            }

        code, summary = run_closed_loop(
            args(),
            env,
            Path("unused"),
            event_sink=events.append,
            infer=policy,
            submit_plan=plan,
            session_id="12345678-1234-4234-8234-123456789abc",
            renderer=renderer,
        )
        self.assertEqual(code, 0)
        self.assertEqual(summary["status"], "GOAL_REACHED")
        self.assertEqual(summary["plans_ready"], 3)
        self.assertEqual(summary["actions_applied"], 3)
        self.assertEqual(marker_positions_at_plan, [410.0, 380.0, 350.0])
        ordered = [
            event["event"]
            for event in events
            if event["event"] in {"PLAN_READY", "MOCK_ACTION_APPLIED"}
        ]
        self.assertEqual(
            ordered,
            [
                "PLAN_READY",
                "MOCK_ACTION_APPLIED",
                "PLAN_READY",
                "MOCK_ACTION_APPLIED",
                "PLAN_READY",
                "MOCK_ACTION_APPLIED",
            ],
        )

    def test_plan_failure_does_not_update_mock_environment(self) -> None:
        env = environment("right")
        start_x = env.marker_x

        def fail_plan(*_values):
            raise RuntimeError("MoveIt rejected the plan")

        code, summary = run_closed_loop(
            args(),
            env,
            Path("unused"),
            event_sink=lambda _event: None,
            infer=lambda _frame, _remaining: inference(action("right", 0.03)),
            submit_plan=fail_plan,
            renderer=renderer,
        )
        self.assertEqual(code, 1)
        self.assertEqual(summary["status"], "PLAN_FAILED")
        self.assertEqual(summary["plans_ready"], 0)
        self.assertEqual(summary["actions_applied"], 0)
        self.assertEqual(env.marker_x, start_x)


class SyntheticFrameTest(unittest.TestCase):
    def test_rendered_frame_contains_center_line_and_red_marker(self) -> None:
        env = environment("right")
        with tempfile.TemporaryDirectory() as directory:
            path = render_frame(env, 1, Path(directory))
            with Image.open(path) as image:
                self.assertEqual(image.size, (640, 480))
                self.assertEqual(image.getpixel((320, 20)), (0, 0, 0))
                red = image.getpixel((410, 240))
                self.assertGreater(red[0], 200)
                self.assertLess(red[1], 60)
                self.assertLess(red[2], 70)


if __name__ == "__main__":
    unittest.main()
