#!/usr/bin/env python3

from pathlib import Path
import sys
from types import SimpleNamespace
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "windows_vlm_bridge"))
sys.path.insert(0, str(ROOT / "scripts"))

from closed_loop_vla_execution_session import (  # noqa: E402
    extract_verified_motion,
    run_hardware_loop,
    validate_args,
    validate_exact_session_limits,
)
from closed_loop_vla_shadow_session import MockVisualEnvironment  # noqa: E402
from isaac_qwen_coordinator import NoAction  # noqa: E402
from static_vla_execution_session import InferenceResult  # noqa: E402
from tests.test_bounded_session_supervisor import SESSION_ID  # noqa: E402


JOINTS = tuple(f"fr3_joint{index}" for index in range(1, 8))


def args(**overrides):
    values = {
        "joint": "fr3_joint3",
        "min_confidence": 0.70,
        "max_delta": 0.03,
        "max_total_delta": 0.03,
        "max_moving_joints": 1,
        "max_actions": 2,
        "max_cumulative_delta": 0.06,
        "max_measured_delta_error": 0.005,
        "max_session_seconds": 60.0,
        "interval": 0.0,
    }
    values.update(overrides)
    return SimpleNamespace(**values)


def configuration_args(**overrides):
    values = {
        **vars(args()),
        "run_armed_session": True,
        "width": 640,
        "height": 480,
        "marker_radius": 12,
        "start_offset_px": 60.0,
        "deadband_px": 15.0,
        "pixels_per_rad": 1000.0,
        "qwen_timeout": 180.0,
        "ubuntu_timeout": 90.0,
        "ubuntu_health_url": "http://10.10.10.1:8080/health",
        "ubuntu_step_url": "http://10.10.10.1:8080/v1/sessions/step",
        "ubuntu_stop_url": "http://10.10.10.1:8080/v1/sessions/stop",
        "session_id": "",
        "task": "",
    }
    values.update(overrides)
    return SimpleNamespace(**values)


def environment(side="right"):
    offset = -60.0 if side == "left" else 60.0
    return MockVisualEnvironment(
        width=640,
        height=480,
        marker_x=320.0 + offset,
        marker_radius=12,
        deadband_px=15.0,
        pixels_per_rad=1000.0,
    )


def session(**limit_overrides):
    limits = {
        "allowed_joints": ["fr3_joint3"],
        "max_moving_joints": 1,
        "max_steps": 2,
        "max_duration_seconds": 60.0,
        "max_delta": 0.03,
        "max_total_delta": 0.03,
        "max_cumulative_delta": 0.06,
    }
    limits.update(limit_overrides)
    return {
        "session_id": SESSION_ID,
        "status": "ARMED",
        "next_sequence": 1,
        "completed_steps": 0,
        "remaining_seconds": 60.0,
        "limits": limits,
    }


def action(side="right", delta=0.03):
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


def receipt(sequence, requested, measured, *, status="ARMED"):
    before = {joint: 0.0 for joint in JOINTS}
    target = dict(before)
    target["fr3_joint3"] = requested
    after = dict(before)
    after["fr3_joint3"] = measured
    target_error = abs(measured - requested)
    return {
        "status": "VERIFIED_SUCCESS",
        "request_id": f"request-{sequence}",
        "action_sha256": "b" * 64,
        "session_id": SESSION_ID,
        "sequence": sequence,
        "current_joint_positions": before,
        "target_joint_positions": target,
        "planning": {"result": "SUCCESS", "point_count": 6},
        "execution": {
            "status": "VERIFIED_SUCCESS",
            "code": "VERIFIED_SUCCESS",
            "verification": {
                "status": "PASSED",
                "commanded_joints": [
                    {
                        "joint": "fr3_joint3",
                        "requested_delta": requested,
                        "measured_delta": measured,
                        "target_error": target_error,
                        "direction_ok": requested * measured > 0.0,
                        "movement_ok": abs(measured) >= 0.001,
                        "target_ok": target_error <= 0.005,
                    }
                ],
                "max_non_commanded_drift": 0.0,
                "max_non_commanded_drift_joint": "fr3_joint1",
                "non_commanded_drift_ok": True,
                "max_settled_velocity": 0.0,
                "max_settled_velocity_joint": "fr3_joint1",
                "settled_velocity_ok": True,
                "measured_joint_positions": after,
                "settled_joint_velocities": {joint: 0.0 for joint in JOINTS},
            },
        },
        "session": {
            **session(),
            "status": status,
            "next_sequence": sequence + 1,
            "completed_steps": sequence,
            "cumulative_absolute_delta": sequence * abs(requested),
        },
    }


def renderer(_environment, observation, _output_dir):
    return Path(f"frame_{observation:03d}.png")


class MeasuredReceiptTest(unittest.TestCase):
    def test_configuration_requires_explicit_execution_and_exact_budget(self) -> None:
        validate_args(configuration_args())
        with self.assertRaisesRegex(RuntimeError, "run-armed-session"):
            validate_args(configuration_args(run_armed_session=False))
        with self.assertRaisesRegex(RuntimeError, "must equal max-delta"):
            validate_args(configuration_args(max_cumulative_delta=0.05))

    def test_extracts_consistent_measured_delta(self) -> None:
        motion = extract_verified_motion(
            receipt(1, 0.03, 0.029),
            joint="fr3_joint3",
            requested_delta=0.03,
            maximum_error=0.005,
        )
        self.assertAlmostEqual(motion.measured_delta, 0.029)
        self.assertAlmostEqual(motion.target_error, 0.001)

    def test_rejects_missing_measured_verification(self) -> None:
        candidate = receipt(1, 0.03, 0.029)
        del candidate["execution"]["verification"]
        with self.assertRaisesRegex(RuntimeError, "passed measured verification"):
            extract_verified_motion(
                candidate,
                joint="fr3_joint3",
                requested_delta=0.03,
                maximum_error=0.005,
            )

    def test_rejects_inconsistent_positions(self) -> None:
        candidate = receipt(1, 0.03, 0.029)
        candidate["execution"]["verification"]["measured_joint_positions"][
            "fr3_joint3"
        ] = 0.02
        with self.assertRaisesRegex(RuntimeError, "conflicts"):
            extract_verified_motion(
                candidate,
                joint="fr3_joint3",
                requested_delta=0.03,
                maximum_error=0.005,
            )

    def test_armed_limits_must_match_exactly(self) -> None:
        validate_exact_session_limits(session(), args())
        with self.assertRaisesRegex(RuntimeError, "max_steps"):
            validate_exact_session_limits(session(max_steps=3), args())


class HardwareLoopStateMachineTest(unittest.TestCase):
    def run_converging(self, side):
        env = environment(side)
        events = []
        stops = []
        submitted = []

        def infer(_frame, _remaining):
            if env.side == "centered":
                raise NoAction("centered")
            delta = 0.03 if env.side == "right" else -0.03
            return inference(action(env.side, delta))

        def submit(candidate, _digest, _frame, sequence, _remaining):
            submitted.append(sequence)
            requested = candidate["deltas"]["fr3_joint3"]
            measured = 0.029 if requested > 0.0 else -0.029
            status = "COMPLETED" if sequence == 2 else "ARMED"
            return receipt(sequence, requested, measured, status=status)

        code, summary = run_hardware_loop(
            args(),
            env,
            Path("unused"),
            session(),
            event_sink=events.append,
            infer=infer,
            submit_step=submit,
            stop_session=lambda _timeout: stops.append(True),
            renderer=renderer,
        )
        return code, summary, events, submitted, stops

    def test_right_loop_uses_two_measured_steps_then_no_action(self) -> None:
        code, summary, events, submitted, stops = self.run_converging("right")
        self.assertEqual(code, 0)
        self.assertEqual(summary["status"], "GOAL_REACHED")
        self.assertEqual(summary["verified_steps"], 2)
        self.assertEqual(submitted, [1, 2])
        self.assertEqual(stops, [])
        self.assertAlmostEqual(summary["final_error_px"], 2.0)
        self.assertAlmostEqual(summary["requested_cumulative_delta"], 0.06)
        self.assertAlmostEqual(summary["measured_cumulative_delta"], 0.058)
        ordered = [
            event["event"]
            for event in events
            if event["event"] in {"STEP_VERIFIED", "MEASURED_MOCK_UPDATE"}
        ]
        self.assertEqual(
            ordered,
            [
                "STEP_VERIFIED",
                "MEASURED_MOCK_UPDATE",
                "STEP_VERIFIED",
                "MEASURED_MOCK_UPDATE",
            ],
        )

    def test_left_loop_converges_from_negative_measured_motion(self) -> None:
        code, summary, _events, submitted, _stops = self.run_converging("left")
        self.assertEqual(code, 0)
        self.assertEqual(summary["status"], "GOAL_REACHED")
        self.assertEqual(submitted, [1, 2])
        self.assertAlmostEqual(summary["final_error_px"], -2.0)

    def test_failed_receipt_does_not_update_marker_and_stops(self) -> None:
        env = environment("right")
        start_x = env.marker_x
        stops = []
        broken = receipt(1, 0.03, 0.029)
        del broken["execution"]["verification"]
        code, summary = run_hardware_loop(
            args(),
            env,
            Path("unused"),
            session(),
            event_sink=lambda _event: None,
            infer=lambda _frame, _remaining: inference(action()),
            submit_step=lambda *_values: broken,
            stop_session=lambda _timeout: stops.append(True)
            or {"session": {"status": "STOPPED"}},
            renderer=renderer,
        )
        self.assertEqual(code, 1)
        self.assertEqual(summary["status"], "STEP_FAILED")
        self.assertEqual(summary["verified_steps"], 0)
        self.assertEqual(env.marker_x, start_x)
        self.assertEqual(stops, [True])

    def test_ambiguous_step_failure_is_not_retried(self) -> None:
        env = environment("right")
        submits = []
        stops = []

        def fail(*_values):
            submits.append(True)
            raise RuntimeError("ambiguous timeout")

        code, summary = run_hardware_loop(
            args(),
            env,
            Path("unused"),
            session(),
            event_sink=lambda _event: None,
            infer=lambda _frame, _remaining: inference(action()),
            submit_step=fail,
            stop_session=lambda _timeout: stops.append(True)
            or {"session": {"status": "STOPPED"}},
            renderer=renderer,
        )
        self.assertEqual(code, 1)
        self.assertEqual(summary["status"], "STEP_FAILED")
        self.assertEqual(submits, [True])
        self.assertEqual(stops, [True])
        self.assertEqual(summary["verified_steps"], 0)

    def test_stop_failure_overrides_goal_success(self) -> None:
        env = environment("right")
        env.marker_x = env.center_x
        code, summary = run_hardware_loop(
            args(),
            env,
            Path("unused"),
            session(),
            event_sink=lambda _event: None,
            infer=lambda _frame, _remaining: (_ for _ in ()).throw(
                NoAction("centered")
            ),
            submit_step=lambda *_values: None,
            stop_session=lambda _timeout: (_ for _ in ()).throw(
                RuntimeError("stop unavailable")
            ),
            renderer=renderer,
        )
        self.assertEqual(code, 1)
        self.assertEqual(summary["status"], "FAILED_STOP_UNCONFIRMED")
        self.assertFalse(summary["stop_confirmed"])


if __name__ == "__main__":
    unittest.main()
