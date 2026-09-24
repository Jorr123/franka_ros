#!/usr/bin/env python3

import pathlib
from types import SimpleNamespace
import sys
import unittest
from unittest.mock import patch

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose, Quaternion


SCRIPTS_DIR = pathlib.Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from moveit_cartesian_ik_joint_test import (  # noqa: E402
    IKCandidate,
    IKSafetyReport,
    JOINTS,
    add_wrist_path_constraint,
    extract_ik_solution,
    make_cartesian_request,
    make_ik_request,
    make_ik_seed_candidates,
    make_pilz_lin_plan_request,
    make_target_pose,
    parse_args,
    quaternion_angular_distance,
    require_nonempty_trajectory,
    require_pilz_execution_checks,
    sample_trajectory_joint_positions,
    select_closest_ik_candidate,
    trajectory_joint_excursion,
    validate_collision_free_samples,
    validate_ik_target,
    validate_pilz_joint_excursions,
    validate_planned_cartesian_path,
    validate_preexecution_state,
)


def transform(x=0.4, y=0.0, z=0.3):
    return SimpleNamespace(
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=x, y=y, z=z),
            rotation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        )
    )


def ik_response(values):
    return SimpleNamespace(
        solution=SimpleNamespace(
            joint_state=SimpleNamespace(
                name=list(JOINTS),
                position=list(values),
            )
        )
    )


def trajectory(joint7_values):
    points = [
        SimpleNamespace(positions=[0.0] * 6 + [value])
        for value in joint7_values
    ]
    return SimpleNamespace(
        joint_trajectory=SimpleNamespace(
            joint_names=list(JOINTS),
            points=points,
        )
    )


def pose(x=0.4, y=0.0, z=0.3):
    value = Pose()
    value.position.x = x
    value.position.y = y
    value.position.z = z
    value.orientation.w = 1.0
    return value


class FakeClock:
    def now(self):
        return SimpleNamespace(to_msg=lambda: Time(sec=1, nanosec=2))


class FakeNode:
    def get_clock(self):
        return FakeClock()


class FakeStateValidityNode(FakeNode):
    def __init__(self, valid_results):
        self.valid_results = iter(valid_results)

    def check_state_validity(self, request, timeout):
        del request, timeout
        return SimpleNamespace(
            valid=next(self.valid_results),
            contacts=[],
        )


class CartesianIKJointPureTests(unittest.TestCase):
    def setUp(self):
        self.current = {joint: 0.0 for joint in JOINTS}

    def test_target_pose_adds_base_frame_translation(self):
        pose = make_target_pose(transform(), 0.01, -0.02, 0.03)
        self.assertAlmostEqual(pose.position.x, 0.41)
        self.assertAlmostEqual(pose.position.y, -0.02)
        self.assertAlmostEqual(pose.position.z, 0.33)
        self.assertAlmostEqual(pose.orientation.w, 1.0)

    def test_default_ik_motion_limits_are_moderately_relaxed(self):
        with patch.object(
            sys,
            "argv",
            ["moveit_cartesian_ik_joint_test.py", "--dy", "0.005"],
        ):
            args = parse_args()

        self.assertAlmostEqual(args.max_delta, 0.12)
        self.assertAlmostEqual(args.max_total_joint_delta, 0.30)
        self.assertAlmostEqual(args.max_joint_excursion, 0.12)
        self.assertAlmostEqual(args.max_wrist_joint_delta, 0.002)
        self.assertEqual(args.ik_attempts, 5)
        self.assertEqual(args.planner, "RRTConnectkConfigDefault")
        self.assertEqual(args.path_mode, "cartesian")
        self.assertAlmostEqual(args.eef_step, 0.00025)
        self.assertAlmostEqual(args.jump_threshold, 0.0)
        self.assertAlmostEqual(args.revolute_jump_threshold, 0.10)
        self.assertAlmostEqual(args.max_planned_cross_track_error, 0.001)
        self.assertAlmostEqual(args.planned_cartesian_goal_tolerance, 0.005)
        self.assertAlmostEqual(args.max_planned_orientation_error, 0.03)
        self.assertAlmostEqual(args.pilz_goal_orientation_tolerance, 0.001)
        self.assertAlmostEqual(args.fk_joint_sample_step, 0.002)
        self.assertEqual(args.max_fk_samples, 500)
        self.assertAlmostEqual(args.max_planned_velocity, 0.02)
        self.assertAlmostEqual(args.max_planned_acceleration, 0.05)
        self.assertAlmostEqual(args.cartesian_verification_tolerance, 0.003)

    def test_cartesian_request_constrains_every_waypoint_locally(self):
        current = {joint: 0.01 * index for index, joint in enumerate(JOINTS)}
        velocities = {joint: 0.0 for joint in JOINTS}
        args = SimpleNamespace(
            planning_frame="base",
            group="fr3_arm",
            ee_link="fr3_hand_tcp",
            eef_step=0.00025,
            jump_threshold=0.0,
            revolute_jump_threshold=0.10,
            max_delta=0.12,
            max_wrist_joint_delta=0.002,
        )
        target = make_target_pose(transform(), 0.0, 0.005, 0.0)
        request = make_cartesian_request(
            FakeNode(),
            target,
            current,
            velocities,
            args,
        )

        self.assertEqual(request.header.frame_id, "base")
        self.assertEqual(request.group_name, "fr3_arm")
        self.assertEqual(request.link_name, "fr3_hand_tcp")
        self.assertEqual(len(request.waypoints), 1)
        self.assertAlmostEqual(request.max_step, 0.00025)
        self.assertAlmostEqual(request.jump_threshold, 0.0)
        self.assertAlmostEqual(request.revolute_jump_threshold, 0.10)
        self.assertTrue(request.avoid_collisions)
        constraints = request.path_constraints.joint_constraints
        self.assertEqual(
            [constraint.joint_name for constraint in constraints],
            JOINTS,
        )
        self.assertAlmostEqual(constraints[0].tolerance_above, 0.12)
        self.assertAlmostEqual(constraints[-1].tolerance_above, 0.002)

    def test_fk_sampling_densifies_between_trajectory_points(self):
        planned = trajectory((0.0, 0.005))
        samples = sample_trajectory_joint_positions(
            planned,
            max_joint_step=0.002,
            max_samples=10,
        )
        self.assertEqual(len(samples), 4)
        self.assertAlmostEqual(samples[0]["fr3_joint7"], 0.0)
        self.assertAlmostEqual(samples[1]["fr3_joint7"], 0.005 / 3.0)
        self.assertAlmostEqual(samples[-1]["fr3_joint7"], 0.005)

    def test_fk_sampling_rejects_excess_sample_count(self):
        planned = trajectory((0.0, 0.01))
        with self.assertRaisesRegex(RuntimeError, "more than 3 samples"):
            sample_trajectory_joint_positions(
                planned,
                max_joint_step=0.002,
                max_samples=3,
            )

    def test_pilz_lin_request_uses_cartesian_goal_and_pipeline(self):
        with patch.object(
            sys,
            "argv",
            [
                "moveit_cartesian_ik_joint_test.py",
                "--path-mode",
                "pilz-lin",
                "--dy",
                "-0.005",
            ],
        ):
            args = parse_args()
        current = {joint: 0.01 * index for index, joint in enumerate(JOINTS)}
        target = make_target_pose(transform(), 0.0, -0.005, 0.0)

        request = make_pilz_lin_plan_request(
            FakeNode(),
            target,
            current,
            args,
        )
        motion = request.motion_plan_request
        self.assertEqual(motion.pipeline_id, "pilz_industrial_motion_planner")
        self.assertEqual(motion.planner_id, "LIN")
        self.assertEqual(motion.num_planning_attempts, 1)
        self.assertEqual(motion.group_name, "fr3_arm")
        self.assertEqual(
            list(motion.start_state.joint_state.position),
            [current[joint] for joint in JOINTS],
        )
        self.assertEqual(
            list(motion.start_state.joint_state.velocity),
            [0.0] * len(JOINTS),
        )
        self.assertEqual(len(motion.goal_constraints), 1)
        goal = motion.goal_constraints[0]
        self.assertEqual(len(goal.joint_constraints), 0)
        self.assertEqual(len(goal.position_constraints), 1)
        self.assertEqual(len(goal.orientation_constraints), 1)
        self.assertEqual(
            goal.position_constraints[0].link_name,
            "fr3_hand_tcp",
        )
        goal_pose = (
            goal.position_constraints[0]
            .constraint_region.primitive_poses[0]
        )
        self.assertAlmostEqual(goal_pose.position.y, -0.005)
        self.assertAlmostEqual(
            goal.orientation_constraints[0].absolute_x_axis_tolerance,
            0.001,
        )
        self.assertEqual(
            goal.orientation_constraints[0].orientation,
            target.orientation,
        )

    def test_pilz_empty_trajectory_is_rejected(self):
        empty = trajectory(())
        with self.assertRaisesRegex(RuntimeError, "empty or unusable"):
            require_nonempty_trajectory(empty)

    def test_pilz_joint7_excursion_is_rejected(self):
        planned = trajectory((0.0, 0.003))
        with self.assertRaisesRegex(RuntimeError, "fr3_joint7 excursion"):
            validate_pilz_joint_excursions(
                planned,
                self.current,
                max_wrist_joint_delta=0.002,
                max_total_joint_delta=0.30,
            )

    def test_pilz_total_joint_excursion_is_rejected(self):
        planned = trajectory((0.0, 0.0))
        planned.joint_trajectory.points[-1].positions = [0.04] * 6 + [0.0]
        with self.assertRaisesRegex(RuntimeError, "total joint excursion"):
            validate_pilz_joint_excursions(
                planned,
                self.current,
                max_wrist_joint_delta=0.002,
                max_total_joint_delta=0.20,
            )

    def test_pilz_execution_gate_rejects_any_missing_validation(self):
        incomplete = set(
            (
                "nonempty_trajectory",
                "joint_trajectory",
                "joint_excursions",
                "collision_free",
            )
        )
        with self.assertRaisesRegex(
            RuntimeError,
            "cartesian_path",
        ):
            require_pilz_execution_checks(incomplete)

    def test_pilz_collision_sample_rejection_blocks_validation(self):
        args = SimpleNamespace(group="fr3_arm", fk_timeout=1.0)
        node = FakeStateValidityNode((True, False))
        with self.assertRaisesRegex(RuntimeError, "Collision detected"):
            validate_collision_free_samples(
                node,
                (dict(self.current), dict(self.current)),
                args,
            )

    def test_planned_cartesian_path_accepts_straight_waypoints(self):
        poses = [
            pose(y=0.001 * index)
            for index in range(6)
        ]
        report = validate_planned_cartesian_path(
            start_position=(0.4, 0.0, 0.3),
            target_position=(0.4, 0.005, 0.3),
            target_orientation=(0.0, 0.0, 0.0, 1.0),
            poses=poses,
            max_cross_track_error=0.001,
            max_progress_regression=0.0005,
            max_segment_overrun=0.0005,
            final_target_error_limit=0.001,
            max_orientation_error=0.01,
        )

        self.assertEqual(report.point_count, 6)
        self.assertAlmostEqual(report.max_cross_track_error, 0.0)
        self.assertAlmostEqual(report.minimum_z_offset, 0.0)
        self.assertAlmostEqual(report.final_target_error, 0.0)
        self.assertAlmostEqual(report.final_orientation_error, 0.0)

    def test_planned_cartesian_path_rejects_downward_dip(self):
        poses = [
            pose(y=0.0),
            pose(y=0.001),
            pose(y=0.002, z=0.2975),
            pose(y=0.003),
            pose(y=0.004),
            pose(y=0.005),
        ]
        with self.assertRaisesRegex(RuntimeError, "cross-track"):
            validate_planned_cartesian_path(
                start_position=(0.4, 0.0, 0.3),
                target_position=(0.4, 0.005, 0.3),
                target_orientation=(0.0, 0.0, 0.0, 1.0),
                poses=poses,
                max_cross_track_error=0.001,
                max_progress_regression=0.0005,
                max_segment_overrun=0.0005,
                final_target_error_limit=0.001,
                max_orientation_error=0.01,
            )

    def test_planned_cartesian_path_rejects_backward_step(self):
        poses = [
            pose(y=0.0),
            pose(y=0.002),
            pose(y=0.001),
            pose(y=0.003),
            pose(y=0.004),
            pose(y=0.005),
        ]
        with self.assertRaisesRegex(RuntimeError, "backward"):
            validate_planned_cartesian_path(
                start_position=(0.4, 0.0, 0.3),
                target_position=(0.4, 0.005, 0.3),
                target_orientation=(0.0, 0.0, 0.0, 1.0),
                poses=poses,
                max_cross_track_error=0.001,
                max_progress_regression=0.0005,
                max_segment_overrun=0.0005,
                final_target_error_limit=0.001,
                max_orientation_error=0.01,
            )

    def test_planned_cartesian_path_rejects_endpoint_error(self):
        poses = [
            pose(y=0.0),
            pose(y=0.001),
            pose(y=0.002),
            pose(y=0.003),
        ]
        with self.assertRaisesRegex(RuntimeError, "endpoint error"):
            validate_planned_cartesian_path(
                start_position=(0.4, 0.0, 0.3),
                target_position=(0.4, 0.005, 0.3),
                target_orientation=(0.0, 0.0, 0.0, 1.0),
                poses=poses,
                max_cross_track_error=0.003,
                max_progress_regression=0.0005,
                max_segment_overrun=0.0005,
                final_target_error_limit=0.001,
                max_orientation_error=0.03,
            )

    def test_planned_cartesian_path_rejects_orientation_error(self):
        poses = [pose(y=0.0), pose(y=0.005)]
        poses[-1].orientation.z = 0.5
        poses[-1].orientation.w = 3.0 ** 0.5 / 2.0
        with self.assertRaisesRegex(RuntimeError, "orientation changes"):
            validate_planned_cartesian_path(
                start_position=(0.4, 0.0, 0.3),
                target_position=(0.4, 0.005, 0.3),
                target_orientation=(0.0, 0.0, 0.0, 1.0),
                poses=poses,
                max_cross_track_error=0.001,
                max_progress_regression=0.0005,
                max_segment_overrun=0.0005,
                final_target_error_limit=0.005,
                max_orientation_error=0.03,
            )

    def test_quaternion_distance_treats_sign_as_equivalent(self):
        self.assertAlmostEqual(
            quaternion_angular_distance(
                (0.0, 0.0, 0.0, 1.0),
                (0.0, 0.0, 0.0, -1.0),
            ),
            0.0,
        )

    def test_extracts_named_ik_solution(self):
        values = [0.01 * index for index in range(7)]
        solution = extract_ik_solution(ik_response(values))
        self.assertEqual(list(solution), JOINTS)
        self.assertAlmostEqual(solution["fr3_joint7"], 0.06)

    def test_ik_request_constrains_every_joint_near_measured_state(self):
        current = {joint: 0.01 * index for index, joint in enumerate(JOINTS)}
        seed = dict(current)
        seed["fr3_joint1"] += 0.01
        velocities = {joint: 0.0 for joint in JOINTS}
        args = SimpleNamespace(
            group="fr3_arm",
            ee_link="fr3_hand_tcp",
            planning_frame="base",
            ik_timeout=1.0,
            ik_joint7_tolerance=0.001,
            max_delta=0.12,
        )
        request = make_ik_request(
            FakeNode(),
            make_target_pose(transform(), 0.0, 0.005, 0.0),
            seed,
            current,
            velocities,
            args,
        )

        self.assertAlmostEqual(
            request.ik_request.robot_state.joint_state.position[0],
            seed["fr3_joint1"],
        )
        constraints = request.ik_request.constraints.joint_constraints
        self.assertEqual(len(constraints), 7)
        self.assertEqual(
            [constraint.joint_name for constraint in constraints],
            JOINTS,
        )
        self.assertAlmostEqual(
            constraints[0].position,
            current["fr3_joint1"],
        )
        self.assertAlmostEqual(constraints[0].tolerance_above, 0.12)
        self.assertAlmostEqual(
            constraints[-1].position,
            current["fr3_joint7"],
        )
        self.assertAlmostEqual(constraints[-1].tolerance_above, 0.001)
        self.assertAlmostEqual(constraints[-1].tolerance_below, 0.001)

    def test_local_ik_seed_candidates_are_deterministic_and_hold_joint7(self):
        current = {
            joint: 0.1 * index for index, joint in enumerate(JOINTS)
        }
        seeds = make_ik_seed_candidates(
            current,
            attempts=3,
            perturbation=0.01,
            max_offset=0.06,
        )

        self.assertEqual(seeds[0], current)
        self.assertAlmostEqual(
            seeds[1]["fr3_joint1"],
            current["fr3_joint1"] + 0.01,
        )
        self.assertAlmostEqual(
            seeds[2]["fr3_joint1"],
            current["fr3_joint1"] - 0.01,
        )
        for seed in seeds:
            self.assertEqual(seed["fr3_joint7"], current["fr3_joint7"])

    def test_closest_local_ik_candidate_wins(self):
        farther = IKCandidate(
            attempt=1,
            target=dict(self.current),
            safety=IKSafetyReport(
                total_delta=0.08,
                max_delta=0.03,
                max_delta_joint="fr3_joint3",
                wrist_delta=0.0001,
            ),
        )
        closer = IKCandidate(
            attempt=4,
            target=dict(self.current),
            safety=IKSafetyReport(
                total_delta=0.04,
                max_delta=0.02,
                max_delta_joint="fr3_joint6",
                wrist_delta=0.0002,
            ),
        )

        selected = select_closest_ik_candidate((farther, closer))
        self.assertEqual(selected.attempt, 4)

    def test_plan_request_constrains_wrist_along_entire_path(self):
        plan_request = SimpleNamespace(
            motion_plan_request=SimpleNamespace(
                path_constraints=SimpleNamespace(joint_constraints=[])
            )
        )
        current = dict(self.current)
        current["fr3_joint7"] = 0.6
        add_wrist_path_constraint(plan_request, current, 0.002)

        constraints = (
            plan_request.motion_plan_request.path_constraints.joint_constraints
        )
        self.assertEqual(len(constraints), 1)
        self.assertEqual(constraints[0].joint_name, "fr3_joint7")
        self.assertAlmostEqual(constraints[0].position, 0.6)
        self.assertAlmostEqual(constraints[0].tolerance_above, 0.002)
        self.assertAlmostEqual(constraints[0].tolerance_below, 0.002)

    def test_rejects_ik_solution_missing_joint(self):
        response = ik_response([0.0] * 7)
        response.solution.joint_state.name.pop()
        response.solution.joint_state.position.pop()
        with self.assertRaisesRegex(RuntimeError, "missing"):
            extract_ik_solution(response)

    def test_accepts_bounded_ik_target(self):
        target = dict(self.current)
        target["fr3_joint3"] = 0.03
        target["fr3_joint7"] = 0.005
        report = validate_ik_target(
            self.current,
            target,
            max_joint_delta=0.08,
            max_total_joint_delta=0.20,
            max_wrist_joint_delta=0.01,
        )
        self.assertAlmostEqual(report.total_delta, 0.035)
        self.assertEqual(report.max_delta_joint, "fr3_joint3")

    def test_rejects_wrist_target_that_matches_failed_run(self):
        target = dict(self.current)
        target["fr3_joint7"] = 0.017847
        with self.assertRaisesRegex(RuntimeError, "fr3_joint7"):
            validate_ik_target(
                self.current,
                target,
                max_joint_delta=0.08,
                max_total_joint_delta=0.20,
                max_wrist_joint_delta=0.01,
            )

    def test_rejects_large_total_joint_change(self):
        target = {joint: 0.04 for joint in JOINTS}
        with self.assertRaisesRegex(RuntimeError, "total absolute"):
            validate_ik_target(
                self.current,
                target,
                max_joint_delta=0.08,
                max_total_joint_delta=0.20,
                max_wrist_joint_delta=0.05,
            )

    def test_measures_joint7_path_excursion(self):
        planned = trajectory((0.0, 0.008, -0.012))
        excursion = trajectory_joint_excursion(
            planned,
            self.current,
            "fr3_joint7",
        )
        self.assertAlmostEqual(excursion, 0.012)

    def test_preexecution_state_accepts_fresh_state(self):
        measured = dict(self.current)
        measured["fr3_joint2"] = 0.002
        joint, error = validate_preexecution_state(
            self.current,
            measured,
            0.003,
        )
        self.assertEqual(joint, "fr3_joint2")
        self.assertAlmostEqual(error, 0.002)

    def test_preexecution_state_rejects_stale_plan(self):
        measured = dict(self.current)
        measured["fr3_joint4"] = -0.004
        with self.assertRaisesRegex(RuntimeError, "changed after IK"):
            validate_preexecution_state(
                self.current,
                measured,
                0.003,
            )


if __name__ == "__main__":
    unittest.main()
