#!/usr/bin/env python3

import pathlib
import unittest
import xml.etree.ElementTree as ET

import yaml


ROOT = pathlib.Path(__file__).resolve().parents[1]
CONFIG = ROOT / "src" / "franka_fr3_moveit_config" / "config"
PACKAGE_XML = ROOT / "src" / "franka_fr3_moveit_config" / "package.xml"
JOINTS = [f"fr3_joint{index}" for index in range(1, 8)]


class PilzMoveItConfigTest(unittest.TestCase):
    def test_pilz_pipeline_loads_command_planner(self):
        config = yaml.safe_load(
            (
                CONFIG
                / "pilz_industrial_motion_planner_planning.yaml"
            ).read_text(encoding="ascii")
        )
        self.assertEqual(
            config["planning_plugin"],
            "pilz_industrial_motion_planner/CommandPlanner",
        )
        self.assertEqual(config["default_planner_config"], "PTP")

    def test_cartesian_limits_are_conservative_and_complete(self):
        config = yaml.safe_load(
            (CONFIG / "pilz_cartesian_limits.yaml").read_text(
                encoding="ascii"
            )
        )["cartesian_limits"]
        self.assertGreater(config["max_trans_vel"], 0.0)
        self.assertGreater(config["max_trans_acc"], 0.0)
        self.assertLess(config["max_trans_dec"], 0.0)
        self.assertGreater(config["max_rot_vel"], 0.0)

    def test_all_fr3_joints_have_conservative_dynamics_limits(self):
        limits = yaml.safe_load(
            (CONFIG / "joint_limits.yaml").read_text(encoding="ascii")
        )["joint_limits"]
        self.assertEqual(set(limits), set(JOINTS))
        for joint in JOINTS:
            self.assertTrue(limits[joint]["has_velocity_limits"])
            self.assertLessEqual(limits[joint]["max_velocity"], 0.20)
            self.assertTrue(limits[joint]["has_acceleration_limits"])
            self.assertLessEqual(limits[joint]["max_acceleration"], 0.50)

    def test_package_declares_pilz_runtime_dependency(self):
        root = ET.parse(PACKAGE_XML).getroot()
        dependencies = {
            element.text
            for element in root.findall("exec_depend")
        }
        self.assertIn("pilz_industrial_motion_planner", dependencies)


if __name__ == "__main__":
    unittest.main()
