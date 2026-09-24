#!/usr/bin/env python3

import copy
from pathlib import Path
import unittest

import yaml


WORKSPACE = Path(__file__).resolve().parents[1]
CONFIG_DIR = WORKSPACE / "src" / "franka_fr3_moveit_config" / "config"


def controller_parameters(path: Path):
    with path.open(encoding="utf-8") as stream:
        documents = list(yaml.safe_load_all(stream))
    return documents[0]["/**"]["fr3_arm_controller"]["ros__parameters"]


class Joint7TuningProfileTests(unittest.TestCase):
    def setUp(self):
        self.default = controller_parameters(
            CONFIG_DIR / "fr3_ros_controllers.yaml"
        )
        self.tuned = controller_parameters(
            CONFIG_DIR / "fr3_ros_controllers_joint7_p75.yaml"
        )
        self.tuned_p100 = controller_parameters(
            CONFIG_DIR / "fr3_ros_controllers_joint7_p100.yaml"
        )
        self.tuned_p125 = controller_parameters(
            CONFIG_DIR / "fr3_ros_controllers_joint7_p125.yaml"
        )

    def test_profile_changes_only_joint7_proportional_gain(self):
        expected = copy.deepcopy(self.default)
        expected["gains"]["fr3_joint7"]["p"] = 75.0
        self.assertEqual(self.tuned, expected)

    def test_effort_interface_and_non_integrating_joint7_gain_are_preserved(self):
        self.assertEqual(self.tuned["command_interfaces"], ["effort"])
        self.assertEqual(self.tuned["gains"]["fr3_joint7"]["i"], 0.0)
        self.assertEqual(self.tuned["gains"]["fr3_joint7"]["d"], 5.0)
        self.assertEqual(self.tuned["constraints"], self.default["constraints"])

    def test_p100_profile_changes_only_joint7_proportional_gain(self):
        expected = copy.deepcopy(self.default)
        expected["gains"]["fr3_joint7"]["p"] = 100.0
        self.assertEqual(self.tuned_p100, expected)

    def test_p125_profile_changes_only_joint7_proportional_gain(self):
        expected = copy.deepcopy(self.default)
        expected["gains"]["fr3_joint7"]["p"] = 125.0
        self.assertEqual(self.tuned_p125, expected)


if __name__ == "__main__":
    unittest.main()
