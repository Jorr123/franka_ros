# Copyright (c) 2026
# SPDX-License-Identifier: Apache-2.0

"""Launch MoveIt Servo beside an already-running FR3 MoveIt stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path, "r", encoding="ascii") as config_file:
        return yaml.safe_load(config_file)


def generate_launch_description():
    robot_ip = LaunchConfiguration("robot_ip")
    namespace = LaunchConfiguration("namespace")
    load_gripper = LaunchConfiguration("load_gripper")
    ee_id = LaunchConfiguration("ee_id")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")

    description_package = get_package_share_directory("franka_description")
    robot_xacro = os.path.join(
        description_package, "robots", "fr3", "fr3.urdf.xacro"
    )
    semantic_xacro = os.path.join(
        description_package, "robots", "fr3", "fr3.srdf.xacro"
    )

    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    FindExecutable(name="xacro"),
                    " ",
                    robot_xacro,
                    " hand:=",
                    load_gripper,
                    " robot_ip:=",
                    robot_ip,
                    " ee_id:=",
                    ee_id,
                    " use_fake_hardware:=",
                    use_fake_hardware,
                    " fake_sensor_commands:=",
                    fake_sensor_commands,
                    " ros2_control:=true",
                ]
            ),
            value_type=str,
        )
    }
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            Command(
                [
                    FindExecutable(name="xacro"),
                    " ",
                    semantic_xacro,
                    " hand:=",
                    load_gripper,
                    " ee_id:=",
                    ee_id,
                ]
            ),
            value_type=str,
        )
    }
    kinematics = load_yaml(
        "franka_fr3_moveit_config", "config/kinematics.yaml"
    )
    servo_parameters = {
        "moveit_servo": load_yaml(
            "franka_fr3_moveit_config", "config/fr3_servo.yaml"
        )
    }

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        namespace=namespace,
        output="screen",
        parameters=[
            servo_parameters,
            robot_description,
            robot_description_semantic,
            kinematics,
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_ip", description="Hostname or IP address of the FR3."
            ),
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("load_gripper", default_value="true"),
            DeclareLaunchArgument("ee_id", default_value="franka_hand"),
            DeclareLaunchArgument("use_fake_hardware", default_value="false"),
            DeclareLaunchArgument("fake_sensor_commands", default_value="false"),
            servo_node,
        ]
    )
