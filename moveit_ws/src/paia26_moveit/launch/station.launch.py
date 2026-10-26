"""Supply CRANE+ V2 robot description/kinematics to a standalone MoveIt node.

This deliberately mirrors the vendor's example.launch.py parameters rather
than depending on undocumented CLI/global parameter discovery behavior.
"""

import os

from ament_index_python.packages import get_package_share_directory
from crane_plus_description.robot_description_loader import RobotDescriptionLoader
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    description_loader = RobotDescriptionLoader()
    description_loader.use_camera = "false"
    robot_description = description_loader.load()

    share = get_package_share_directory("crane_plus_moveit_config")
    with open(os.path.join(share, "config", "crane_plus.srdf"), encoding="utf-8") as f:
        srdf = f.read()
    with open(os.path.join(share, "config", "kinematics.yaml"), encoding="utf-8") as f:
        kinematics = yaml.safe_load(f)

    return LaunchDescription([
        DeclareLaunchArgument("station", default_value="CENTER"),
        Node(
            package="paia26_moveit",
            executable="station_move",
            name="paia26_station_move",
            output="screen",
            arguments=[LaunchConfiguration("station")],
            parameters=[
                {"robot_description": robot_description},
                {"robot_description_semantic": srdf},
                kinematics,
                {"use_sim_time": True},
            ],
        ),
    ])
