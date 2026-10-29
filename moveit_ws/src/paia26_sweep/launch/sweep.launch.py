"""PAIA26 v0.9: MoveIt multi-phase sweep; default mode is planning only."""
import os
from ament_index_python.packages import get_package_share_directory
from crane_plus_description.robot_description_loader import RobotDescriptionLoader
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    loader = RobotDescriptionLoader()
    loader.use_camera = "false"
    srdf_path = os.path.join(get_package_share_directory("crane_plus_moveit_config"),
                             "config", "crane_plus.srdf")
    kin_path = os.path.join(get_package_share_directory("crane_plus_moveit_config"),
                            "config", "kinematics.yaml")
    with open(srdf_path, encoding="utf-8") as f:
        srdf = f.read()
    with open(kin_path, encoding="utf-8") as f:
        kinematics = yaml.safe_load(f)
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="plan",
                              description="plan (no motion) or execute (simulated arm moves)"),
        Node(package="paia26_sweep", executable="sweep_move",
             name="paia26_sweep_move", output="screen",
             arguments=[LaunchConfiguration("mode")],
             parameters=[{"robot_description": loader.load()},
                         {"robot_description_semantic": srdf},
                         kinematics, {"use_sim_time": True}]),
    ])
