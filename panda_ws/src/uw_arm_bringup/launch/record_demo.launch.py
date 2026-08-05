#!/usr/bin/env python3
"""
Records the Phase 1 demonstration video.

Runs the full underwater simulation headless (no Gazebo GUI, no RViz), drives
the demo sequence, and writes an MP4 from the in-simulation cameras.

    ros2 launch uw_arm_bringup record_demo.launch.py \
        output:=/home/ak/uw_arm_demo.mp4 duration:=75

Headless is deliberate: the video comes from camera sensors, so a GUI would only
add load and risk the recording picking up window chrome.
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    Shutdown,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


def generate_launch_description():
    bringup_share = get_package_share_directory("uw_arm_bringup")

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "underwater_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": "-r -s -v 2",   # -s = server only, no GUI
            "rviz": "false",
            "use_camera": LaunchConfiguration("wrist_cam"),
        }.items(),
    )

    recorder = Node(
        package="uw_arm_bringup",
        executable="record_demo.py",
        output="screen",
        parameters=[{
            "output": LaunchConfiguration("output"),
            # Explicit float: a command-line "duration:=70" arrives as INTEGER
            # and rclpy rejects it against the node's DOUBLE declaration.
            "duration": ParameterValue(LaunchConfiguration("duration"),
                                       value_type=float),
            "fps": 30,
            # No use_sim_time: the recorder is driven by camera callbacks, and
            # a stalled /clock subscription would otherwise freeze it silently.
        }],
    )

    demo = Node(
        package="uw_arm_bringup",
        executable="demo_sequence.py",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "output",
            default_value=os.path.expanduser("~/uw_arm_demo.mp4")),
        DeclareLaunchArgument("duration", default_value="75.0"),
        DeclareLaunchArgument("wrist_cam", default_value="true"),

        sim,
        # The recorder must be capturing before the arm starts moving, and the
        # controllers need time to spawn. 18 s covers Gazebo start plus the
        # chained spawners; demo_sequence then blocks on its action servers.
        TimerAction(period=14.0, actions=[recorder]),
        TimerAction(period=18.0, actions=[demo]),

        # Tear the whole launch down as soon as the file is closed.
        RegisterEventHandler(
            OnProcessExit(target_action=recorder,
                          on_exit=[Shutdown(reason="recording finished")])
        ),
    ])
