#!/usr/bin/env python3
"""
RViz-only bringup - no Gazebo, no controllers.

Use this FIRST. It is how you check that the recovered joint origins are right:
drag each slider through its full range and confirm that every link pivots about
a physically sensible hinge instead of swinging about the base. Any joint that
sweeps the whole arm around the origin means its axis point in
uw_arm.urdf.xacro still needs correcting.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    xacro_file = PathJoinSubstitution(
        [FindPackageShare("uw_arm_description"), "urdf", "uw_arm.urdf.xacro"]
    )

    # use_gazebo:=false strips the <gazebo> blocks so RViz does not choke on
    # sensor and plugin tags it cannot interpret.
    # Paths are quoted because launch's Command substitution shlex-splits the
    # assembled string, and this workspace lives under a directory with a space
    # in its name ("ROBOTIC ARM"). Without the quotes xacro is handed two args.
    robot_description = ParameterValue(
        Command([
            'xacro "', xacro_file, '"',
            " use_gazebo:=false",
            " use_camera:=", LaunchConfiguration("use_camera"),
            " use_gripper:=", LaunchConfiguration("use_gripper"),
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_camera", default_value="true"),
        DeclareLaunchArgument("use_gripper", default_value="true"),
        DeclareLaunchArgument(
            "gui", default_value="false",
            description="true uses joint_state_publisher_gui sliders "
                        "(needs ros-jazzy-joint-state-publisher-gui installed)"),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        # joint_state_publisher_gui is NOT part of a default ros-jazzy-desktop
        # install, so the automatic sweep is the default here. It also exercises
        # every joint through its full range without any manual dragging.
        # For sliders instead:
        #   sudo apt install ros-jazzy-joint-state-publisher-gui
        #   ros2 launch uw_arm_bringup display.launch.py gui:=true
        Node(
            package="uw_arm_bringup",
            executable="joint_sweep.py",
            output="screen",
            condition=UnlessCondition(LaunchConfiguration("gui")),
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="screen",
            condition=IfCondition(LaunchConfiguration("gui")),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=[
                "-d",
                PathJoinSubstitution(
                    [FindPackageShare("uw_arm_description"), "rviz", "uw_arm.rviz"]
                ),
            ],
        ),
    ])
