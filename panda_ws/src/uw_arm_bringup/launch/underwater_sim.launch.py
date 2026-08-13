#!/usr/bin/env python3
"""
Full simulation bringup: underwater world + manipulator + controllers + camera.

Brings up, in dependency order:
  1. Gazebo Harmonic with the underwater world (buoyancy, hydrodynamics, lights)
  2. robot_state_publisher fed from the xacro
  3. the arm spawned onto the pedestal at z = 0.5
  4. ros_gz bridges for clock, camera and the grasp latch topics
  5. joint_state_broadcaster, then arm_controller and gripper_controller

The controller spawners are chained on process-exit events rather than fired in
parallel. gz_ros2_control only creates the controller_manager once the model is
inserted into the world, so an unsequenced spawner races the simulator and fails
with "Controller manager not available".
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gpu_env import gpu_env  # noqa: E402

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    desc_share = get_package_share_directory("uw_arm_description")
    bringup_share = get_package_share_directory("uw_arm_bringup")

    # Only the BARE FILENAME is handed to Gazebo, never the absolute path.
    # ros_gz_sim concatenates gz_args into a single string that is whitespace-
    # split downstream, so an absolute path containing a space ("ROBOTIC ARM")
    # is torn into two tokens; Gazebo then treats the first fragment as a world
    # NAME and tries to download it from Fuel, failing with
    #   "Fuel world download failed because Fetch failed"
    # and exiting 255. Resolving the world through GZ_SIM_RESOURCE_PATH avoids
    # putting a path with spaces on that command line at all.
    world_dir = os.path.join(bringup_share, "worlds")
    controllers_yaml = os.path.join(bringup_share, "config", "controllers.yaml")
    bridge_yaml = os.path.join(bringup_share, "config", "gz_bridge.yaml")
    xacro_file = os.path.join(desc_share, "urdf", "uw_arm.urdf.xacro")

    # Gazebo resolves package:// against this path. Both packages' parents must
    # be present or the STL meshes and the seabed texture will fail to load.
    resource_path = os.pathsep.join([
        os.path.dirname(desc_share),
        os.path.dirname(bringup_share),
        world_dir,
        os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
    ])

    # Paths are quoted because launch's Command substitution shlex-splits the
    # assembled string, and this workspace lives under a directory with a space
    # in its name ("ROBOTIC ARM"). Without the quotes xacro is handed two args.
    robot_description = ParameterValue(
        Command([
            'xacro "', xacro_file, '"',
            " use_gazebo:=true",
            " use_camera:=", LaunchConfiguration("use_camera"),
            " use_gripper:=true",
            ' controllers_yaml:="', controllers_yaml, '"',
        ]),
        value_type=str,
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"
            ])
        ),
        launch_arguments={
            "gz_args": [LaunchConfiguration("gz_args"), " ",
                        LaunchConfiguration("world")],
            "on_exit_shutdown": "true",
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "uw_arm",
            # z=0: the pedestal height is carried by the URDF's world_to_base
            # joint so TF and Gazebo agree. Do not add it here as well.
            "-x", "0.0", "-y", "0.0", "-z", "0.0",
            "-allow_renaming", "false",
        ],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        parameters=[{"config_file": bridge_yaml, "use_sim_time": True}],
    )

    # ros_gz_image rather than the generic bridge: republishes through
    # image_transport and avoids a full uncompressed copy at 20 Hz.
    # ONE image_bridge PROCESS PER TOPIC. A single image_bridge given several
    # image topics interleaves them: frames from one camera surface on another
    # camera's ROS topic. Recording it looks like the renderer is flickering -
    # the scene view intermittently shows the wrist camera's content and vice
    # versa. Separate processes keep each stream on its own topic.
    image_bridges = [
        Node(
            package="ros_gz_image",
            executable="image_bridge",
            name="image_bridge_" + topic.strip("/").replace("/", "_"),
            output="screen",
            arguments=[topic],
            parameters=[{"use_sim_time": True}],
        )
        for topic in ("/wrist_camera/image",
                      "/wrist_camera/depth_image",
                      "/scene_camera/image")
    ]

    # Gazebo publishes object poses on /gz/tf parented to the world name
    # ("underwater"), while robot_state_publisher roots the robot at "world".
    # Without this identity link the two TF trees are disconnected and no lookup
    # between base_link and target_canister can succeed.
    world_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_underwater",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "underwater"],
        parameters=[{"use_sim_time": True}],
    )


    object_tf = Node(
        package="uw_arm_bringup",
        executable="object_tf_publisher.py",
        output="log",
        parameters=[{"world": "underwater", "parent_frame": "world",
                     "use_sim_time": True}],
    )

    # Kinematic grasp weld. gz-sim has no gazebo_grasp_plugin equivalent and its
    # DetachableJoint cannot attach on demand, so the hold is done here.
    grasp_manager = Node(
        package="uw_arm_bringup",
        executable="grasp_manager.py",
        output="screen",
        parameters=[{"world": "underwater", "payload": "target_canister",
                     "gripper_frame": "grasp_link", "reference_frame": "world",
                     "use_sim_time": True}],
    )

    def spawner(name):
        return Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=[name, "--controller-manager", "/controller_manager",
                       "--controller-manager-timeout", "60"],
        )

    jsb = spawner("joint_state_broadcaster")
    arm_ctl = spawner("arm_controller")
    grip_ctl = spawner("gripper_controller")

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        condition=IfCondition(LaunchConfiguration("rviz")),
        parameters=[{"use_sim_time": True}],
        arguments=["-d", os.path.join(desc_share, "rviz", "uw_arm.rviz")],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "gz_args", default_value="-r -v 3",
            description="-r runs immediately; add -s for headless."),
        DeclareLaunchArgument(
            "world", default_value="underwater.sdf",
            description="World FILENAME, resolved via GZ_SIM_RESOURCE_PATH. "
                        "Must not be an absolute path - see note in this file."),
        DeclareLaunchArgument(
            "use_camera", default_value="true",
            description="false drops the wrist RGB-D sensor"),
        DeclareLaunchArgument("rviz", default_value="true"),

        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
        *gpu_env(),

        gz_sim,
        robot_state_publisher,
        spawn_robot,
        bridge,
        *image_bridges,
        world_link,
        object_tf,
        grasp_manager,

        RegisterEventHandler(
            OnProcessExit(target_action=spawn_robot, on_exit=[jsb])
        ),
        RegisterEventHandler(
            OnProcessExit(target_action=jsb, on_exit=[arm_ctl])
        ),
        RegisterEventHandler(
            OnProcessExit(target_action=arm_ctl, on_exit=[grip_ctl, rviz])
        ),
    ])
