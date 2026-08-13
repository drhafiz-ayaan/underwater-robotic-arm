#!/usr/bin/env python3
"""
Scripted demonstration sequence.

Drives the arm through joint-space waypoints via arm_controller and actuates the
gripper via gripper_controller, publishing a caption on /demo/status that
record_demo.py burns into the video.

This is deliberately JOINT SPACE, not a Cartesian pick. The joint axis points in
uw_arm.urdf.xacro are recovered estimates accurate to a few mm, so a Cartesian
grasp claim would not be honest until the SolidWorks re-export lands. What this
does demonstrate is the full simulation stack: controllers accepting trajectories,
the gripper actuating, the wrist camera streaming, and the arm moving under
hydrodynamic loading. Perception-driven picking (pick_and_place.py) replaces it.
"""

import math

import rclpy
from control_msgs.action import FollowJointTrajectory, GripperCommand
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

HOME = [1.5708, 0.0, 1.5708, 1.5708, 0.0, 1.5708]

# (caption, joint targets, seconds to reach). All within the declared limits:
# j1 [0,3.142] j2 [-1.57,1.57] j3 [0,3.142] j4 [0,3.142] j5 [-1.57,1.57] j6 [0,3.14]
WAYPOINTS = [
    ("Home posture",                 HOME,                                        4.0),
    ("Base yaw - joint 1 sweep",     [2.60, 0.00, 1.5708, 1.5708, 0.00, 1.5708],  5.0),
    ("Shoulder pitch - joint 2",     [2.60, 0.85, 1.5708, 1.5708, 0.00, 1.5708],  4.0),
    ("Elbow - joints 3 and 4",       [2.60, 0.85, 2.35, 0.95, 0.00, 1.5708],      5.0),
    ("Wrist articulation - 5 and 6", [2.60, 0.85, 2.35, 0.95, -0.90, 2.60],       4.0),
    ("Reach toward work panel",      [0.90, -0.55, 1.05, 2.20, 0.60, 0.80],       6.0),
    ("Descend to panel height",      [0.90, -0.90, 0.75, 2.45, 0.60, 0.80],       4.0),
    ("Lift clear of panel",          [0.90, -0.20, 1.30, 1.90, 0.40, 0.80],       4.0),
    ("Transfer to drop-off",         [2.10, 0.30, 1.80, 1.40, 0.00, 2.00],        6.0),
    ("Return home",                  HOME,                                        5.0),
]

# (caption, gripper position in metres) fired between the waypoints above
GRIPPER_AT = {
    5: ("Opening jaws", 0.035),
    6: ("Closing jaws on target", 0.004),
    8: ("Releasing at drop-off", 0.035),
}


class DemoSequence(Node):
    def __init__(self):
        super().__init__("demo_sequence")
        self.status = self.create_publisher(String, "/demo/status", 10)
        self.arm = ActionClient(self, FollowJointTrajectory,
                                "/arm_controller/follow_joint_trajectory")
        self.grip = ActionClient(self, GripperCommand,
                                 "/gripper_controller/gripper_cmd")

    def say(self, text):
        self.get_logger().info(text)
        self.status.publish(String(data=text))

    def wait_for_servers(self, timeout=90.0):
        self.say("Waiting for controllers")
        if not self.arm.wait_for_server(timeout_sec=timeout):
            self.get_logger().error("arm_controller action server never appeared")
            return False
        if not self.grip.wait_for_server(timeout_sec=20.0):
            self.get_logger().warn("gripper_controller absent - continuing without it")
        return True

    def send_arm(self, positions, seconds):
        traj = JointTrajectory()
        traj.joint_names = JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in positions]
        pt.velocities = [0.0] * len(JOINTS)
        pt.time_from_start.sec = int(seconds)
        pt.time_from_start.nanosec = int((seconds % 1.0) * 1e9)
        traj.points.append(pt)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        fut = self.arm.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        handle = fut.result()
        if handle is None or not handle.accepted:
            self.get_logger().error("trajectory goal rejected")
            return
        res = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res, timeout_sec=seconds + 15.0)

    def send_grip(self, position):
        if not self.grip.server_is_ready():
            return
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = 50.0
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        handle = fut.result()
        if handle is None or not handle.accepted:
            return
        res = handle.get_result_async()
        # allow_stalling is on, so a successful grasp reports as a stall
        rclpy.spin_until_future_complete(self, res, timeout_sec=8.0)

    def run(self):
        if not self.wait_for_servers():
            return
        self.say("Underwater manipulator bring-up")
        for i, (caption, targets, secs) in enumerate(WAYPOINTS):
            if i in GRIPPER_AT:
                gcap, gpos = GRIPPER_AT[i]
                self.say(gcap)
                self.send_grip(gpos)
            self.say(caption)
            self.send_arm(targets, secs)
        self.say("Sequence complete")


def main():
    rclpy.init()
    node = DemoSequence()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
