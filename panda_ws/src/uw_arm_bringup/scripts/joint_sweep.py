#!/usr/bin/env python3
"""
Publishes a slow sinusoidal sweep on /joint_states.

Exists so the recovered kinematics can be checked in RViz WITHOUT
joint_state_publisher_gui, which is not part of a default ros-jazzy-desktop
install. It also sweeps every joint through its full declared range
automatically, which is a better check than dragging sliders by hand: watch each
link and confirm it pivots about a plausible hinge instead of swinging the whole
arm about the base.
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# name, lower, upper - must match uw_arm.urdf.xacro
ARM_JOINTS = [
    ("joint1", 0.0, 3.142),
    ("joint2", -1.57, 1.57),
    ("joint3", 0.0, 3.142),
    ("joint4", 0.0, 3.142),
    ("joint5", -1.57, 1.57),
    ("joint6", 0.0, 3.14),
]
GRIPPER_JOINTS = [("finger_left_joint", 0.0, 0.035),
                  ("finger_right_joint", 0.0, 0.035)]


class JointSweep(Node):
    def __init__(self):
        super().__init__("joint_sweep")
        self.declare_parameter("period", 12.0)
        self.declare_parameter("stagger", 1.6)
        self.declare_parameter("margin", 0.05)
        self.pub = self.create_publisher(JointState, "joint_states", 10)
        self.t0 = self.get_clock().now()
        self.create_timer(1.0 / 30.0, self.tick)
        self.get_logger().info("sweeping %d joints" % len(ARM_JOINTS))

    def tick(self):
        period = self.get_parameter("period").value
        stagger = self.get_parameter("stagger").value
        margin = self.get_parameter("margin").value
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        for i, (name, lo, hi) in enumerate(ARM_JOINTS):
            # Stay just inside the limits so RViz never flags an out-of-range
            # joint while the sweep is at an extreme.
            span = (hi - lo) * (1.0 - 2.0 * margin)
            mid = 0.5 * (lo + hi)
            phase = 2.0 * math.pi * (t - i * stagger) / period
            msg.name.append(name)
            msg.position.append(mid + 0.5 * span * math.sin(phase))

        grip = 0.0175 * (1.0 + math.sin(2.0 * math.pi * t / (period * 0.5)))
        for name, lo, hi in GRIPPER_JOINTS:
            msg.name.append(name)
            msg.position.append(max(lo, min(hi, grip)))

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = JointSweep()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
