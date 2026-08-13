#!/usr/bin/env python3
"""
Publishes TF frames for the world's objects.

WHY THIS EXISTS
    ros_gz_bridge can convert gz.msgs.Pose_V to tf2_msgs/TFMessage, but on Jazzy
    the conversion leaves frame_id and child_frame_id EMPTY for both
    /world/<w>/pose/info and /world/<w>/dynamic_pose/info. Every object arrives
    as an anonymous transform, so there is no way to ask for the canister. The
    PoseArray conversion drops the names too. Gazebo itself has them - they are
    right there in `gz topic -e` - so this node reads that and republishes
    properly named frames.

    It shells out to `gz topic` rather than binding gz-transport because the
    Python bindings are not part of the ROS 2 Jazzy packaging here, and at 2 Hz
    for a handful of static-ish objects the cost is irrelevant.

ROLE IN THE PROJECT
    This is Interim scaffolding: ground truth standing in for perception. The
    perception perception node publishes the SAME frame name, `target_canister`,
    estimated from the wrist RGB-D stream. pick_and_place.py consumes the frame
    either way and does not care which is running - run one or the other, never
    both.
"""

import re
import subprocess

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

# One "pose { ... }" block: name, then position, then orientation.
POSE_RE = re.compile(
    r'name:\s*"([^"]+)".*?position\s*\{(.*?)\}.*?orientation\s*\{(.*?)\}',
    re.S)
NUM_RE = re.compile(r'([xyzw]):\s*(-?[\d.eE+-]+)')


def _nums(block, default):
    out = dict(default)
    for k, v in NUM_RE.findall(block):
        out[k] = float(v)
    return out


class ObjectTf(Node):
    def __init__(self):
        super().__init__("object_tf_publisher")
        self.declare_parameter("world", "underwater")
        self.declare_parameter("parent_frame", "world")
        self.declare_parameter("rate", 2.0)
        self.declare_parameter(
            "objects",
            ["target_canister", "drift_float", "neutral_pod",
             "dropoff_basket", "intervention_panel"])

        self.world = self.get_parameter("world").value
        self.parent = self.get_parameter("parent_frame").value
        self.wanted = set(self.get_parameter("objects").value)
        self.bc = TransformBroadcaster(self)
        self.warned = False
        self.create_timer(1.0 / float(self.get_parameter("rate").value), self.tick)
        self.get_logger().info(
            "publishing TF for %s under '%s'"
            % (", ".join(sorted(self.wanted)), self.parent))

    def _read(self):
        for topic in ("pose/info", "dynamic_pose/info"):
            try:
                r = subprocess.run(
                    ["gz", "topic", "-e", "-t",
                     "/world/%s/%s" % (self.world, topic), "-n", "1"],
                    capture_output=True, text=True, timeout=5.0)
                if r.stdout.strip():
                    return r.stdout
            except (subprocess.TimeoutExpired, FileNotFoundError):
                continue
        return ""

    def tick(self):
        text = self._read()
        if not text:
            if not self.warned:
                self.get_logger().warn("no pose data from gz - is the world running?")
                self.warned = True
            return
        self.warned = False

        now = self.get_clock().now().to_msg()
        sent = 0
        for name, pos, ori in POSE_RE.findall(text):
            if name not in self.wanted:
                continue
            p = _nums(pos, {"x": 0.0, "y": 0.0, "z": 0.0})
            o = _nums(ori, {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0})
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.parent
            t.child_frame_id = name
            t.transform.translation.x = p["x"]
            t.transform.translation.y = p["y"]
            t.transform.translation.z = p["z"]
            t.transform.rotation.x = o["x"]
            t.transform.rotation.y = o["y"]
            t.transform.rotation.z = o["z"]
            t.transform.rotation.w = o["w"]
            self.bc.sendTransform(t)
            sent += 1
        if sent == 0 and not self.warned:
            self.get_logger().warn("pose data had none of the requested objects")
            self.warned = True


def main():
    rclpy.init()
    node = ObjectTf()
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
