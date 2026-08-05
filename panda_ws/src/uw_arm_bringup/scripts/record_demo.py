#!/usr/bin/env python3
"""
Records an MP4 from the in-simulation cameras.

Frames come from the Gazebo camera sensors over the ROS bridge, not from a
screen grab: the result is deterministic, has no window chrome or mouse pointer,
and needs neither ffmpeg nor a compositor. Requires GPU offscreen rendering,
which on this machine means the NVIDIA EGL vendor must be forced (see the
SetEnvironmentVariable calls in record_demo.launch.py).

Layout: third-person scene camera fills the frame, the wrist camera sits
picture-in-picture, and the current /demo/status caption is burned in.
"""

import os

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

FG = (236, 240, 241)
ACCENT = (60, 200, 250)   # BGR amber
DIM = (150, 160, 165)


def to_bgr(msg):
    buf = np.frombuffer(msg.data, np.uint8)
    if msg.encoding in ("rgb8", "bgr8"):
        img = buf.reshape(msg.height, msg.width, 3)
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if msg.encoding == "rgb8" else img.copy()
    if msg.encoding == "rgba8":
        return cv2.cvtColor(buf.reshape(msg.height, msg.width, 4), cv2.COLOR_RGBA2BGR)
    if msg.encoding == "mono8":
        return cv2.cvtColor(buf.reshape(msg.height, msg.width), cv2.COLOR_GRAY2BGR)
    raise ValueError("unhandled encoding %s" % msg.encoding)


class Recorder(Node):
    def __init__(self):
        super().__init__("record_demo")
        self.declare_parameter("output", os.path.expanduser("~/uw_arm_demo.mp4"))
        self.declare_parameter("fps", 30)
        self.declare_parameter("duration", 90.0)
        self.declare_parameter("title", "Underwater 6-DOF Manipulator - Pick and Place")
        self.declare_parameter("subtitle", "ROS 2 Jazzy | Gazebo Harmonic | Ifra Aerial Robotics")

        self.fps = int(self.get_parameter("fps").value)
        self.path = self.get_parameter("output").value
        self.duration = float(self.get_parameter("duration").value)

        self.scene = None
        self.wrist = None
        self.caption = "Starting up"
        self.writer = None
        self.frames = 0

        # RELIABLE, matching image_transport's publisher.
        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Image, "/scene_camera/image", self.on_scene, sensor_qos)
        self.create_subscription(Image, "/wrist_camera/image", self.on_wrist, sensor_qos)
        self.create_subscription(String, "/demo/status", self.on_status, 10)
        self.get_logger().info("recording to %s" % self.path)

    def on_scene(self, msg):
        # Writing straight from the camera callback, NOT from a timer.
        # A timer here would need use_sim_time to line up with the simulation,
        # and a node whose /clock subscription has not resolved never fires its
        # timers at all - the recorder then sits silent forever and produces no
        # file. Driving off the 30 Hz sensor stream needs no clock and gives the
        # output its frame rate for free.
        self.scene = to_bgr(msg)
        self.tick()

    def on_wrist(self, msg):
        self.wrist = to_bgr(msg)

    def on_status(self, msg):
        self.caption = msg.data

    def compose(self):
        frame = self.scene.copy()
        h, w = frame.shape[:2]

        # Title bar
        bar = frame.copy()
        cv2.rectangle(bar, (0, 0), (w, 74), (28, 26, 24), -1)
        cv2.addWeighted(bar, 0.72, frame, 0.28, 0, frame)
        cv2.putText(frame, self.get_parameter("title").value, (24, 32),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.72, FG, 2, cv2.LINE_AA)
        cv2.putText(frame, self.get_parameter("subtitle").value, (24, 58),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.46, DIM, 1, cv2.LINE_AA)

        # Wrist camera picture-in-picture
        if self.wrist is not None:
            pw = w // 4
            ph = int(pw * self.wrist.shape[0] / self.wrist.shape[1])
            pip = cv2.resize(self.wrist, (pw, ph), interpolation=cv2.INTER_AREA)
            x0, y0 = w - pw - 24, 94
            frame[y0:y0 + ph, x0:x0 + pw] = pip
            cv2.rectangle(frame, (x0 - 2, y0 - 2), (x0 + pw + 2, y0 + ph + 2), ACCENT, 2)
            cv2.putText(frame, "WRIST RGB-D (eye-in-hand)", (x0, y0 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.44, ACCENT, 1, cv2.LINE_AA)

        # Caption strip
        strip = frame.copy()
        cv2.rectangle(strip, (0, h - 62), (w, h), (28, 26, 24), -1)
        cv2.addWeighted(strip, 0.72, frame, 0.28, 0, frame)
        cv2.circle(frame, (34, h - 31), 7, ACCENT, -1)
        cv2.putText(frame, self.caption, (56, h - 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.66, FG, 2, cv2.LINE_AA)

        t = self.frames / float(self.fps)
        stamp = "t = %04.1f s" % t
        (tw, _), _ = cv2.getTextSize(stamp, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
        cv2.putText(frame, stamp, (w - tw - 24, h - 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, DIM, 1, cv2.LINE_AA)
        return frame

    def tick(self):
        if self.scene is None:
            return
        frame = self.compose()
        if self.writer is None:
            h, w = frame.shape[:2]
            os.makedirs(os.path.dirname(self.path) or ".", exist_ok=True)
            # H.264 first: it produces roughly a seventh of the file that the
            # mp4v (MPEG-4 Part 2) fallback does for this content - the
            # difference between a ~15 MB attachment and a ~105 MB one. Fall
            # back only if the codec is unavailable.
            for fourcc in ("avc1", "mp4v"):
                self.writer = cv2.VideoWriter(
                    self.path, cv2.VideoWriter_fourcc(*fourcc), self.fps, (w, h))
                if self.writer.isOpened():
                    self.get_logger().info(
                        "writing %dx%d @ %d fps [%s]" % (w, h, self.fps, fourcc))
                    break
                self.writer.release()
                self.get_logger().warn("codec %s unavailable, trying next" % fourcc)
            else:
                self.get_logger().error("no usable codec for %s" % self.path)
                raise SystemExit(1)
        self.writer.write(frame)
        self.frames += 1
        if self.frames % (self.fps * 5) == 0:
            self.get_logger().info("%.0f s recorded" % (self.frames / self.fps))
        if self.frames >= int(self.duration * self.fps):
            self.get_logger().info("duration reached")
            self.close()
            raise SystemExit

    def close(self):
        if self.writer is not None:
            self.writer.release()
            self.writer = None
            self.get_logger().info("wrote %d frames (%.1f s) to %s"
                                   % (self.frames, self.frames / float(self.fps),
                                      self.path))
        elif self.frames == 0:
            self.get_logger().error(
                "no frames captured - is /scene_camera/image publishing?")


def main():
    rclpy.init()
    node = Recorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
