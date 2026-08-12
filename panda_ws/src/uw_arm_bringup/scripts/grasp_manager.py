#!/usr/bin/env python3
"""
Kinematic grasp attachment for Gazebo Harmonic.

WHY THIS EXISTS
    Holding an object with contact friction alone is unreliable in gz-sim: the
    jaws must penetrate the object to generate normal force, and the contact
    solver resolves that penetration by ejecting it. Gazebo Classic solved this
    with gazebo_grasp_plugin, which welds a fixed joint on contact. There is no
    Harmonic equivalent, and gz-sim 8.11's DetachableJoint cannot attach on
    demand - it ignores <attach_topic>, so /gripper/attach silently does nothing
    (verified by publishing it and driving the arm away with the target left
    behind).

    So the weld is done here instead: on attach, the payload's pose relative to
    the gripper is recorded, and from then on the payload is rigidly driven to
    that relative pose through Gazebo's set_pose service until release.

WHAT IS AND IS NOT SIMULATED
    Real: perception of the target, the IK solution, the arm trajectory, the
    fluid dynamics, and the contact between jaws and payload during approach.
    Simulated: the grasp ADHESION only - exactly the role gazebo_grasp_plugin
    plays in a Classic-based stack. The arm still has to reach the object and
    close on it correctly; nothing here rescues a bad grasp pose, because the
    relative transform is captured from wherever the gripper actually is.

    pick_and_place.py commands the jaws to leave ~1 mm of clearance per side
    while attached. Squeezing onto a payload that is being pose-driven would put
    the contact solver in a fight with the teleport and shake the whole arm.

INTERFACE
    /gripper/attach  (std_msgs/Empty) - weld payload to the gripper
    /gripper/detach  (std_msgs/Empty) - release
    /gripper/attached (std_msgs/Bool, latched) - current state
"""

import math

import rclpy
from gz.msgs10.pose_pb2 import Pose as GzPose
from gz.msgs10.boolean_pb2 import Boolean as GzBoolean
from gz.transport13 import Node as GzNode
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Empty
from tf2_ros import Buffer, TransformListener


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_conj(q):
    x, y, z, w = q
    return (-x, -y, -z, w)


def quat_rot(q, v):
    """Rotate vector v by quaternion q."""
    qv = (v[0], v[1], v[2], 0.0)
    r = quat_mul(quat_mul(q, qv), quat_conj(q))
    return (r[0], r[1], r[2])


class GraspManager(Node):
    def __init__(self):
        super().__init__("grasp_manager")
        self.declare_parameter("world", "underwater")
        self.declare_parameter("payload", "target_canister")
        self.declare_parameter("gripper_frame", "grasp_link")
        self.declare_parameter("reference_frame", "world")
        self.declare_parameter("rate", 60.0)

        self.world = self.get_parameter("world").value
        self.payload = self.get_parameter("payload").value
        self.gripper_frame = self.get_parameter("gripper_frame").value
        self.ref = self.get_parameter("reference_frame").value
        self.service = "/world/%s/set_pose" % self.world

        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)
        self.gz = GzNode()

        self.offset = None          # payload pose in gripper frame
        self.failures = 0

        latched = QoSProfile(depth=1,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(Bool, "/gripper/attached", latched)
        self.create_subscription(Empty, "/gripper/attach", self.on_attach, 10)
        self.create_subscription(Empty, "/gripper/detach", self.on_detach, 10)
        self.create_timer(1.0 / float(self.get_parameter("rate").value), self.tick)
        self.publish_state(False)
        self.get_logger().info(
            "ready - payload '%s', gripper frame '%s', service %s"
            % (self.payload, self.gripper_frame, self.service))

    # ------------------------------------------------------------------ utils
    def publish_state(self, attached):
        self.state_pub.publish(Bool(data=bool(attached)))

    def gripper_pose(self):
        """(translation, quaternion) of the gripper frame in the world frame."""
        try:
            tr = self.tf_buf.lookup_transform(self.ref, self.gripper_frame,
                                              rclpy.time.Time())
        except Exception:
            return None
        t = tr.transform.translation
        r = tr.transform.rotation
        return (t.x, t.y, t.z), (r.x, r.y, r.z, r.w)

    def payload_pose(self):
        """Payload pose in the world frame, via the TF that object_tf_publisher
        broadcasts from Gazebo ground truth."""
        try:
            tr = self.tf_buf.lookup_transform(self.ref, self.payload,
                                              rclpy.time.Time())
        except Exception:
            return None
        t = tr.transform.translation
        r = tr.transform.rotation
        return (t.x, t.y, t.z), (r.x, r.y, r.z, r.w)

    def set_pose(self, pos, quat):
        req = GzPose()
        req.name = self.payload
        req.position.x, req.position.y, req.position.z = pos
        (req.orientation.x, req.orientation.y,
         req.orientation.z, req.orientation.w) = quat
        try:
            _, ok = self.gz.request(self.service, req, GzPose, GzBoolean, 200)
            return ok
        except Exception:
            return False

    # --------------------------------------------------------------- handlers
    def on_attach(self, _msg):
        if self.offset is not None:
            return
        g = self.gripper_pose()
        p = self.payload_pose()
        if g is None or p is None:
            self.get_logger().error(
                "cannot attach - missing TF for %s"
                % (self.gripper_frame if g is None else self.payload))
            return
        gp, gq = g
        pp, pq = p
        # payload pose expressed in the gripper frame
        d = (pp[0] - gp[0], pp[1] - gp[1], pp[2] - gp[2])
        inv = quat_conj(gq)
        self.offset = (quat_rot(inv, d), quat_mul(inv, pq))
        self.failures = 0
        self.publish_state(True)
        dist = math.sqrt(sum(c * c for c in self.offset[0]))
        self.get_logger().info(
            "ATTACHED '%s' at %.1f mm from %s"
            % (self.payload, dist * 1000.0, self.gripper_frame))

    def on_detach(self, _msg):
        if self.offset is None:
            return
        self.offset = None
        self.publish_state(False)
        self.get_logger().info("RELEASED '%s'" % self.payload)

    def tick(self):
        if self.offset is None:
            return
        g = self.gripper_pose()
        if g is None:
            return
        gp, gq = g
        off_p, off_q = self.offset
        rot = quat_rot(gq, off_p)
        pos = (gp[0] + rot[0], gp[1] + rot[1], gp[2] + rot[2])
        quat = quat_mul(gq, off_q)
        if self.set_pose(pos, quat):
            self.failures = 0
        else:
            self.failures += 1
            if self.failures in (10, 100):
                self.get_logger().warn(
                    "set_pose failing (%d consecutive) - is %s advertised?"
                    % (self.failures, self.service))


def main():
    rclpy.init()
    node = GraspManager()
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
