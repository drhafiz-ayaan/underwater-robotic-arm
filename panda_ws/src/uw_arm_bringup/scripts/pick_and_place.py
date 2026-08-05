#!/usr/bin/env python3
"""
Closed-loop pick-and-place driven by the target's measured pose.

This REPLACES demo_sequence.py, which only played back hand-picked joint angles
and never went anywhere near the object. Here every arm pose is solved by inverse
kinematics from where the target actually is, so the gripper arrives at the
canister, closes on it, carries it and drops it in the basket.

WHERE THE TARGET POSE COMES FROM
    The frame "target_canister" on /gz/tf. Today that is Gazebo ground truth;
    in Phase 2 the perception node publishes the same frame name from the wrist
    RGB-D stream and nothing below changes.

IK
    Damped least squares on a 5-DOF task: full position, plus alignment of the
    gripper approach axis with the commanded direction. Roll about the approach
    axis is left free on purpose - for a square canister any roll grasps a face,
    and freeing it turns an often-unsolvable 6-DOF request into one this arm's
    fairly tight joint limits can satisfy.

    The joint axis points in the URDF are recovered estimates, but IK is solved
    against that same description, so the simulation is internally consistent and
    the grasp lands. Re-exporting from SolidWorks changes the numbers, not this.

GRASP
    Jaw friction alone will not hold a wet object through a transfer, so the
    Gazebo DetachableJoint is latched via /gripper/attach once the jaws close,
    and released with /gripper/detach. The jaws still close on the object.
"""

import math
import sys

import numpy as np
import rclpy
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory, GripperCommand
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Empty, String
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Weight on joint-space distance when ranking IK solutions. Low values let
# successive waypoints pick geometrically valid but wildly different arm
# configurations; the arm then whips between them and a grasped payload is torn
# out of the jaws mid-transfer. High values keep the motion smooth and local.
CONTINUITY = 0.45

JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
HOME = [1.5708, 0.0, 1.5708, 1.5708, 0.0, 1.5708]
TARGET_FRAME = "target_canister"
PLACE_XYZ = (0.35, 0.25, 0.60)    # 0.10 m above the basket rim, world frame


# ----------------------------------------------------------------- kinematics
def _vec(txt, default="0 0 0"):
    return np.array([float(v) for v in (txt or default).split()])


def _rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


def _axis_R(axis, theta):
    u = axis / np.linalg.norm(axis)
    K = np.array([[0, -u[2], u[1]], [u[2], 0, -u[0]], [-u[1], u[0], 0]])
    return np.eye(3) + math.sin(theta) * K + (1 - math.cos(theta)) * K @ K


class Chain:
    """Forward kinematics and Jacobian for base_link -> grasp_link."""

    ORDER = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6",
             "link6_to_tool0", "tool0_to_palm", "palm_to_grasp"]

    def __init__(self, urdf_xml):
        root = ET.fromstring(urdf_xml)
        joints = {j.get("name"): j for j in root.findall("joint")}
        self.steps, self.limits = [], []
        for name in self.ORDER:
            j = joints[name]
            o = j.find("origin")
            ax = j.find("axis")
            lim = j.find("limit")
            self.steps.append((
                j.get("type"),
                _vec(o.get("xyz") if o is not None else None),
                _rpy_to_R(*_vec(o.get("rpy") if o is not None else None, "0 0 0")),
                _vec(ax.get("xyz")) if ax is not None else None,
            ))
            if j.get("type") == "revolute":
                self.limits.append((float(lim.get("lower")), float(lim.get("upper"))))
        self.limits = np.array(self.limits)

    def fk(self, q):
        """Return (position, rotation) of grasp_link in base_link."""
        p, R, k = np.zeros(3), np.eye(3), 0
        for typ, o, Ro, ax in self.steps:
            p = p + R @ o
            R = R @ Ro
            if typ == "revolute":
                R = R @ _axis_R(ax, q[k])
                k += 1
        return p, R

    def _frames(self, q):
        out, p, R, k = [], np.zeros(3), np.eye(3), 0
        for typ, o, Ro, ax in self.steps:
            p = p + R @ o
            R = R @ Ro
            if typ == "revolute":
                out.append((p.copy(), R @ ax / np.linalg.norm(ax)))
                R = R @ _axis_R(ax, q[k])
                k += 1
        return out, p, R

    def jacobian(self, q):
        axes, pe, _ = self._frames(q)
        J = np.zeros((6, len(q)))
        for i, (pi, zi) in enumerate(axes):
            J[:3, i] = np.cross(zi, pe - pi)
            J[3:, i] = zi
        return J

    def _pos_ik(self, p_goal, q0, iters=300, tol=1e-3):
        """Position-only damped least squares from one seed."""
        q = np.array(q0, float)
        lo, hi = self.limits[:, 0], self.limits[:, 1]
        for _ in range(iters):
            p, _ = self.fk(q)
            e = p_goal - p
            if np.linalg.norm(e) < tol:
                return q, True
            J = self.jacobian(q)[:3, :]
            dq = J.T @ np.linalg.solve(J @ J.T + 0.0025 * np.eye(3), e)
            q = np.clip(q + np.clip(dq, -0.3, 0.3), lo, hi)
        p, _ = self.fk(q)
        return q, np.linalg.norm(p_goal - p) < 2e-3

    def ik(self, p_goal, approach, q0, restarts=120, tol=4e-3, seed=0):
        """
        Solve position exactly, then choose the solution whose gripper approach
        axis best matches `approach`.

        Orientation is NOT commanded as a hard constraint. This arm's limits
        (four joints restricted to 0..pi) do not admit a straight top-down grasp
        at the panel - the most downward approach reachable there is about 53
        degrees off vertical - so demanding [0,0,-1] makes every solve fail even
        though the point itself is exactly reachable. Solving position first and
        then ranking by approach alignment always returns the best grasp the arm
        can actually strike.

        Solutions are also scored on joint-space distance from q0, so successive
        waypoints do not trigger a full reconfiguration mid-transfer.

        Returns (q, ok, achieved_approach_axis).
        """
        a_goal = np.asarray(approach, float)
        a_goal = a_goal / np.linalg.norm(a_goal)
        lo, hi = self.limits[:, 0], self.limits[:, 1]
        rng = np.random.default_rng(seed)
        q0 = np.clip(np.array(q0, float), lo, hi)

        best = None
        for i in range(restarts + 1):
            start = q0 if i == 0 else rng.uniform(lo, hi)
            q, ok = self._pos_ik(p_goal, start)
            if not ok:
                continue
            p, R = self.fk(q)
            if np.linalg.norm(p - p_goal) > tol:
                continue
            a = R @ np.array([0.0, 0.0, 1.0])
            score = float(a @ a_goal) - CONTINUITY * float(np.linalg.norm(q - q0))
            if best is None or score > best[0]:
                best = (score, q, a)
        if best is None:
            return np.array(q0), False, np.array([0.0, 0.0, -1.0])
        return best[1], True, best[2]


def _skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


# ----------------------------------------------------------------------- node
class PickAndPlace(Node):
    def __init__(self):
        super().__init__("pick_and_place")
        share = get_package_share_directory("uw_arm_description")
        import subprocess
        urdf = subprocess.run(
            ["xacro", share + "/urdf/uw_arm.urdf.xacro", "use_gazebo:=false"],
            capture_output=True, text=True, check=True).stdout
        self.chain = Chain(urdf)

        self.status = self.create_publisher(String, "/demo/status", 10)
        self.attach = self.create_publisher(Empty, "/gripper/attach", 10)
        self.detach = self.create_publisher(Empty, "/gripper/detach", 10)
        self.arm = ActionClient(self, FollowJointTrajectory,
                                "/arm_controller/follow_joint_trajectory")
        self.grip = ActionClient(self, GripperCommand,
                                 "/gripper_controller/gripper_cmd")
        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)
        self.q = list(HOME)

    def say(self, text):
        self.get_logger().info(text)
        self.status.publish(String(data=text))

    def target_in_base(self, timeout=25.0):
        """Target position expressed in base_link. Falls back to /gz/tf raw."""
        from rclpy.duration import Duration
        deadline = self.get_clock().now() + Duration(seconds=timeout)
        while self.get_clock().now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            try:
                tr = self.tf_buf.lookup_transform(
                    "base_link", TARGET_FRAME, rclpy.time.Time())
            except Exception:
                continue
            t = tr.transform.translation
            return np.array([t.x, t.y, t.z])
        return None

    def point_in_base(self, xyz_world):
        """Convert a world-frame point into base_link."""
        for _ in range(60):
            try:
                tr = self.tf_buf.lookup_transform(
                    "base_link", "world", rclpy.time.Time())
            except Exception:
                rclpy.spin_once(self, timeout_sec=0.1)
                continue
            t = tr.transform.translation
            # world_to_base is a pure translation, so this is just an offset.
            return np.array(xyz_world) + np.array([t.x, t.y, t.z])
        raise RuntimeError("cannot transform world point into base_link")

    def goto(self, q, seconds, caption=None):
        if caption:
            self.say(caption)
        traj = JointTrajectory()
        traj.joint_names = JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in q]
        pt.velocities = [0.0] * 6
        pt.time_from_start.sec = int(seconds)
        pt.time_from_start.nanosec = int((seconds % 1.0) * 1e9)
        traj.points.append(pt)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        fut = self.arm.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        h = fut.result()
        if h is None or not h.accepted:
            self.get_logger().error("trajectory rejected")
            return False
        res = h.get_result_async()
        rclpy.spin_until_future_complete(self, res, timeout_sec=seconds + 15.0)
        self.q = list(q)
        return True

    def gripper(self, position, caption=None):
        if caption:
            self.say(caption)
        if not self.grip.server_is_ready():
            return
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = 50.0
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=8.0)
        h = fut.result()
        if h is not None and h.accepted:
            rclpy.spin_until_future_complete(self, h.get_result_async(),
                                             timeout_sec=8.0)

    def latch(self):
        """
        Latch the detachable joint while the gripper is stationary on the object.

        Published repeatedly over ~1.5 s rather than once. The first message on a
        freshly created publisher is routinely lost to discovery, and a latch
        that lands LATE is worse than none: the joint is created between two
        bodies that have already separated, and the solver resolves that offset
        violently - the payload is flung across the scene mid-transfer.
        """
        self.say("Latching grasp")
        for _ in range(15):
            self.attach.publish(Empty())
            for _ in range(2):
                rclpy.spin_once(self, timeout_sec=0.05)
        self.say("Grasp latched")

    def solve(self, p_base, approach, label):
        q, ok, axis = self.chain.ik(p_base, approach, self.q)
        p, _ = self.chain.fk(q)
        err = np.linalg.norm(p - p_base) * 1000
        self.get_logger().info(
            "IK %-14s target=[%.3f %.3f %.3f] residual=%.1f mm "
            "approach=[%.2f %.2f %.2f] %s"
            % (label, *p_base, err, *axis, "OK" if ok else "FAILED"))
        return q, ok, axis

    def run(self):
        self.say("Waiting for controllers")
        if not self.arm.wait_for_server(timeout_sec=90.0):
            self.get_logger().error("arm_controller unavailable")
            return False
        self.grip.wait_for_server(timeout_sec=15.0)

        self.goto(HOME, 4.0, "Home posture")
        self.gripper(0.035, "Opening jaws")

        self.say("Locating target from TF")
        p = self.target_in_base()
        if p is None:
            self.get_logger().error("frame '%s' never appeared" % TARGET_FRAME)
            return False
        self.get_logger().info("target in base_link: [%.3f %.3f %.3f]" % (*p,))

        down = np.array([0.0, 0.0, -1.0])   # preferred, not demanded

        # Solve the grasp first: its achieved approach axis defines the straight
        # line the gripper must back off along, so the retreat is guaranteed
        # collision-free rather than a guess at "straight up".
        q_grasp, ok, axis = self.solve(p, down, "grasp")
        if not ok:
            self.get_logger().error("target not reachable")
            return False

        q, ok, _ = self.solve(p - axis * 0.13, down, "pre-grasp")
        if not ok:
            self.get_logger().error("no pre-grasp solution")
            return False
        self.goto(q, 5.0, "Approaching target")

        self.goto(q_grasp, 3.5, "Closing on target")

        self.gripper(0.004, "Closing jaws on target")
        self.latch()

        # Retreat back along the approach line, then straight up.
        q, ok, _ = self.solve(p - axis * 0.13, down, "retreat")
        if ok:
            self.goto(q, 3.0, "Lifting clear of the panel")
        q, ok, _ = self.solve(p - axis * 0.13 + np.array([0.0, 0.0, 0.12]),
                              down, "lift")
        if ok:
            self.goto(q, 4.0, "Raising the payload")

        place = self.point_in_base(PLACE_XYZ)
        q, ok, place_axis = self.solve(place, down, "over-basket")
        if not ok:
            self.get_logger().error("drop-off point not reachable")
            return False
        q_hi, hi_ok, _ = self.solve(place - place_axis * 0.15, down, "pre-place")
        if hi_ok:
            self.goto(q_hi, 8.0, "Transferring to drop-off basket")
        self.goto(q, 4.5, "Lowering into basket")

        self.detach.publish(Empty())
        self.gripper(0.035, "Releasing target")
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.05)

        self.goto(HOME, 5.0, "Returning home")
        self.say("Pick and place complete")
        return True


def main():
    rclpy.init()
    node = PickAndPlace()
    ok = False
    try:
        ok = node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
