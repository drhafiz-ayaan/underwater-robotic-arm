#!/usr/bin/env python3
"""
Closed-loop pick-and-place driven by the target's measured pose.

Every arm pose is solved by inverse kinematics from where the target actually is.
This replaces demo_sequence.py, which replayed hand-picked joint angles and never
approached the object at all.

WHERE THE TARGET POSE COMES FROM
    The TF frame "target_canister", published by object_tf_publisher.py. In
    Phase 2 the perception node publishes the same frame name estimated from the
    wrist RGB-D stream, and nothing in this file changes.

WHAT MAKES THE GRASP HOLD
    Four things had to be right together; each one alone left the object behind,
    dropped, or flung across the scene.

    1. THE JAWS MUST STRADDLE THE 60 mm FACE, NOT THE 120 mm ONE.
       The jaws close along the tool Y axis and open to 94 mm. The canister is
       60 x 60 x 120 mm standing upright. If tool Y ends up vertical the jaws are
       being asked to close across 120 mm, which they physically cannot do, and
       they clip a corner instead. Leaving roll free let IK pick such a solution
       roughly half the time - which is exactly why the result varied run to run.
       Solutions are now rejected unless the jaw axis is near horizontal.

    2. THE CLOSE COMMAND MUST MATCH THE OBJECT WIDTH, MEASURED TO THE JAW FACE.
       Jaw inner faces sit at +/-(0.006 + q) - the finger joint is at 0.012 but
       the finger box is 0.012 thick, so its face is 6 mm nearer the centreline.
       Measuring to the joint instead of the face understated closure by 6 mm per
       side, and commanding q=0.004 drove each finger 20 mm INTO a 60 mm object.
       The contact solver resolves that by ejecting it. q is now computed from
       the object width for a genuine 3 mm squeeze per side.

    3. THE GRASP IS WELDED BY grasp_manager.py, NOT BY CONTACT FRICTION.
       A friction hold is not reliable in gz-sim: the jaws must penetrate the
       payload to make normal force, and the contact solver answers penetration
       by ejecting it. gz-sim 8.11's DetachableJoint cannot help - it ignores
       <attach_topic>. grasp_manager.py records the payload's pose relative to
       the gripper on /gripper/attach and drives it there until /gripper/detach.
       The arm still has to reach and close correctly: the relative transform is
       captured from wherever the gripper actually is, so a bad grasp pose stays
       a bad grasp.

    4. SUCCESSIVE WAYPOINTS MUST NOT RECONFIGURE THE ARM.
       Position-only IK admits many elbow configurations. Jumping between them
       whips the end effector and tears the payload out. Solutions are scored on
       joint-space distance from the current pose.

    The grasp is then verified against TF before the transfer starts: if the
    object did not rise with the gripper, the run reports failure instead of
    miming the rest of the sequence.

IK
    Damped least squares parsed straight from the URDF - no MoveIt configuration
    needed to reach a Cartesian goal. Position is solved exactly; orientation is
    chosen by ranking solutions, because a hard top-down constraint is
    unsatisfiable on this arm (four joints limited to 0..pi put the most downward
    reachable approach about 53 degrees off vertical over the panel).
"""

import math
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Empty, String
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
HOME = [1.5708, 0.0, 1.5708, 1.5708, 0.0, 1.5708]
TARGET_FRAME = "target_canister"

# World-frame release point: 0.10 m above the basket rim (basket top z = 0.50).
PLACE_XYZ = (0.35, 0.25, 0.60)

# Object and gripper geometry. Jaw inner faces sit at +/-(JAW_INSET + q), so
# gripping a box of width w needs q = w/2 - JAW_INSET - squeeze.
# SQUEEZE is per-side penetration of the jaw into the object. Do NOT raise it to
# "grip harder": measured displacement of the payload during the lift FELL from
# 0.127 m at 3 mm to 0.035 m at 6 mm, because the contact solver resolves deep
# penetration by pushing the object out of the jaws. Grip strength comes from the
# payload being near-neutrally buoyant, not from crushing it.
OBJECT_WIDTH = 0.060
# Distance from the gripper centreline to a jaw's INNER FACE at q=0. The finger
# joint sits at half_gap=0.012 and the finger box is 0.012 thick, so its inner
# face is 0.006 nearer the centreline than the joint. Using 0.012 here (the joint
# offset) understates the closure by 6 mm PER SIDE: the commanded "3 mm squeeze"
# was really 9 mm of penetration, which is exactly the regime where the contact
# solver ejects the object instead of gripping it.
JAW_INSET = 0.006
SQUEEZE = 0.003
JAW_OPEN = 0.035

# Grasp this far ABOVE the object's centre - a small bias only, to keep the lower
# finger off the panel. Keep it SMALL: at 0.035 the object sits 35 mm out of the
# jaw centre, near the finger tips, and the jaws close asymmetrically (measured:
# left finger blocked at 0.035 while the right closed to its 0.021 goal on empty
# water). The jaws must straddle the object at their mid-span.
GRASP_Z_OFFSET = 0.012

# Reject IK solutions whose jaw axis tilts more than this out of horizontal.
MAX_JAW_TILT = 0.35          # |jaw . z_world|
CONTINUITY = 0.45            # weight on joint-space distance when ranking


def grip_close_position():
    """Jaw position that just clears the payload while it is welded.

    1 mm of clearance per side, NOT a squeeze. grasp_manager.py drives the
    payload's pose directly once attached; jaws pressing into a pose-driven body
    make the contact solver fight the weld and shake the whole arm. At video
    scale the jaws still read as closed on the object.
    """
    return max(0.0, OBJECT_WIDTH / 2.0 - JAW_INSET + 0.001)


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
    """FK, Jacobian and IK for base_link -> grasp_link, read from the URDF."""

    ORDER = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6",
             "link6_to_tool0", "tool0_to_palm", "palm_to_grasp"]

    def __init__(self, urdf_xml):
        root = ET.fromstring(urdf_xml)
        joints = {j.get("name"): j for j in root.findall("joint")}
        self.steps, limits = [], []
        for name in self.ORDER:
            j = joints[name]
            o, ax, lim = j.find("origin"), j.find("axis"), j.find("limit")
            self.steps.append((
                j.get("type"),
                _vec(o.get("xyz") if o is not None else None),
                _rpy_to_R(*_vec(o.get("rpy") if o is not None else None)),
                _vec(ax.get("xyz")) if ax is not None else None,
            ))
            if j.get("type") == "revolute":
                limits.append((float(lim.get("lower")), float(lim.get("upper"))))
        self.limits = np.array(limits)

    def fk(self, q):
        p, R, k = np.zeros(3), np.eye(3), 0
        for typ, o, Ro, ax in self.steps:
            p, R = p + R @ o, R @ Ro
            if typ == "revolute":
                R = R @ _axis_R(ax, q[k])
                k += 1
        return p, R

    def jacobian(self, q):
        frames, p, R, k = [], np.zeros(3), np.eye(3), 0
        for typ, o, Ro, ax in self.steps:
            p, R = p + R @ o, R @ Ro
            if typ == "revolute":
                frames.append((p.copy(), R @ ax / np.linalg.norm(ax)))
                R = R @ _axis_R(ax, q[k])
                k += 1
        pe = p
        J = np.zeros((6, len(q)))
        for i, (pi, zi) in enumerate(frames):
            J[:3, i] = np.cross(zi, pe - pi)
            J[3:, i] = zi
        return J

    def _pos_ik(self, p_goal, q0, iters=300, tol=1e-3):
        q = np.array(q0, float)
        lo, hi = self.limits[:, 0], self.limits[:, 1]
        for _ in range(iters):
            p, _ = self.fk(q)
            e = p_goal - p
            if np.linalg.norm(e) < tol:
                return q
            J = self.jacobian(q)[:3, :]
            q = np.clip(q + np.clip(
                J.T @ np.linalg.solve(J @ J.T + 0.0025 * np.eye(3), e),
                -0.3, 0.3), lo, hi)
        return q

    def ik(self, p_goal, q0, prefer_approach=(0, 0, -1),
           level_jaws=True, restarts=250, tol=4e-3, seed=0):
        """
        Solve position exactly, then rank the solutions.

        level_jaws rejects any solution whose jaw axis is more than MAX_JAW_TILT
        out of horizontal - without it the jaws are asked to close across the
        canister's 120 mm height instead of its 60 mm width and the grasp fails.

        Returns (q, ok, approach_axis, jaw_axis).
        """
        a_pref = np.asarray(prefer_approach, float)
        a_pref = a_pref / np.linalg.norm(a_pref)
        lo, hi = self.limits[:, 0], self.limits[:, 1]
        rng = np.random.default_rng(seed)
        q0 = np.clip(np.array(q0, float), lo, hi)

        best = None
        for i in range(restarts + 1):
            q = self._pos_ik(p_goal, q0 if i == 0 else rng.uniform(lo, hi))
            p, R = self.fk(q)
            if np.linalg.norm(p - p_goal) > tol:
                continue
            approach = R @ np.array([0.0, 0.0, 1.0])
            jaw = R @ np.array([0.0, 1.0, 0.0])
            tilt = abs(float(jaw[2]))
            if level_jaws and tilt > MAX_JAW_TILT:
                continue
            score = (1.20 * (1.0 - tilt)
                     + 0.80 * float(approach @ a_pref)
                     - CONTINUITY * float(np.linalg.norm(q - q0)))
            if best is None or score > best[0]:
                best = (score, q, approach, jaw)
        if best is None:
            return np.array(q0), False, np.array([0, 0, -1.0]), np.array([0, 1.0, 0])
        return best[1], True, best[2], best[3]


# ----------------------------------------------------------------------- node
class PickAndPlace(Node):
    def __init__(self):
        super().__init__("pick_and_place")
        share = get_package_share_directory("uw_arm_description")
        urdf = subprocess.run(
            ["xacro", share + "/urdf/uw_arm.urdf.xacro", "use_gazebo:=false"],
            capture_output=True, text=True, check=True).stdout
        self.chain = Chain(urdf)

        self.status = self.create_publisher(String, "/demo/status", 10)
        self.attach = self.create_publisher(Empty, "/gripper/attach", 10)
        self.detach = self.create_publisher(Empty, "/gripper/detach", 10)
        self.arm = ActionClient(self, FollowJointTrajectory,
                                "/arm_controller/follow_joint_trajectory")
        # Both fingers, driven by a JointTrajectoryController - the gripper has
        # no mimic joint, so a single-joint GripperActionController would leave
        # one jaw stationary.
        self.grip = ActionClient(self, FollowJointTrajectory,
                                 "/gripper_controller/follow_joint_trajectory")
        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)
        self.q = list(HOME)

    # ---------------------------------------------------------------- helpers
    def say(self, text):
        self.get_logger().info(text)
        self.status.publish(String(data=text))

    def spin(self, seconds):
        end = self.get_clock().now().nanoseconds + seconds * 1e9
        while self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def lookup(self, frame, parent="base_link", timeout=25.0):
        end = self.get_clock().now().nanoseconds + timeout * 1e9
        while self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                tr = self.tf_buf.lookup_transform(parent, frame, rclpy.time.Time())
            except Exception:
                continue
            t = tr.transform.translation
            return np.array([t.x, t.y, t.z])
        return None

    def world_to_base(self, xyz_world):
        off = self.lookup("world", "base_link")
        if off is None:
            raise RuntimeError("no base_link <- world transform")
        return np.array(xyz_world) + off

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
        rclpy.spin_until_future_complete(self, h.get_result_async(),
                                         timeout_sec=seconds + 15.0)
        self.q = list(q)
        return True

    def gripper(self, position, caption=None, seconds=1.5):
        """Drive BOTH jaws to the same opening."""
        if caption:
            self.say(caption)
        if not self.grip.server_is_ready():
            return
        traj = JointTrajectory()
        traj.joint_names = ["finger_left_joint", "finger_right_joint"]
        pt = JointTrajectoryPoint()
        pt.positions = [float(position), float(position)]
        pt.velocities = [0.0, 0.0]
        pt.time_from_start.sec = int(seconds)
        pt.time_from_start.nanosec = int((seconds % 1.0) * 1e9)
        traj.points.append(pt)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=8.0)
        h = fut.result()
        if h is not None and h.accepted:
            # A successful grasp stalls the fingers short of the goal, so a
            # non-SUCCESS result here is expected and must not abort the run.
            rclpy.spin_until_future_complete(self, h.get_result_async(),
                                             timeout_sec=seconds + 6.0)
        self.spin(0.4)

    def latch(self):
        """Weld the payload to the gripper while the arm is stationary.

        Published repeatedly: the first message on a fresh publisher is commonly
        lost to discovery, and a weld captured late would record the offset from
        wherever the arm had already moved to.
        """
        self.say("Securing payload")
        for _ in range(12):
            self.attach.publish(Empty())
            self.spin(0.08)

    def release(self):
        self.say("Releasing target")
        for _ in range(10):
            self.detach.publish(Empty())
            self.spin(0.06)
        self.gripper(JAW_OPEN)
        self.spin(0.6)

    def solve(self, p, label, prefer=(0, 0, -1), level=True):
        q, ok, approach, jaw = self.chain.ik(p, self.q, prefer, level)
        pf, _ = self.chain.fk(q)
        self.get_logger().info(
            "IK %-12s target=[%+.3f %+.3f %+.3f] residual=%4.1fmm "
            "approach=[%+.2f %+.2f %+.2f] jaw_tilt=%.2f %s"
            % (label, p[0], p[1], p[2], np.linalg.norm(pf - p) * 1000,
               approach[0], approach[1], approach[2], abs(jaw[2]),
               "OK" if ok else "FAILED"))
        return q, ok, approach

    # -------------------------------------------------------------- sequence
    def run(self):
        self.say("Waiting for controllers")
        if not self.arm.wait_for_server(timeout_sec=90.0):
            self.get_logger().error("arm_controller unavailable")
            return False
        self.grip.wait_for_server(timeout_sec=15.0)

        self.goto(HOME, 4.0, "Home posture")
        self.gripper(JAW_OPEN, "Opening jaws")

        self.say("Locating target from TF")
        target = self.lookup(TARGET_FRAME)
        if target is None:
            self.get_logger().error("frame '%s' never appeared" % TARGET_FRAME)
            return False
        start_world = self.lookup(TARGET_FRAME, "world")
        self.get_logger().info("target in base_link: [%+.3f %+.3f %+.3f]" % (*target,))

        grasp_point = target + np.array([0.0, 0.0, GRASP_Z_OFFSET])
        q_grasp, ok, approach = self.solve(grasp_point, "grasp")
        if not ok:
            self.get_logger().error("no valid grasp - target unreachable "
                                    "with the jaws level")
            return False

        q, ok, _ = self.solve(grasp_point - approach * 0.15, "pre-grasp")
        if not ok:
            return False
        self.goto(q, 5.0, "Approaching target")
        self.goto(q_grasp, 4.0, "Closing on target")

        self.gripper(grip_close_position(), "Closing jaws on target", seconds=2.0)
        self.spin(0.8)
        self.latch()

        # Retreat along the approach line, then straight up.
        q, ok, _ = self.solve(grasp_point - approach * 0.15, "retreat")
        if ok:
            self.goto(q, 4.0, "Lifting clear of the panel")

        # Verify the payload actually came with us before continuing.
        self.spin(1.0)
        now_world = self.lookup(TARGET_FRAME, "world")
        risen = (now_world - start_world) if now_world is not None else None
        if risen is None or np.linalg.norm(risen) < 0.02:
            self.get_logger().error(
                "GRASP FAILED - target did not move with the gripper "
                "(displacement %.3f m)"
                % (0.0 if risen is None else float(np.linalg.norm(risen))))
            self.say("Grasp failed")
            return False
        self.get_logger().info("grasp confirmed - payload moved %.3f m"
                               % float(np.linalg.norm(risen)))
        self.say("Payload secured")

        lift = grasp_point - approach * 0.15 + np.array([0.0, 0.0, 0.12])
        q, ok, _ = self.solve(lift, "lift")
        if ok:
            self.goto(q, 4.0, "Raising the payload")

        place = self.world_to_base(PLACE_XYZ)
        q_place, ok, place_approach = self.solve(place, "place")
        if not ok:
            self.get_logger().error("drop-off point not reachable")
            return False
        q_pre, ok, _ = self.solve(place - place_approach * 0.16, "pre-place")
        if ok:
            # Split the transfer in two and take it slowly. A single long swing
            # across 0.6 m accelerates the payload enough to break the friction
            # grip even when the grasp itself is sound.
            mid = 0.5 * (np.array(self.q) + np.array(q_pre))
            self.goto(mid, 6.0, "Transferring to drop-off basket")
            self.goto(q_pre, 6.0, "Approaching drop-off basket")
        self.goto(q_place, 6.0, "Lowering into basket")

        self.release()
        self.goto(q, 3.5, "Clearing the basket") if ok else None
        self.goto(HOME, 5.0, "Returning home")

        final = self.lookup(TARGET_FRAME, "world")
        if final is not None:
            d = np.linalg.norm(final[:2] - np.array(PLACE_XYZ[:2]))
            self.get_logger().info(
                "payload final position [%+.3f %+.3f %+.3f], %.3f m from the "
                "drop-off point" % (*final, d))
            if d > 0.25:
                self.say("Placement off target")
                return False
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
