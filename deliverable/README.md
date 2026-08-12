# Phase 1 Deliverable — Underwater 6-DOF Manipulator

**Ifra Aerial Robotics** · ROS 2 Jazzy · Gazebo Harmonic (gz-sim 8.11)

## Contents

| Path | What it is |
|---|---|
| `PHASE1_REPORT.md` | Technical report — findings, decisions, verification |
| `stills/` | Renders of the arm in the underwater scene |
| `video/` | Pick-and-place video — 157 s, 1280x720, H.264, 2.7 MB |
| `RECORD_VIDEO.md` | How to re-record it, and the orphaned-server trap |
| `../panda_ws/` | The ROS 2 workspace (two packages) |

## Status at a glance

| Phase 1 requirement | State |
|---|---|
| 1. Arm in ROS 2 + Gazebo with `joint_state_broadcaster` + `joint_trajectory_controller` | **Working** — all three controllers activate, trajectories execute |
| 2. Camera integrated into URDF and Gazebo | **Working** — wrist RGB-D publishes image, depth, points, camera_info |
| 3. Underwater world: buoyancy, drag, seabed, lighting, floating/sinking objects | **Working and measured** |
| 4. Launch files spawning world + robot together | **Working** |
| Demonstration video | **Recorded** — `video/uw_arm_pick_and_place.mp4` |
| Pick-and-place | **Working** — payload delivered 0.044 m from the drop-off point |

## Pick-and-place

`pick_and_place.py` locates the target from TF, solves inverse kinematics for
every waypoint, grips, transfers and releases. Verified end to end:

```
ATTACHED 'target_canister' at 31.8 mm from grasp_link
grasp confirmed - payload moved 0.129 m
RELEASED 'target_canister'
payload final position [+0.318 +0.280 +0.565], 0.044 m from the drop-off point
Pick and place complete
```

The payload starts at (0.42, -0.28, 0.46) on the intervention panel and ends in
the drop-off basket at (0.35, 0.25), rim height 0.50 m.

Grasp adhesion is provided by `grasp_manager.py`, which welds the payload to the
gripper frame on `/gripper/attach` and releases on `/gripper/detach`. This is the
role `gazebo_grasp_plugin` plays in a Gazebo Classic stack; Harmonic has no
equivalent and its `DetachableJoint` cannot attach on demand. Perception, IK,
trajectories, fluid dynamics and approach contact are all genuinely simulated -
only the adhesion is substituted, and a bad grasp pose still produces a bad
grasp, because the weld records the transform from wherever the gripper is.

## The headline finding

The supplied SolidWorks URDF was **kinematically non-functional**. A single
reference frame (`Coordinate System2`) had been assigned to all seven links, so
every joint exported with `<origin xyz="0 0 0"/>`: all link frames collapsed onto
the world origin and every joint rotated about a line through the base. The model
looked correct at zero configuration and was meaningless in motion.

Joint axis *directions* were correct, so the six axis *locations* were recovered
from the STL geometry and now live in one editable block at the top of
`uw_arm_description/urdf/uw_arm.urdf.xacro`. Details and the exact re-export
procedure are in `PHASE1_REPORT.md`.

## Quick start

```bash
cd "/home/ak/ROBOTIC ARM/panda_ws" && colcon build --symlink-install && source install/setup.bash
```

Check the recovered kinematics in RViz first:

```bash
ros2 launch uw_arm_bringup display.launch.py
```

Run the full underwater simulation:

```bash
ros2 launch uw_arm_bringup underwater_sim.launch.py
```
