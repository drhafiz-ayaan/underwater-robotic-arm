# Phase 1 Deliverable — Underwater 6-DOF Manipulator

**Ifra Aerial Robotics** · ROS 2 Jazzy · Gazebo Harmonic (gz-sim 8.11)

## Contents

| Path | What it is |
|---|---|
| `PHASE1_REPORT.md` | Technical report — findings, decisions, verification |
| `stills/` | Renders of the arm in the underwater scene |
| `video/` | Demonstration video (see `RECORD_VIDEO.md` — **not yet recorded**) |
| `RECORD_VIDEO.md` | How to record the video, and why it is not automated yet |
| `../panda_ws/` | The ROS 2 workspace (two packages) |

## Status at a glance

| Phase 1 requirement | State |
|---|---|
| 1. Arm in ROS 2 + Gazebo with `joint_state_broadcaster` + `joint_trajectory_controller` | **Working** — all three controllers activate, trajectories execute |
| 2. Camera integrated into URDF and Gazebo | **Working** — wrist RGB-D publishes image, depth, points, camera_info |
| 3. Underwater world: buoyancy, drag, seabed, lighting, floating/sinking objects | **Working and measured** |
| 4. Launch files spawning world + robot together | **Working** |
| Demonstration video | **Blocked** on a Gazebo offscreen-rendering fault — record via GUI, see `RECORD_VIDEO.md` |

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
