# Underwater Intervention Manipulator

ROS 2 **Jazzy** + **Gazebo Harmonic** (gz-sim 8.11).

## Status

| Capability | State |
|---|---|
| 1. Arm in ROS 2 + Gazebo, `joint_state_broadcaster` + `joint_trajectory_controller` | Written, **not yet run** — needs `ros2_control` installed |
| 2. Camera integrated into URDF + Gazebo | Written, **not yet run** |
| 3. Underwater world: buoyancy, drag, seabed, lighting, floating/sinking objects | **Verified running** |
| 4. Launch files spawning world + robot together | Written, **not yet run** |

Items 1, 2 and 4 could not be executed here because `ros2_control`, `ros2_controllers`
and `gz_ros2_control` are not installed on this machine. Install them first:

```bash
sudo apt update && sudo apt install -y ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-gz-ros2-control ros-jazzy-moveit ros-jazzy-ros-gz
```

## Provenance and the kinematic fix

Rebuilt from `reference/arm water.urdf/` (SolidWorks URDF Exporter 1.6.0).

The original export assigned **one** SolidWorks reference frame
(`Coordinate System2`) to all seven links, so every joint came out with
`<origin xyz="0 0 0"/>`. All link frames collapsed onto the world origin and
every joint rotated about a line through the base — the model looked correct at
zero configuration and was meaningless in motion.

Joint **axis directions** in the export are correct. Only the axis **locations**
were lost, and a revolute joint needs just one point on its axis, so the six
points were recovered from where consecutive STL meshes interpenetrate. They sit
in a single block at the top of `urdf/uw_arm.urdf.xacro` — accurate to a few mm,
and the only thing to replace after a clean SolidWorks re-export.

`reference/` carries a `COLCON_IGNORE`: `project(arm water.urdf)` is not valid
CMake and breaks the build otherwise.

## Layout

```
src/uw_arm_description/
  urdf/uw_arm.urdf.xacro          chain + recovered axis points  <-- edit here
  urdf/gripper.xacro              parallel-jaw gripper (not in the CAD)
  urdf/camera.xacro               wrist RGB-D, eye-in-hand
  urdf/uw_arm.ros2_control.xacro  position interfaces
  urdf/uw_arm.gazebo.xacro        hydrodynamics, sensor, grasp latch
src/uw_arm_bringup/
  worlds/underwater.sdf           seabed, lights, buoyancy, cargo
  config/controllers.yaml         jsb + arm_controller + gripper_controller
  config/gz_bridge.yaml           clock, camera, grasp topics
  launch/display.launch.py        RViz only — verify kinematics here first
  launch/underwater_sim.launch.py full bringup
```

## Run

```bash
cd "/home/ak/ROBOTIC ARM/panda_ws" && colcon build --symlink-install && source install/setup.bash
```

Check the recovered kinematics before anything else — drag every slider and
confirm each link pivots about a sensible hinge rather than swinging about the
base:

```bash
ros2 launch uw_arm_bringup display.launch.py
```

Full simulation:

```bash
ros2 launch uw_arm_bringup underwater_sim.launch.py
```

## Verified behaviour

5000 iterations (5 s) headless, no warnings or errors. Object heights:

| Model | Start z | After 12 s | Expected |
|---|---|---|---|
| `target_canister` (ρ≈2000) | 0.460 | 0.460 | sinks / holds station |
| `drift_float` (ρ≈300) | 1.200 | 8.02 | rises to the 8.0 m surface |
| `neutral_pod` (ρ≈1025) | 0.800 | 0.773 | hovers |

## Two traps worth remembering

**Added mass must go in `<fluid_added_mass>`, not the Hydrodynamics plugin.**
The plugin's `xDotU`/`yDotV`/… path is integrated explicitly, so once the added
mass approaches the body mass the effective mass matrix stops being
positive-definite and the solver dies inside 50 ms with
`ODE INTERNAL ERROR 1: assertion "aabbBound >= dMinIntExact" failed`. Neutrally
buoyant bodies — everything here — sit exactly at that ratio. The SDF cargo uses
`<fluid_added_mass>`, which dartsim solves implicitly. URDF has no equivalent
element, so the arm caps plugin added mass at 30% of link mass instead.

**No infinite `<plane>` collisions.** A plane gives ODE an unbounded AABB and
triggers the same assertion. The seabed is a 12 m thin box.

Also: `graded_buoyancy` supports only `<box>` and `<sphere>` collisions — a
cylinder is silently given zero buoyancy.

## Known gaps

- Joint axis points are estimates (few mm). Re-export from SolidWorks with one
  coordinate system per joint for exact values.
- The CAD has no gripper; `gripper.xacro` is a stand-in sized to the scene
  (94 mm max opening vs a 60 mm target).
- `tool_offset = 0.21` m is estimated from the link6 mesh extent.
- Object placement on the intervention panel assumes the arm can reach
  ~0.55 m horizontally at z ≈ 0.45. Confirm once the axis points are exact.
- The workspace path contains a space (`ROBOTIC ARM`). Launch files quote paths
  to cope, but renaming to `robotic_arm` would remove a whole class of problems.
