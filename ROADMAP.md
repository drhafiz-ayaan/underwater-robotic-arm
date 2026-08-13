# Underwater Intervention Manipulator — Roadmap

**Ayaan Aatif** · ROS 2 Jazzy · Gazebo Harmonic (gz-sim 8.11)

End goal: a manipulator mounted **beneath an underwater rover**, able to find a
free-floating object by sight, grasp it, and carry it to a collection point.

Each milestone ends with something you can run yourself and something you can
look at. No milestone is called "done" on my say-so — every one has a number that
either passes or does not.

---

## Milestone status

| # | Milestone | State |
|---|---|---|
| M1 | Neutral-buoyancy scene, rebrand, run scripts | **in progress** |
| M2 | Rover mount — arm hangs inverted beneath the vehicle | not started |
| M3 | Perception — find the payload from the wrist camera | not started |
| M4 | Closed-loop dynamic grasp — track a drifting payload | not started |

---

## M1 — Floating payload and a scene you can drive

Objects that rest on tables are not what an ROV manipulator meets. The payload is
now trimmed to **exactly seawater density** (1025 kg/m³, 0.4428 kg for 0.432 L)
so it **hovers in the water column** at z = 0.80 m, free in open water, 0.40 m
clear of any surface. The arm must grasp something that is floating, not braced.

Also in M1:
- Pedestal recoloured from amber to subsea grey. It sat ~15° of hue from the
  orange payload and was the single biggest false-positive risk for M3's colour
  segmenter — worse because it sits directly under the wrist camera.
- All "Ifra Aerial Robotics" branding replaced with **Ayaan Aatif**.
- "Phase 1 / Phase 2" naming removed throughout.
- `run_sim.sh`, `run_pick_place.sh`, `record_video.sh` at the repo root.

**Passes when:** the payload holds station in open water (no measurable drift
over 12 s) and pick-and-place still completes on the floating target.

## M2 — Rover mount, arm inverted

A rover body above the work area with the arm hanging **upside down** beneath it,
which is how a work-class ROV carries a manipulator. The rover holds station; it
does not need to swim.

**Doing this before perception, not after** — deliberately. Four of the six
joints are limited to 0–π, and inverting the mount changes the reachable set and
which grasp approaches are achievable. I already measured that a straight
top-down grasp is unsatisfiable in the current upright mount. Tuning perception
and closed-loop tracking against the upright geometry and *then* flipping would
mean redoing the grasp analysis. Better to reach final geometry first.

Work: rover hull model, mount frame flipping the arm, re-run the reachability
sweep, re-site the payload and collection basket in the new workspace, retune
grasp approach preference.

**Passes when:** the reachability sweep finds jaw-level solutions at the payload
and drop-off, and pick-and-place completes from the inverted mount.

## M3 — Perception

Replaces ground-truth object poses with an estimate from the wrist RGB-D stream.
The seam already exists: `pick_and_place.py` reads the TF frame
`target_canister`, and `object_tf_publisher.py` (ground truth) simply gets
swapped for `perception_node.py`. Ground truth stays on as the scoring yardstick.

Pipeline: HSV colour gate → depth gate (0.15–1.2 m) → largest component with
area/aspect rejection → deproject via `camera_info` → transform to `base_link` at
the image timestamp → workspace-bounds rejection → filter → broadcast TF.

Also needed: a **survey pose**. From HOME the wrist camera points at open water
and cannot see the payload at all, so the arm has to look before it can find.

**Passes when:** detection rate ≥ 95 % while in view, pose RMS error vs ground
truth < 15 mm over the approach, zero false positives on pedestal/basket/pod, and
5/5 end-to-end runs place within 0.10 m using perception TF only.

## M4 — Closed-loop dynamic grasp

A gentle current makes the payload drift, so a pose captured once at the start is
stale by the time the arm arrives. The approach re-reads the estimate and
re-solves IK as it closes in, freezing at a standoff where the gripper begins to
occlude the payload.

**Passes when:** the payload is displaced mid-approach and the arm still
completes the grasp; 5/5 runs succeed with the target drifting.

---

## Running it

```bash
cd "/home/ak/ROBOTIC ARM" && ./run_sim.sh
```

Wait for `activated gripper_controller`, then in a second terminal:

```bash
cd "/home/ak/ROBOTIC ARM" && ./run_pick_place.sh
```

To record a video instead of watching live:

```bash
cd "/home/ak/ROBOTIC ARM" && ./record_video.sh
```

**Always let `run_sim.sh` clear orphans first.** A Gazebo server left over from a
previous run keeps publishing on the same gz-transport topics and silently
corrupts both the simulation and any recording, with nothing in the logs to say
so. `run_sim.sh` handles this; if you start Gazebo by hand, run
`panda_ws/install/uw_arm_bringup/lib/uw_arm_bringup/kill_sim.sh` first.

## Decisions taken

| Decision | Choice | Why |
|---|---|---|
| Motion planning | Custom damped-least-squares IK, **not MoveIt 2** | MoveIt's KDL solver would fail often here: four joints limited to 0–π make a strict pose goal frequently unsatisfiable. The DLS solver succeeds by taking 250 random restarts and ranking solutions by achievable approach. |
| Grasp adhesion | `grasp_manager.py` weld | gz-sim 8.11 has no `gazebo_grasp_plugin` equivalent and its `DetachableJoint` ignores `<attach_topic>`. Contact friction alone ejects the payload. |
| Payload trim | Neutral (1025 kg/m³) | Matches real intervention cargo and makes the scene read as underwater. |
