# Recording the demonstration video

**Read this first — the automated headless recorder does not currently produce a
clean video on this machine, and the reason matters.**

## What works and what does not

| Path | Status |
|---|---|
| Simulation, physics, buoyancy, controllers, arm motion | Working and verified |
| Gazebo **GUI** rendering (GLX) | Working — renders the arm correctly |
| Gazebo **headless offscreen sensor** rendering (EGL) | Unreliable — see below |

`record_demo.launch.py` records from camera sensors inside a headless server.
On this machine that offscreen render path intermittently omits the spawned
robot's visuals: sampling 360 frames of a recording gives several distinct
renders of the *same static scene*, some containing the arm and most not.
Neither pinning the NVIDIA EGL vendor, reducing to one shadow-casting light,
nor splitting the ROS image bridges fixed it. The physics and control stack are
unaffected — this is purely how the scene is rasterised offscreen.

## Recommended: record from the Gazebo GUI

The GUI renders through GLX in its own process, which is a different code path
and renders correctly.

**1.** Start the simulation with the GUI (drop the `-s`):

```bash
cd "/home/ak/ROBOTIC ARM/panda_ws" && source install/setup.bash && ros2 launch uw_arm_bringup underwater_sim.launch.py gz_args:="-r -v 3" rviz:=false
```

**2.** Wait until the terminal prints `Configured and activated gripper_controller`
(about 15 s). Frame the shot by dragging in the 3-D view.

**3.** In the Gazebo window, open the ⋮ menu (top-right) → **Video Recorder** →
choose **mp4** → press record.

**4.** In a second terminal, run the scripted motion sequence:

```bash
cd "/home/ak/ROBOTIC ARM/panda_ws" && source install/setup.bash && ros2 run uw_arm_bringup demo_sequence.py
```

It runs about 50 s and prints each caption as it goes. Stop the recorder when it
prints `Sequence complete`. Gazebo writes the file to a location it shows in the
dialog.

## Alternative: screen capture

`ffmpeg` is not installed. If you would rather capture the whole window:

```bash
sudo apt install -y ffmpeg && ffmpeg -f x11grab -framerate 30 -i :1 -c:v libx264 -preset fast -crf 23 "/home/ak/ROBOTIC ARM/deliverable/video/uw_arm_phase1_demo.mp4"
```

Run the demo sequence as in step 4 above, then press `q` in the ffmpeg terminal.

## If you want to retry the automated recorder

Everything is wired and will work the moment the offscreen renderer behaves:

```bash
cd "/home/ak/ROBOTIC ARM/panda_ws" && source install/setup.bash && ros2 launch uw_arm_bringup record_demo.launch.py output:="/home/ak/ROBOTIC ARM/deliverable/video/uw_arm_phase1_demo.mp4" duration:=72.0
```

Check the result before sending it — sample the frame brightness and look for
jumps, which is the signature of the dropped-geometry frames:

```bash
python3 -c "import cv2,numpy as np;c=cv2.VideoCapture('/home/ak/ROBOTIC ARM/deliverable/video/uw_arm_phase1_demo.mp4');m=[];\
[m.append(f[80:600].mean()) for _ in range(int(c.get(7))) for ok,f in [c.read()] if ok];m=np.array(m);\
print('std',round(m.std(),2),'jumps',int((abs(np.diff(m))>12).sum()))"
```

A clean recording has `std` around 1 and `jumps` at 0.

## Worth trying if you have time

The most likely remaining cause is the offscreen render scene not picking up
entities inserted after it initialises. Two things to test:

- Put the arm in the world file directly instead of spawning it with
  `ros_gz_sim create`, so it exists before the render scene is built.
- Run the server with `--headless-rendering`, which forces a different EGL
  surface setup.
