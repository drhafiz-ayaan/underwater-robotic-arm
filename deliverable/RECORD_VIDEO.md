# Recording the demonstration video

The recorder is automated and works. `video/uw_arm_phase1_demo.mp4` was produced
by it: 56 s, 1280x720, H.264, 1.2 MB.

```bash
cd "/home/ak/ROBOTIC ARM/panda_ws" && source install/setup.bash && ros2 launch uw_arm_bringup record_demo.launch.py output:="/home/ak/ROBOTIC ARM/deliverable/video/uw_arm_phase1_demo.mp4" duration:=56.0
```

It runs the simulation headless, drives `demo_sequence.py`, and writes the file
from the in-simulation cameras — no screen capture, no ffmpeg, no window chrome.
Takes about two and a half minutes and exits on its own.

## Read this before re-recording: kill orphaned servers first

**Gazebo servers that outlive a previous run will silently corrupt the
recording.** gz-transport topics are global to the machine, so a leftover server
publishing `/scene_camera/image` interleaves its frames with the live one. The
recording then cuts between two or more different simulations several times a
second — the robot present in one frame and gone in the next, with the lighting
jumping around. Nothing in the logs reports a problem.

Check before every run:

```bash
pgrep -af "gz sim|image_bridge|parameter_bridge"
```

If anything is listed, kill it and wait for the port to clear:

```bash
pkill -9 -f "gz sim" ; pkill -9 -f image_bridge ; pkill -9 -f parameter_bridge ; sleep 3
```

## Verifying a recording

Frame-brightness variance catches the interleaving instantly. A clean recording
has `std` near 1 and `jumps` at 0; a corrupted one gives `std` above 10 and
dozens of jumps.

```bash
python3 -c "import cv2,numpy as np;c=cv2.VideoCapture('/home/ak/ROBOTIC ARM/deliverable/video/uw_arm_phase1_demo.mp4');m=[];[m.append(f[80:600].mean()) for _ in range(int(c.get(7))) for ok,f in [c.read()] if ok];m=np.array(m);print('std',round(m.std(),2),'jumps',int((abs(np.diff(m))>12).sum()))"
```

## Watching it live instead

To drive the arm while watching in the Gazebo GUI, in two terminals:

```bash
cd "/home/ak/ROBOTIC ARM/panda_ws" && source install/setup.bash && ros2 launch uw_arm_bringup underwater_sim.launch.py gz_args:="-r -v 3" rviz:=false
```

```bash
cd "/home/ak/ROBOTIC ARM/panda_ws" && source install/setup.bash && ros2 run uw_arm_bringup demo_sequence.py
```

The world now carries its own `<gui>` block, so the viewport always opens on this
world with the camera framed on the arm. Without it the GUI falls back to
`~/.gz/sim/8/gui.config`, which remembers the last world opened — a stale entry
there leaves the viewport empty while the server runs perfectly.
