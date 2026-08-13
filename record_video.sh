#!/usr/bin/env bash
# Headless run that records the demonstration video to deliverable/video/.
set -e
WS="/home/ak/ROBOTIC ARM/panda_ws"
OUT="${1:-/home/ak/ROBOTIC ARM/deliverable/video/uw_arm_pick_and_place.mp4}"
source /opt/ros/jazzy/setup.bash
cd "$WS"
colcon build --symlink-install >/dev/null
source install/setup.bash
bash install/uw_arm_bringup/lib/uw_arm_bringup/kill_sim.sh || true
sleep 2
exec ros2 launch uw_arm_bringup record_demo.launch.py output:="$OUT" duration:=175.0
