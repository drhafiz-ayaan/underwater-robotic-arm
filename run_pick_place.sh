#!/usr/bin/env bash
# Run one perception/IK-driven pick-and-place against a simulation that is
# already running (start it with ./run_sim.sh in another terminal).
set -e
WS="/home/ak/ROBOTIC ARM/panda_ws"
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
exec ros2 run uw_arm_bringup pick_and_place.py
