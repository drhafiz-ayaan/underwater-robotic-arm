#!/usr/bin/env bash
# Launch the underwater simulation WITH the Gazebo GUI so you can watch it live.
# Leave this running; start the pick-and-place from a second terminal with
# ./run_pick_place.sh
set -e
WS="/home/ak/ROBOTIC ARM/panda_ws"
source /opt/ros/jazzy/setup.bash
cd "$WS"
colcon build --symlink-install >/dev/null
source install/setup.bash
# Orphaned Gazebo servers silently corrupt everything - always clear them first.
bash install/uw_arm_bringup/lib/uw_arm_bringup/kill_sim.sh || true
sleep 2
echo "Starting simulation (GUI). Wait for 'activated gripper_controller', then run ./run_pick_place.sh"
exec ros2 launch uw_arm_bringup underwater_sim.launch.py gz_args:="-r -v 3" rviz:=false
