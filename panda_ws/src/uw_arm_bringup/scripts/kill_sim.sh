#!/usr/bin/env bash
# Kill every simulation process, including orphans.
#
# RUN THIS BEFORE EVERY SIM LAUNCH.
#
# Gazebo routinely outlives the `ros2 launch` that started it - Ctrl-C, a crash,
# or a `timeout` all leave the server running. gz-transport topics are global to
# the machine, so a survivor is not harmless:
#
#   * its /scene_camera/image frames interleave with the live one, and a
#     recording cuts between two different simulations several times a second
#   * its controller_manager owns the controller names, so new spawners fail
#     with "Controller already loaded" then "Failed loading controller"
#   * the GUI can bind to its world and show an empty viewport
#   * /clock gets duplicate publishers
#
# None of these are reported as errors anywhere.
#
# Note the `pkill -f` trap: a pattern like "gz sim" matches pkill's OWN command
# line, so pkill kills its parent shell and exits 143/144 having killed nothing.
# Matching by PID avoids that.

set -u

PATTERN='gz sim|gz-sim|ros_gz|image_bridge|parameter_bridge|controller_manager|robot_state_publisher|record_demo|pick_and_place|demo_sequence'

# Every ancestor of this script must be spared. The caller's command line often
# contains one of the words above (e.g. `... && ros2 launch ... record_demo ...`),
# so a naive pgrep match kills the very shell that invoked the cleanup - the
# launch then dies instantly with no log file and no explanation.
ancestors=" $$ "
pid=$$
while [ "$pid" -gt 1 ]; do
  pid=$(ps -o ppid= -p "$pid" 2>/dev/null | tr -d ' ')
  [ -z "$pid" ] && break
  ancestors="$ancestors$pid "
done

collect() {
  for p in $(pgrep -f "$PATTERN" 2>/dev/null); do
    case "$ancestors" in *" $p "*) continue ;; esac
    printf '%s ' "$p"
  done
}

pids=$(collect)
if [ -z "${pids// /}" ]; then
  echo "nothing running"
  exit 0
fi

echo "killing: $pids"
for p in $pids; do kill -9 "$p" 2>/dev/null || true; done
sleep 3

left=$(collect)
if [ -z "${left// /}" ]; then
  echo "all clear"
else
  echo "STILL RUNNING: $left" >&2
  exit 1
fi
