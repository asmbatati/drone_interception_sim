#!/usr/bin/env bash
#
# Cleanly stop the interception sim: kill the PX4 SITL instances, the Gazebo
# server, and the ROS nodes belonging to THIS sim only (matched by
# GZ_PARTITION), so a crashed/ctrl-c'd run can't leave PX4 daemons holding the
# instance lock (which makes the next launch die with
# "PX4 server already running for instance N").
#
# Other projects' PX4/gz (different GZ_PARTITION) are left untouched.
#
#   ./stop_sim.sh [partition]      # default partition: d2d_intercept
set -u
PARTITION="${1:-d2d_intercept}"

echo "Stopping interception sim (GZ_PARTITION=${PARTITION})..."

# px4 + gz processes whose environment has our partition
for pid in $(pgrep -f "bin/px4|gz sim" 2>/dev/null); do
  if tr '\0' '\n' < "/proc/$pid/environ" 2>/dev/null | grep -qx "GZ_PARTITION=${PARTITION}"; then
    kill -9 "$pid" 2>/dev/null && echo "  killed pid $pid"
  fi
done

# interception ROS nodes (namespaced /interceptor and /target)
pkill -9 -f "mavros_node .*__ns:=/interceptor" 2>/dev/null
pkill -9 -f "mavros_node .*__ns:=/target" 2>/dev/null
pkill -9 -f "drone_interception_sim/lib" 2>/dev/null
pkill -9 -f "uav_gz_sim/lib/uav_gz_sim/tf_relay" 2>/dev/null
pkill -9 -f "ros2 launch drone_interception_sim" 2>/dev/null

sleep 1
echo "Done. (run before relaunching if a previous run didn't exit cleanly)"
