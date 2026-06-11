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

# px4 + gz + ros_gz bridge processes whose environment has our partition.
# The bridges (parameter_bridge/image_bridge, incl. the /clock bridge) inherit
# GZ_PARTITION from the launch env; orphaned ones survive a crashed run as
# zombies that poison the graph (/clock ends up with 0 live publishers and
# every use_sim_time node freezes), so they MUST be swept here too.
for pid in $(pgrep -f "bin/px4|gz sim|parameter_bridge|image_bridge" 2>/dev/null); do
  if tr '\0' '\n' < "/proc/$pid/environ" 2>/dev/null | grep -qx "GZ_PARTITION=${PARTITION}"; then
    kill -9 "$pid" 2>/dev/null && echo "  killed pid $pid"
  fi
done

# prior launches first (so they can't respawn children mid-cleanup)
pkill -9 -f "ros2 launch drone_interception_sim" 2>/dev/null
pkill -9 -f "ros2 launch d2dtracker_states" 2>/dev/null

# all of THIS sim's ROS nodes (mavros, gz bridge, static TFs, tf_relay,
# markers, target_trajectory, metrics). These don't carry GZ_PARTITION, so they
# must be matched by cmdline or they pile up across restarts and publish
# duplicate/conflicting /clock and TF.
for pat in \
  "__ns:=/interceptor/mavros" "__ns:=/target/mavros" \
  "gz_ns:=interceptor" "gz_ns:=target" \
  "map2global_tf_node" "map2map_frd_tf_node" \
  "map2px4_interceptor_tf_node" "map2px4_target_tf_node" \
  "odom2base_tf_relay" "drone_interception_sim/lib" \
  "interceptor_depthcam_bridge"; do
  pkill -9 -f "$pat" 2>/dev/null
done

sleep 1
echo "Done. (run before relaunching if a previous run didn't exit cleanly)"
