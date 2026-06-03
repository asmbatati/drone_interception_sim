#!/usr/bin/env bash
#
# Health check for the interception sim. Run AFTER launching it, in a terminal
# with the same RMW_IMPLEMENTATION as the sim. Reports FCU connection state and
# the key topics each controller needs.
#
#   ros2 launch drone_interception_sim interception.launch.py   # terminal 1
#   ./check_sim.sh                                              # terminal 2
set -u

echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<unset>}"
echo

check_connected() {
  local ns="$1"
  echo "--- /${ns}/mavros/state ---"
  timeout 5 ros2 topic echo "/${ns}/mavros/state" --once 2>/dev/null \
    | grep -E "connected|armed|mode" \
    || echo "  (no message — FCU not up yet for ${ns})"
  echo
}

echo "===== Competing Gazebo servers (should be only this sim's) ====="
pgrep -af "gz sim" | grep -v pgrep || echo "  none"
echo

echo "===== FCU connection ====="
check_connected interceptor
check_connected target

echo "===== Key topics present ====="
for t in \
  /interceptor/mavros/local_position/odom \
  /target/mavros/local_position/odom \
  /target/target_path \
  /interceptor/intercept_point \
  /interceptor/interception_path ; do
  if timeout 3 ros2 topic info "$t" >/dev/null 2>&1; then
    echo "  [ok]      $t"
  else
    echo "  [missing] $t"
  fi
done
echo
echo "Tip: if a topic is missing, the producer node (strategy/BT/guidance) may"
echo "     not be running yet — that is expected until you start a controller."
