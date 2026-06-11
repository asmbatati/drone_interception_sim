#!/usr/bin/env bash
#
# Launch the interception sim under a reliable RMW.
#
# Why this wrapper exists: the zenoh RMW on this setup delivers only low-rate
# topics (e.g. mavros/state) but DROPS high-rate ones (mavros odom @30 Hz,
# /clock @200 Hz), which breaks sim-time and the detection->KF->... perception
# chain. FastRTPS (Fast DDS, the ROS 2 default) delivers them reliably.
#
# RMW_IMPLEMENTATION must be set in the SHELL *before* `ros2 launch`: ros2
# launch's Node actions (mavros, the gz bridge) inherit the launch process's
# startup environment, which an os.environ change or SetEnvironmentVariable
# action inside the launch file cannot override. Exporting it here does.
#
# Usage:
#   ./run_sim.sh [ros2-launch-args...]          # FastRTPS (default)
#   RMW=rmw_zenoh_cpp ./run_sim.sh ...          # override the RMW
#   ./run_sim.sh gpu:=false headless:=1
#
# Every OTHER terminal that talks to this sim must export the SAME
# RMW_IMPLEMENTATION (and ROS_DOMAIN_ID).
set -eu

export RMW_IMPLEMENTATION="${RMW:-rmw_fastrtps_cpp}"
echo "[run_sim] RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
echo "[run_sim] reminder: export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION} in every terminal talking to the sim"

exec ros2 launch drone_interception_sim interception.launch.py "$@"
