#!/usr/bin/env bash
set -e

export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

export PYTHONPATH="/home/rokey/isaacsim/exts/isaacsim.ros2.bridge/jazzy/rclpy"

export LD_LIBRARY_PATH="/home/rokey/isaacsim/exts/isaacsim.ros2.bridge/jazzy/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

exec /home/rokey/isaacsim/python.sh \
  /home/rokey/netclean_project/sim/standalone/netclean_standalone.py \
  --config /home/rokey/netclean_project/sim/config/sim_config.yaml \
  "$@"
