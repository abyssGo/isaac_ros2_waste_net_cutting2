#!/usr/bin/env bash
set -euo pipefail

: "${ISAAC_SIM_ROOT:?ISAAC_SIM_ROOT를 Isaac Sim 설치 경로로 지정하세요.}"

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
BRIDGE_ROOT="${ISAAC_SIM_ROOT}/exts/isaacsim.ros2.bridge/${ROS_DISTRO}"

export ROS_DISTRO
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export PYTHONPATH="${BRIDGE_ROOT}/rclpy${PYTHONPATH:+:${PYTHONPATH}}"
export LD_LIBRARY_PATH="${BRIDGE_ROOT}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

if [[ ! -x "${ISAAC_SIM_ROOT}/python.sh" ]]; then
  echo "Isaac Sim python.sh를 찾을 수 없습니다: ${ISAAC_SIM_ROOT}/python.sh" >&2
  exit 1
fi

exec "${ISAAC_SIM_ROOT}/python.sh" \
  "${PROJECT_ROOT}/sim/standalone/netclean_standalone.py" \
  "$@"
