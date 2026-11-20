#!/usr/bin/env bash
set -euo pipefail
MODE="${1:-mission}"
case "$MODE" in map|plan|move|mission|explore) ;; *) echo "Choose: map, plan, move, mission, explore"; exit 2;; esac
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -f /opt/ros/humble/setup.bash ]]; then echo "ROS 2 Humble not found at /opt/ros/humble"; exit 1; fi
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=88
export ROS_LOCALHOST_ONLY=1
cd "$HERE/demo_source/student_ws/src/week8_student_lab"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID | Running student mode=$MODE | Ctrl+C to stop"
exec python3 -m week8_student_lab.ros_demo --ros-args -p "mode:=$MODE" -p 'step_period:=0.45'
