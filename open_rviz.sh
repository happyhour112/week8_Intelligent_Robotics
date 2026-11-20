#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=88
export ROS_LOCALHOST_ONLY=1
exec rviz2
