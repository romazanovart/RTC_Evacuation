#!/usr/bin/env bash
set -euo pipefail

repository="${RTC_EVACUATION_ROOT:-$HOME/RTC_Evacuation}"
workspace="$repository/ros2/src/ros2_ws"

source /opt/ros/humble/setup.bash
source "$workspace/install/setup.bash"

exec ros2 launch rover_web web.launch.py \
  start_camera:=true \
  start_qr:=true \
  terminal_enabled:=false \
  start_terminal:=false \
  rosboard_enabled:=false
