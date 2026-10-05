#!/usr/bin/env bash
# 카메라 + 인지 노드 실행. 인자는 launch 인자로 그대로 전달
#   ./scripts/run_perception.sh
#   ./scripts/run_perception.sh camera:=false output_topic:=/target
set -eo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
source /opt/ros/${ROS_DISTRO:-lyrical}/setup.bash
source "$HERE/../ros2_ws/install/setup.bash"
exec ros2 launch "$HERE/../launch/perception.launch.py" "$@"
