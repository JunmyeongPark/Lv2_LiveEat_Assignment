#!/usr/bin/env bash
# 카메라 + 인지 노드 실행. 인자는 launch 인자로 그대로 전달
#   ./scripts/run_perception.sh
#   ./scripts/run_perception.sh camera:=false output_topic:=/target
#   ./scripts/run_perception.sh model:=v3       # 모델 선택 (v4 기본 | v1 | v2 | v3)
set -eo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
source /opt/ros/${ROS_DISTRO:-lyrical}/setup.bash
source "$HERE/../ros2_ws/install/setup.bash"
exec ros2 launch "$HERE/../launch/perception.launch.py" "$@"
