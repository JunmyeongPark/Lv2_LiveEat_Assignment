#!/usr/bin/env bash
# 와플파이 배포 빌드: perception(NCNN만) + planning + control
#   ./scripts/build_pi.sh            # 전체
#   ./scripts/build_pi.sh perception # 일부 패키지만
# ncnn 설치 위치가 다르면 NCNN_DIR=<prefix>/lib/cmake/ncnn 지정 (설치: tools/benchmark/README.md 3절)
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
NCNN_DIR=${NCNN_DIR:-$HOME/ncnn-install/lib/cmake/ncnn}
PKGS=${*:-perception planning control}

source /opt/ros/${ROS_DISTRO:-lyrical}/setup.bash
cd "$HERE/../ros2_ws"
colcon build --packages-select $PKGS --cmake-args -Dncnn_DIR="$NCNN_DIR"
