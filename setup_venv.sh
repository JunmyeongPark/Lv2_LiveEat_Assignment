#!/usr/bin/env bash
# 레포 루트에 .venv 생성
#   ./setup_venv.sh          → 분석·벤치마크 도구 (라즈베리파이·PC)
#   ./setup_venv.sh export   → + YOLO 학습·변환 도구 (PC)
#
# --system-site-packages: apt로 설치된 ROS 파이썬 패키지(rclpy 등)를 venv 안에서도 사용하기 위함
set -euo pipefail
cd "$(dirname "$0")"

REQ=requirements.txt
[[ "${1:-}" == "export" ]] && REQ=requirements-export.txt

if [[ ! -d .venv ]]; then
  python3 -m venv --system-site-packages .venv
fi
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -r "$REQ"

echo
echo "완료: source .venv/bin/activate"
echo "ROS와 함께 쓸 때 순서: source /opt/ros/\$ROS_DISTRO/setup.bash → source .venv/bin/activate → source install/setup.bash"
