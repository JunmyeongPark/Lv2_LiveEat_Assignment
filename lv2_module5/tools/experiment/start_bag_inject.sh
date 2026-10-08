#!/bin/bash
# 장애 주입(fault_injector) 시연용 bag 기록. 사용: bash ~/start_bag_inject.sh INJ-01
# -e 정규식으로 토픽을 지정한다 (이 로봇의 ros2 bag record 가 토픽 나열을 받지 못함). 주입 이벤트·진단·주입 전후 토픽 포함.
#   /inject/event      키를 누른 시각 (주입 시각)
#   /tracking_status   planning 상태 전이 시각 (반응 시각)
#   /detection ↔ /inject/detection, /inject/planning_cmd_vel ↔ /planning/cmd_vel  주입 전후 비교
RUN_ID=$1
[ -z "$RUN_ID" ] && { echo "사용법: bash ~/start_bag_inject.sh INJ-01"; exit 1; }
OUT=~/experiment_logs/$RUN_ID
[ -e "$OUT/bag" ] && { echo "$OUT/bag 이미 있음. 지우지 말고 다음 번호를 쓰세요"; exit 1; }
mkdir -p "$OUT"
source /opt/ros/lyrical/setup.bash
cp ~/code_fingerprint.txt "$OUT/" || { echo "먼저 ~/code_fingerprint.txt 를 만드세요"; exit 1; }
{ echo "run_id=$RUN_ID"; echo "start_epoch=$(date +%s.%N)"
  echo "git_commit=로봇에 git 없음, code_fingerprint.txt 의 sha256 참고"
  echo "bag_topics=detection,tracking_status,planning/*,control/*,perception/camera_health,inject/*"; } > "$OUT/meta.txt"
exec ros2 bag record \
  -e '^/(detection|tracking_status|planning/.*|control/.*|perception/camera_health|inject/.*)$' \
  -o "$OUT/bag"
