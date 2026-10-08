#!/bin/bash
# JPEG 영상(5 fps, jpeg_tap.py) + 상태 · 검출 · 명령 · control 토픽 기록 (M-BAG · M-BAGL · 검출률 판정용).
# 원본 영상 대신 /camera/color/jpeg 를 담아 기록 부하가 작다. 먼저 tools/inject/jpeg_tap.py 를 켜 둘 것.
# 사용: bash ~/start_bag_jpeg.sh BAG-OK
RUN_ID=$1
[ -z "$RUN_ID" ] && { echo "사용법: bash ~/start_bag_jpeg.sh BAG-OK"; exit 1; }
OUT=~/experiment_logs/$RUN_ID
[ -e "$OUT/bag" ] && { echo "$OUT/bag 이미 있음. 지우지 말고 다음 번호를 쓰세요"; exit 1; }
mkdir -p "$OUT"
source /opt/ros/lyrical/setup.bash
cp ~/code_fingerprint.txt "$OUT/" || { echo "먼저 ~/code_fingerprint.txt 를 만드세요"; exit 1; }
{ echo "run_id=$RUN_ID"; echo "start_epoch=$(date +%s.%N)"
  echo "git_commit=로봇에 git 없음, code_fingerprint.txt 의 sha256 참고"
  echo "bag_topics=camera/color/jpeg(5fps), detection, tracking_status, planning/*, control/*, perception/camera_health, inject/*"; } > "$OUT/meta.txt"
exec ros2 bag record \
  -e '^/(camera/color/jpeg|detection|tracking_status|planning/.*|control/.*|perception/camera_health|inject/.*)$' \
  -o "$OUT/bag"
