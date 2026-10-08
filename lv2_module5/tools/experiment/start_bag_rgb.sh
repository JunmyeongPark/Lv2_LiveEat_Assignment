#!/bin/bash
# 영상까지 함께 기록 (평가 8 판정 프레임 · 평가 9 성공/소실 bag · 입력 재처리용). 사용: bash ~/start_bag_rgb.sh B0-01
# 컬러 + 정렬 depth 를 둘 다 넣는다: perception 은 둘을 짝지어 검출하므로 depth 가 없으면 bag 으로 입력 재처리를 못 한다.
# 용량: 컬러 640x360 rgb8 ≈ 0.69 MB/프레임 + depth 640x360 16UC1 ≈ 0.46 MB/프레임, 30 fps → 약 35 MB/s.
#        30 초 ≈ 1 GB. 10~30 초만 기록하고 끝나면 Ctrl+C.
RUN_ID=$1
[ -z "$RUN_ID" ] && { echo "사용법: bash ~/start_bag_rgb.sh B0-01"; exit 1; }
OUT=~/experiment_logs/$RUN_ID
[ -e "$OUT/bag" ] && { echo "$OUT/bag 이미 있음. 지우지 말고 다른 이름을 쓰세요"; exit 1; }
mkdir -p "$OUT"
source /opt/ros/lyrical/setup.bash
cp ~/code_fingerprint.txt "$OUT/" || { echo "먼저 ~/code_fingerprint.txt 를 만드세요"; exit 1; }
echo "남은 디스크 (30 초 ≈ 1 GB 필요):"; df -h ~ | tail -1
{ echo "run_id=$RUN_ID"; echo "start_epoch=$(date +%s.%N)"
  echo "git_commit=로봇에 git 없음, code_fingerprint.txt 의 sha256 참고"
  echo "bag_topics=camera color + aligned_depth image_raw, detection, tracking_status, planning/*, control/*, perception/camera_health, inject/*"; } > "$OUT/meta.txt"
exec ros2 bag record \
  -e '^/(camera/camera/color/image_raw|camera/camera/aligned_depth_to_color/image_raw|detection|tracking_status|planning/.*|control/.*|perception/camera_health|inject/.*)$' \
  -o "$OUT/bag"
