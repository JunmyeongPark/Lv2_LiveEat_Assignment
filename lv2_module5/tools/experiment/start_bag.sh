#!/bin/bash
# 일반 시험용 bag 기록 (영상 없음). 사용: bash ~/start_bag.sh R1-01
# 이 로봇의 `ros2 bag record`는 토픽을 나열하는 방식을 받지 못해(unrecognized arguments) -e 정규식으로 지정한다.
RUN_ID=$1
[ -z "$RUN_ID" ] && { echo "사용법: bash ~/start_bag.sh R1-01"; exit 1; }
OUT=~/experiment_logs/$RUN_ID
[ -e "$OUT/bag" ] && { echo "$OUT/bag 이미 있음. 지우지 말고 다음 번호를 쓰세요"; exit 1; }
mkdir -p "$OUT"
source /opt/ros/lyrical/setup.bash
cp ~/code_fingerprint.txt "$OUT/" || { echo "먼저 ~/code_fingerprint.txt 를 만드세요"; exit 1; }
{ echo "run_id=$RUN_ID"; echo "start_epoch=$(date +%s.%N)"
  echo "git_commit=로봇에 git 없음, code_fingerprint.txt 의 sha256 참고"
  echo "bag_topics=detection,tracking_status,planning/cmd_vel,planning/arm_command,control/joint_states,control/imu,control/odom_yaw_deg"; } > "$OUT/meta.txt"
exec ros2 bag record \
  -e '^/(detection|tracking_status|planning/cmd_vel|planning/arm_command|control/joint_states|control/imu|control/odom_yaw_deg)$' \
  -o "$OUT/bag"
