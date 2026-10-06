#!/usr/bin/env bash
# 통합 환경 인지 FPS 측정: 카메라(+판단·제어)를 띄워 둔 상태에서 perception_master를 스레드 수별로 연속 측정
#
#   ./integrated_fps.sh <결과 이름> ["4 3 2"]
#   예) ./integrated_fps.sh cam_only          # 카메라만 띄운 상태
#       ./integrated_fps.sh full_stack        # 카메라 + planning + control 띄운 상태
#       PIN=1 ./integrated_fps.sh pinned "3 2" # 인지를 코어 0..(스레드-1)에 고정 (카메라 등은 직접 taskset -c 3 으로 실행)
#
# 주의: bringup.launch.py는 perception_master도 같이 띄우므로 쓰지 않는다 (같은 노드 2개가 CPU를 나눠 씀).
# 결과: ~/fps_<이름>/ (프레임별 CSV, 노드 로그, 1초 간격 온도·클럭, 실행별 CPU, 이미지 구독자, top, summary.txt)
set -uo pipefail
NAME=${1:?결과 이름 (예: cam_only, full_stack)}
THREADS=${2:-"4 3 2"}
RUN_S=${RUN_S:-60}; COOL_S=${COOL_S:-20}; WARMUP_S=${WARMUP_S:-15}; PIN=${PIN:-0}

HERE=$(cd "$(dirname "$0")" && pwd)
MOD=$(cd "$HERE/../../.." && pwd)               # lv2_module5
set +u  # ROS setup.bash는 정의 안 된 변수를 참조함 (set -u와 충돌)
source /opt/ros/${ROS_DISTRO:-lyrical}/setup.bash
source "$MOD/ros2_ws/install/setup.bash"
set -u
MODEL=$(ros2 pkg prefix perception)/share/perception/models/target_blue_256
BIN=$(ros2 pkg prefix perception)/lib/perception/perception_master
OUT=~/fps_$NAME; mkdir -p "$OUT"

COLOR=/camera/camera/color/image_raw
DEPTH=/camera/camera/aligned_depth_to_color/image_raw
ros2 topic list | grep -q "^$DEPTH$" || { echo "카메라 토픽 없음: $DEPTH (realsense를 align_depth.enable:=true로 먼저 실행)"; exit 1; }
if ros2 node list | grep -q perception_master; then
  echo "perception_master가 이미 실행 중 → 끄고 다시 실행 (bringup 대신 카메라·planning·control만 따로 띄우기)"; exit 1
fi

{
  echo "date: $(date -Iseconds)  host: $(hostname)  git: $(git -C "$MOD" rev-parse --short HEAD)"
  echo "nodes: $(ros2 node list | tr '\n' ' ')"
  echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}  RUN_S=$RUN_S WARMUP_S=$WARMUP_S threads=$THREADS PIN=$PIN"
  for p in $(pgrep -f "realsense2_camera_node|lib/planning/planning_master|lib/control/control_master"); do
    echo "affinity: $(taskset -cp $p 2>/dev/null) $(ps -o comm= -p $p)"; done
  vcgencmd get_throttled 2>/dev/null || echo "get_throttled: 권한 없음 (sudo usermod -aG video \$USER)"
} > "$OUT/env.txt"
cat "$OUT/env.txt"

# 1초 간격 온도·CPU0 클럭 (전 구간)
( while true; do echo "$(date +%s.%N),$(cat /sys/class/thermal/thermal_zone0/temp),$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq)"; sleep 1; done ) > "$OUT/thermal.csv" &
TPID=$!
trap 'kill $TPID 2>/dev/null' EXIT

echo "name,cpu_pct,rss_mb" > "$OUT/proc.csv"
for th in $THREADS; do
  name=t$th
  echo "== $name (${RUN_S}s)"
  echo "$(date +%s.%N),start,$name" >> "$OUT/events.csv"
  PREFIX=(); [[ $PIN == 1 ]] && PREFIX=(taskset -c 0-$((th - 1)))
  "${PREFIX[@]}" "$BIN" --ros-args --params-file "$MOD/config/perception.yaml" \
    -p model_param:=$MODEL/model.ncnn.param -p model_bin:=$MODEL/model.ncnn.bin \
    -p num_threads:=$th -p timing_csv:=$OUT/$name.csv > "$OUT/$name.log" 2>&1 &
  pid=$!
  sleep $((RUN_S / 2))
  # 실행 중간: 이미지 구독자(PC rviz 등 외부 구독 확인), 프로세스별 CPU
  { ros2 topic info -v $COLOR; ros2 topic info -v $DEPTH; } > "$OUT/${name}_image_subs.txt" 2>&1
  top -b -n 1 -o %CPU | head -20 > "$OUT/${name}_top.txt"
  taskset -cp $pid >> "$OUT/${name}_top.txt"
  sleep $((RUN_S - RUN_S / 2 - 2))
  ps -p $pid -o %cpu=,rss= | awk -v n=$name '{print n","$1","$2/1024}' >> "$OUT/proc.csv"
  kill -INT $pid; wait $pid
  echo "$(date +%s.%N),end,$name" >> "$OUT/events.csv"
  [[ $th != ${THREADS##* } ]] && sleep $COOL_S
done
kill $TPID

# 요약: 워밍업 제외, 발행 FPS = (프레임 수 − 1) / 첫·마지막 stamp 간격
{
  printf "%-5s %7s %10s %10s %13s %8s %8s %12s %10s\n" run frames infer_mean infer_p95 callback_mean fps cpu clk_min_MHz temp_max
  for th in $THREADS; do
    f=$OUT/t$th.csv
    s=$(awk -F, -v n=t$th '$2=="start"&&$3==n{print $1}' "$OUT/events.csv")
    e=$(awk -F, -v n=t$th '$2=="end"&&$3==n{print $1}' "$OUT/events.csv")
    # mawk(우분투 기본 awk)에는 asort가 없어서 p95는 sort로 계산
    read -r frames im cm fps < <(awk -F, -v w=$WARMUP_S 'NR==2{t0=$1} NR>1 && $1>=t0+w*1e9 {n++; si+=$4; sc+=$7; if(!a)a=$1; b=$1}
      END{printf "%d %.1f %.1f %.2f\n", n, si/n, sc/n, (n-1)/((b-a)/1e9)}' "$f")
    ip=$(awk -F, -v w=$WARMUP_S 'NR==2{t0=$1} NR>1 && $1>=t0+w*1e9 {print $4}' "$f" | sort -g |
      awk '{v[NR]=$1} END{printf "%.1f", v[int(NR*0.95)]}')
    read -r cmin tmax < <(awk -F, -v s=$s -v e=$e -v w=$WARMUP_S '$1>=s+w && $1<=e {c=$3/1000; t=$2/1000; if(!m||c<m)m=c; if(t>x)x=t} END{printf "%d %.1f\n", m, x}' "$OUT/thermal.csv")
    cpu=$(awk -F, -v n=t$th '$1==n{print $2"%"}' "$OUT/proc.csv")
    printf "%-5s %7s %10s %10s %13s %8s %8s %12s %10s\n" t$th "$frames" "$im" "$ip" "$cm" "$fps" "$cpu" "$cmin" "$tmax"
  done
  echo
  echo "이미지 구독자 수 (1이 정상 = perception_master만):"
  grep -H "Subscription count" "$OUT"/t*_image_subs.txt
} | tee "$OUT/summary.txt"
echo "결과: $OUT"
