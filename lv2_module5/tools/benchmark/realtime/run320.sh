#!/usr/bin/env bash
# 통제 측정: 같은 장면(퍽 고정)에서 NCNN 320x256(v1)과 기준 640x480(v1)을 연속 측정

source /opt/ros/lyrical/setup.bash
cd ~/Lv2_LiveEat_Assignment/lv2_module5/ros2_ws && source install/setup.bash
OUT=~/controlled_320; mkdir -p $OUT
BIN=install/perception/lib/perception/perception_master
RUN_S=75; COOL_S=30
# 1초 간격 온도·클럭 기록 (전 구간)
( while true; do echo "$(date +%s.%N),$(cat /sys/class/thermal/thermal_zone0/temp),$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq)"; sleep 1; done ) > $OUT/thermal.csv &
TPID=$!
run() { # name backend h modeldir
  local name=$1 be=$2 h=$3 d=$4
  local w=$( [ $h = 256 ] && echo 320 || echo 640 )
  echo "$(date +%s.%N),start,$name" >> $OUT/events.csv
  $BIN --ros-args --params-file ../config/perception.yaml -p backend:=$be \
    -p model_param:=$HOME/models/$d/model.ncnn.param -p model_bin:=$HOME/models/$d/model.ncnn.bin \
    -p model_onnx:=$HOME/models/$d/model.onnx -p input_width:=$w -p input_height:=$h \
    -p timing_csv:=$OUT/$name.csv > $OUT/$name.log 2>&1 &
  local pid=$!
  sleep $((RUN_S-2))
  ps -p $pid -o %cpu=,rss= | awk -v n=$name '{print n","$1","$2/1024}' >> $OUT/proc.csv
  kill -INT $pid; wait $pid
  echo "$(date +%s.%N),end,$name" >> $OUT/events.csv
}
echo "name,cpu_pct,rss_mb" > $OUT/proc.csv
run v1_320x256 ncnn 256 target_blue_256;     sleep $COOL_S
run v1_640x480 ncnn 480 target_blue_480
kill $TPID
echo DONE > $OUT/DONE
