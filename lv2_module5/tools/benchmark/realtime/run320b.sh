#!/usr/bin/env bash
# 통제 측정: 같은 장면(퍽 고정)에서 320x256 NCNN/ONNX, 스레드 3/4, 기준 640x480 연속 측정
source /opt/ros/lyrical/setup.bash
cd ~/Lv2_LiveEat_Assignment/lv2_module5/ros2_ws && source install/setup.bash
OUT=~/controlled_320b; mkdir -p $OUT
BIN=install/perception/lib/perception/perception_master
RUN_S=75; COOL_S=30
( while true; do echo "$(date +%s.%N),$(cat /sys/class/thermal/thermal_zone0/temp),$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq)"; sleep 1; done ) > $OUT/thermal.csv &
TPID=$!
run() { # name backend width height threads modeldir
  local name=$1 be=$2 w=$3 h=$4 th=$5 d=$6
  echo "$(date +%s.%N),start,$name" >> $OUT/events.csv
  $BIN --ros-args --params-file ../config/perception.yaml -p backend:=$be \
    -p model_param:=$HOME/models/$d/model.ncnn.param -p model_bin:=$HOME/models/$d/model.ncnn.bin \
    -p model_onnx:=$HOME/models/$d/model.onnx -p input_width:=$w -p input_height:=$h -p num_threads:=$th \
    -p timing_csv:=$OUT/$name.csv > $OUT/$name.log 2>&1 &
  local pid=$!
  sleep $((RUN_S-2))
  ps -p $pid -o %cpu=,rss= | awk -v n=$name '{print n","$1","$2/1024}' >> $OUT/proc.csv
  kill -INT $pid; wait $pid
  echo "$(date +%s.%N),end,$name" >> $OUT/events.csv
}
echo "name,cpu_pct,rss_mb" > $OUT/proc.csv
run ncnn_320x256_t4 ncnn 320 256 4 target_blue_256; sleep $COOL_S
run onnx_320x256_t4 onnx 320 256 4 target_blue_256; sleep $COOL_S
run ncnn_320x256_t3 ncnn 320 256 3 target_blue_256; sleep $COOL_S
run ncnn_640x480_t4 ncnn 640 480 4 target_blue_480
kill $TPID
echo DONE > $OUT/DONE
