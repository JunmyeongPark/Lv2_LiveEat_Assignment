#!/usr/bin/env bash
# run_benchmark.sh — 같은 입력(bag/영상/이미지 폴더)으로 NCNN, ONNX를 차례로 벤치마크
#
# 사용
#   ./run_benchmark.sh <input> <out_dir> [반복 횟수=3]
#
# 환경 변수 (모델 경로·조건)
#   NCNN_PARAM, NCNN_BIN   NCNN 모델 (model.ncnn.param / model.ncnn.bin)
#   ONNX_MODEL             ONNX 모델 (model.onnx)
#   TOPIC                  bag 이미지 토픽 (기본 /camera/camera/color/image_raw)
#   SIZE=320|640x480 THREADS=3 CONF=0.25 WARMUP=10 SAVE_EVERY=0 COOLDOWN=60
#   BACKENDS="ncnn onnx"   비교할 백엔드
#   BENCH=<detector_bench 경로>  (기본: colcon install 경로에서 찾음)
#
# 결과: <out_dir>/<backend>_r<i>.csv, .csv.meta.json, <backend>_r<i>_sys.csv, frames/
set -euo pipefail

INPUT=${1:?input (bag dir | video | image dir)}
OUT=${2:?out dir}
REPEAT=${3:-3}

TOPIC=${TOPIC:-/camera/camera/color/image_raw}
SIZE=${SIZE:-320}; THREADS=${THREADS:-3}; CONF=${CONF:-0.25}
WARMUP=${WARMUP:-10}; SAVE_EVERY=${SAVE_EVERY:-0}; COOLDOWN=${COOLDOWN:-60}
BACKENDS=${BACKENDS:-"ncnn onnx"}
HERE=$(cd "$(dirname "$0")" && pwd)

if [[ -z "${BENCH:-}" ]]; then
  PREFIX=$(ros2 pkg prefix perception 2>/dev/null || true)
  BENCH="${PREFIX}/lib/perception/detector_bench"
fi
[[ -x "$BENCH" ]] || { echo "detector_bench not found: $BENCH (source install/setup.bash 또는 BENCH= 지정)"; exit 1; }

mkdir -p "$OUT"
{
  echo "date: $(date -Iseconds)"
  echo "host: $(hostname)"
  echo "kernel: $(uname -a)"
  echo "cpu: $(grep -m1 -E 'Model|model name' /proc/cpuinfo | cut -d: -f2- | xargs)"
  echo "input: $INPUT"; echo "topic: $TOPIC"
  echo "size: $SIZE threads: $THREADS conf: $CONF warmup: $WARMUP repeat: $REPEAT"
  echo "git: $(git -C "$HERE" rev-parse --short HEAD 2>/dev/null || echo unknown)"
} > "$OUT/env.txt"

for ((r = 1; r <= REPEAT; r++)); do
  for B in $BACKENDS; do
    case $B in
      ncnn) MODEL_ARGS=(--ncnn-param "${NCNN_PARAM:?NCNN_PARAM}" --ncnn-bin "${NCNN_BIN:?NCNN_BIN}") ;;
      onnx) MODEL_ARGS=(--onnx "${ONNX_MODEL:?ONNX_MODEL}") ;;
      *) echo "unknown backend $B"; exit 1 ;;
    esac
    TAG="${B}_r${r}"
    SAVE_ARGS=()
    [[ "$SAVE_EVERY" -gt 0 && $r -eq 1 ]] && SAVE_ARGS=(--save-every "$SAVE_EVERY" --save-dir "$OUT/frames")

    echo "== $TAG"
    "$BENCH" --backend "$B" "${MODEL_ARGS[@]}" --input "$INPUT" --topic "$TOPIC" \
      --size "$SIZE" --threads "$THREADS" --conf "$CONF" --warmup "$WARMUP" \
      --out "$OUT/$TAG.csv" "${SAVE_ARGS[@]}" &
    PID=$!
    python3 "$HERE/sys_monitor.py" --pid "$PID" --out "$OUT/${TAG}_sys.csv" &
    MON=$!
    wait "$PID"
    wait "$MON" || true

    echo "   cooldown ${COOLDOWN}s (온도가 내려간 상태에서 다음 실행)"
    sleep "$COOLDOWN"
  done
done

echo "done -> $OUT"
echo "분석: python3 $HERE/compare_backends.py $OUT"
