# Detector 백엔드 비교 (NCNN vs ONNX Runtime) — Raspberry Pi 4

같은 YOLO26 모델을 NCNN / ONNX로 변환해 **같은 입력 프레임**으로 속도·자원·출력 일치도를 비교한다.
전처리(letterbox)·후처리(출력 해석)는 두 백엔드가 같은 코드(`perception/src/detector.cpp`)를 쓰므로 **추론 엔진 차이만** 비교된다.

## 1. 비교 조건 (시험 전에 고정)
| 항목 | 값 | 비고 |
|---|---|---|
| 하드웨어 | Raspberry Pi 4 (RAM ?GB), 방열판/팬 여부 기록 | `env.txt`에 자동 기록 |
| 모델 | 같은 `best.pt`에서 export | 커밋·파일 해시 기록 |
| 입력 크기 | 두 백엔드 동일 (예: 320, 640x480) | export `imgsz`와 같아야 함. `--size`/`SIZE`는 `WxH`, imgsz는 `[H,W]` |
| 출력 head | 두 모델 모두 같은 형식 (기본 raw) | NCNN은 raw만 지원 → ONNX도 기본 export(raw) 사용 |
| 스레드 | 3 (1코어는 모터·IMU용) | `--threads` |
| 입력 | 같은 bag (color 토픽) | 실시간 재생이 아니라 **전 프레임 순차 처리** (드롭 없음) |
| 반복 | 백엔드별 3회, 실행 사이 60초 쿨다운 | 발열 영향 줄이기 |
| 워밍업 | 10회 (기록 제외) | |

## 2. 지표
| 구분 | 지표 | 정의 |
|---|---|---|
| 속도 | `infer_ms` mean / p50 / p95 / max | 엔진 추론 시간 |
| 속도 | `total_ms` mean / p95 | 전처리 + 추론 + 후처리 |
| 속도 | `fps` | 1000 / mean(total_ms) — 검출기 단독 처리 FPS |
| 안정성 | `total_ms_std` | 프레임별 처리 시간 흔들림 (지터) |
| 자원 | `proc_cpu_pct_mean`, `proc_rss_mb_max` | CPU 사용률(4코어 최대 400%), 최대 메모리 |
| 발열 | `temp_c_max`, `cpu_mhz_min`, `throttled_any` | 스로틀링 발생 시 속도 비교 해석 주의 |
| 준비 | `load_ms`, `model_mb` | 모델 로드 시간, 모델 파일 크기 |
| 출력 일치 | 검출 일치율, IoU mean/p5, \|Δe_x\| mean/max | 같은 프레임에서 두 백엔드 결과 비교 |
| 검출 품질 | 검출률(30프레임), 배경 오검출(10프레임) | `--save-every`로 저장한 이미지를 **사람이 대조** (detected_ratio는 정답 대조가 아님) |

## 3. 준비 (Raspberry Pi 4)
```bash
# 모델 변환 (PC에서 해도 됨) — 두 형식 모두 같은 imgsz
yolo export model=best.pt format=ncnn imgsz=320
yolo export model=best.pt format=onnx imgsz=320
# 카메라 640x480에 맞춘 직사각형 입력 (imgsz는 [높이, 너비] 순서) → 실행 시 SIZE=640x480
yolo export model=best.pt format=ncnn imgsz=480,640
yolo export model=best.pt format=onnx imgsz=480,640

# ONNX Runtime (aarch64 C++ 릴리스)
wget https://github.com/microsoft/onnxruntime/releases/download/v1.20.1/onnxruntime-linux-aarch64-1.20.1.tgz
tar xzf onnxruntime-linux-aarch64-1.20.1.tgz -C ~ && mv ~/onnxruntime-linux-aarch64-1.20.1 ~/onnxruntime

# NCNN (소스 빌드, Vulkan 끔)
git clone --depth 1 https://github.com/Tencent/ncnn.git && cd ncnn && mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DNCNN_VULKAN=OFF -DNCNN_BUILD_EXAMPLES=OFF -DNCNN_BUILD_TOOLS=OFF \
      -DNCNN_BUILD_BENCHMARK=OFF -DNCNN_BUILD_TESTS=OFF -DCMAKE_INSTALL_PREFIX=$HOME/ncnn-install ..
make -j4 install

# perception 빌드
cd lv2_module5/ros2_ws
colcon build --packages-select perception --cmake-args \
  -DONNXRUNTIME_ROOT=$HOME/onnxruntime -Dncnn_DIR=$HOME/ncnn-install/lib/cmake/ncnn
source install/setup.bash

# 분석 도구 파이썬 환경 (레포 루트)
./setup_venv.sh && source .venv/bin/activate
```

## 4. 데이터 취득 (bag)
```bash
# RealSense 실행 후 color만 기록 (30초 내외, 목표가 보이는 장면 + 없는 장면 포함)
ros2 bag record -o bench_input_$(date +%m%d) /camera/camera/color/image_raw
```
bag 이름·길이·해상도·FPS를 `recordings/README.md`에 기록.

## 5. 실행
```bash
cd lv2_module5/tools/benchmark
export NCNN_PARAM=~/models/best_ncnn_model/model.ncnn.param
export NCNN_BIN=~/models/best_ncnn_model/model.ncnn.bin
export ONNX_MODEL=~/models/best.onnx
SIZE=320 SAVE_EVERY=10 ./run_benchmark.sh ~/bags/bench_input_1003 ../../results/benchmark/1003 3
python3 compare_backends.py ../../results/benchmark/1003
```
단일 실행: `ros2 run perception detector_bench --help`

## 6. 결과물
| 파일 | 내용 |
|---|---|
| `<backend>_r<i>.csv` | 프레임별 pre/infer/post 시간, 검출 결과, e_x, e_y |
| `<backend>_r<i>.csv.meta.json` | 모델·입력 크기·스레드·로드 시간 |
| `<backend>_r<i>_sys.csv` | CPU·메모리·온도·클럭·스로틀링 (0.5초 간격) |
| `env.txt` | 실행 환경·조건·커밋 |
| `frames/` | 검출 결과를 그린 이미지 (사람 대조용) |
| `summary.md` / `summary.csv` / `per_run.csv` / `agreement.csv` | 비교표 |
| `plots/` | latency 분포, 프레임별 latency, 온도·CPU, e_x 일치도 |

## 7. 실시간 노드에서 확인 (선택)
`config/perception.yaml`의 `timing_csv`에 경로를 넣고 `backend`만 바꿔 실행하면, 실제 카메라 입력에서 프레임별 처리 시간을 기록한다 (ROS 오버헤드 포함).
