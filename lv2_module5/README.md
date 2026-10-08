# lv2_module5 — 비전 객체 추적 시스템 (Live.Eat)

RealSense로 파란 퍽을 찾고(YOLO, NCNN), TurtleBot3 Waffle Pi 차체와 2축 팔(pan·tilt)로 따라가는 ROS 2 시스템이다. 목표를 놓치거나 입력·통신이 끊기면 정지하고 복귀한다.

이 문서만 보고 **설치 → 빌드 → 펌웨어 업로드 → 실행 → 중지 → 기록 → 재현**을 할 수 있게 적었다. 명령은 모두 `lv2_module5/` 폴더 기준이다.

| 문서 | 내용 |
|---|---|
| [`report.md`](report.md) | 문제 1~5 구현·설정·검증·해석·한계 |
| [`results/README.md`](results/README.md) | 실험별 결과·판정·근거 파일 (평가표 4·6·7·8·9) |
| [`recordings/README.md`](recordings/README.md) | bag·영상 위치(Google Drive), 크기, sha256, 재생 방법 |
| [`team.md`](team.md) | 4인 역할·Issue·PR·리뷰 |
| [`presentation.md`](presentation.md) | 5분 시연 순서 |
| [`docs/functional_flow.md`](docs/functional_flow.md) | 노드별 기능 흐름도 (코드 리뷰 지도) |

---

## 1. 환경·장비·버전

| 구분 | 항목 | 값 |
|---|---|---|
| 로봇 | SBC | Raspberry Pi 4 Model B (RAM 4 GB), 호스트 `pa24` |
| | OS / ROS | Ubuntu 26.04.1 LTS (kernel 7.0.0-1020-raspi) / ROS 2 **Lyrical** |
| | 카메라 | Intel RealSense D435, `realsense2_camera` 4.58.4. color 640×360 @30 fps, depth 424×240 → color 정렬 |
| | 추론 | ncnn `f947448` 소스 빌드 (Vulkan OFF), 배포 모델 v4 `ros2_ws/src/perception/models/target_blue_v4_192` (입력 320×192, 스레드 2) |
| | 차체 | TurtleBot3 Waffle Pi, 바퀴 XM430-W210-T (ID 1 왼쪽, 2 오른쪽, 속도 모드) |
| | 팔 | XM430-W350-T (ID 11 pan, 12 tilt, 위치 모드) |
| | MCU | OpenCR, DYNAMIXEL Protocol 2.0 · 1 Mbps (`firmware/opencr_firmware/motor_driver.cpp`), PC 시리얼 1 Mbps `/dev/ttyACM0`, 펌웨어 루프 100 Hz |
| PC | 분석·재생 | Ubuntu + ROS 2 Lyrical (Jazzy에서도 planning·control 빌드 확인), Python 3 |
| 공통 | 파이썬 도구 | 저장소 루트 `requirements.txt` (numpy·pandas·matplotlib·psutil·pyyaml) |

- 실제 DYNAMIXEL ID·baud는 장비에서 스캔해 확인한 값이며 펌웨어 상수와 같다. 다른 장비면 `motor_driver.cpp`의 `ID_*`, `DXL_BAUD`를 먼저 맞춘다.
- 로봇에는 git이 없다. 로봇 코드는 PC 저장소의 `lv2_module5/`를 `~/lv2_module5/`로 복사해 쓰고, 어떤 코드였는지는 회차마다 `code_fingerprint.txt`(파일 sha256)로 남긴다(6절).

## 2. 폴더

```
lv2_module5/
├── config/          camera·perception·planning·control 파라미터 (YAML)
├── launch/          bringup.launch.py (전체), perception.launch.py (카메라+인지)
├── ros2_ws/src/     perception (C++) · planning (Python) · control (C++)
├── firmware/        OpenCR 펌웨어 (opencr_firmware.ino)
├── scripts/         build_pi.sh, run_perception.sh
├── tools/
│   ├── experiment/  bag 기록 스크립트 · 분석 (analyze_1008.py 등)
│   ├── inject/      모의 입력(mock_inputs.py) · 장애 주입(fault_injector.py) · JPEG tap
│   ├── benchmark/   NCNN/ONNX 속도 비교
│   └── sim/         가상 노드 시뮬레이션
├── results/         원본 CSV · 상태 기록 · 판정 이미지 · 그래프 · metrics.csv
└── recordings/      bag · 영상 위치 (파일은 Drive)
```

## 3. 설치 (처음 한 번)

### 3.1 로봇 (Raspberry Pi, SSH 접속)

```bash
# ROS 2 Lyrical이 설치되어 있다고 가정
sudo apt install libopencv-dev ros-lyrical-cv-bridge ros-lyrical-realsense2-camera   # 배포판에 realsense 패키지가 없으면 소스 빌드

# ncnn (C++) 소스 빌드 → ~/ncnn-install
git clone https://github.com/Tencent/ncnn.git && cd ncnn && git checkout f947448
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DNCNN_VULKAN=OFF -DNCNN_BUILD_EXAMPLES=OFF -DNCNN_BUILD_TOOLS=OFF \
      -DNCNN_BUILD_BENCHMARK=OFF -DNCNN_BUILD_TESTS=OFF -DCMAKE_INSTALL_PREFIX=$HOME/ncnn-install ..
make -j4 install

# OpenCR 펌웨어 도구 (업로드도 로봇에서 한다)
#   arduino-cli 설치 후 OpenCR 보드 패키지 + Dynamixel2Arduino 라이브러리
arduino-cli core install OpenCR:OpenCR      # OpenCR 보드 매니저 URL을 arduino-cli 설정에 먼저 추가
arduino-cli lib install Dynamixel2Arduino
```

코드 복사 (PC에서):

```bash
scp -r ~/Lv2_LiveEat_Assignment/lv2_module5 pa24@pa24.local:~/      # 로봇 주소는 환경에 맞게
scp lv2_module5/tools/experiment/start_bag*.sh pa24@pa24.local:~/    # bag 스크립트는 로봇 ~/ 에 둔다
```

### 3.2 PC (분석·재생만)

```bash
cd ~/Lv2_LiveEat_Assignment
./setup_venv.sh && source .venv/bin/activate      # --system-site-packages: apt의 rclpy·rosbag2_py 사용
```

## 4. 빌드·펌웨어 업로드 (로봇)

```bash
cd ~/lv2_module5
./scripts/build_pi.sh                    # perception(NCNN) + planning + control. 일부만: ./scripts/build_pi.sh planning control
#   ncnn 경로가 다르면 NCNN_DIR=<prefix>/lib/cmake/ncnn ./scripts/build_pi.sh
source ros2_ws/install/setup.bash
```

OpenCR 펌웨어 (펌웨어를 바꿨을 때만):

```bash
# control_master와 시리얼 모니터를 먼저 끈다 — 같은 포트(/dev/ttyACM0)를 동시에 쓰면 업로드가 실패한다
arduino-cli compile --fqbn OpenCR:OpenCR:OpenCR firmware/opencr_firmware
arduino-cli upload  --fqbn OpenCR:OpenCR:OpenCR -p /dev/ttyACM0 firmware/opencr_firmware
```

- 전원·리셋 후 약 2~6 s는 OpenCR IMU 자이로 보정 시간이다. 이 동안 로봇을 움직이지 않는다.
- 업로드 확인: OpenCR USER LED 1이 켜져 있으면 명령 대기(watchdog) 상태, control_master를 실행하면 꺼진다.

## 5. 실행·중지 (로봇)

모든 터미널에서 먼저:

```bash
cd ~/lv2_module5 && source /opt/ros/lyrical/setup.bash && source ros2_ws/install/setup.bash
```

| 목적 | 명령 |
|---|---|
| 전체 실행 (카메라 → 인지 → 판단 → 제어) | `ros2 launch launch/bringup.launch.py` |
| 상태 대시보드 화면 | `ros2 launch launch/bringup.launch.py dashboard:=true` |
| 모터 출력 끄고 실행 (연결 시험) | `ros2 launch launch/bringup.launch.py motor_enable:=false` |
| 기록 파일 남기기 | `ros2 launch launch/bringup.launch.py event_log:=results/logs/planning_events_<RUN>.txt csv_log:=results/logs/planning_<RUN>.csv run_id:=<RUN>` |
| planning만 따로 띄우기 | ① `ros2 launch launch/bringup.launch.py planning:=false` ② `ros2 run planning planning_master --ros-args --params-file config/planning.yaml` |
| 카메라 + 인지만 | `./scripts/run_perception.sh` (모델 `model:=v1~v4`, 출력 토픽 `output_topic:=/target` 등) |

- `run_id`는 `B0-01`처럼 글자를 섞는다(숫자만 쓰면 CSV에 숫자로 기록된다).
- 실행 중 모터 끄기·켜기: `ros2 param set /control_master motor_enable false` / `true`
- 상태 확인: `ros2 topic echo /tracking_status std_msgs/msg/String --field data`

**중지**

| 상황 | 방법 | 결과 |
|---|---|---|
| 정상 종료 | launch 터미널에서 `Ctrl+C` | control_master가 종료하면서 정지 명령을 보낸다 |
| 즉시 정지만 | `ros2 param set /control_master motor_enable false` | 노드는 살아 있고 모터에는 정지만 나간다 |
| 판단 입력이 끊김 | (자동) `/detection` 0.5 s 미수신 | planning FAULT(`detection_timeout`), 차체 정지 |
| 판단 명령이 끊김 | (자동) `/planning/cmd_vel` 0.3 s 미수신 | control이 감속 정지 |
| 제어 프로그램·USB가 끊김 | (자동) OpenCR에 명령 0.3 s 미수신 | 펌웨어 watchdog이 바퀴 정지, USER LED 1 점등 |

## 6. 실험 기록 (로봇)

회차를 시작하기 전에 실행 코드 지문을 만든다(로봇에 git이 없어 이것으로 코드 버전을 확인한다):

```bash
cd ~/lv2_module5
sha256sum config/planning.yaml config/control.yaml launch/bringup.launch.py launch/perception.launch.py \
  ros2_ws/src/control/src/control_master.cpp ros2_ws/src/planning/planning/{planning_master,health_monitor,dashboard}.py \
  tools/inject/{fault_injector.py,planning_inject.yaml,mock_inputs.py} > ~/code_fingerprint.txt
```

| 기록 | 명령 (로봇 `~/`) | 담는 것 |
|---|---|---|
| 일반 시험 bag | `bash ~/start_bag_inject.sh <RUN>` | `/detection`, `/tracking_status`, `/planning/*`, `/control/*`, 진단, `/inject/*` |
| + 컬러 JPEG 5 fps | 먼저 `python3 tools/inject/jpeg_tap.py`, 그다음 `bash ~/start_bag_jpeg.sh <RUN>` | 위 + `/camera/color/jpeg` (성공·소실 bag, 검출률 판정용) |
| + 원본 컬러·depth | 카메라 + perception만 켜고 `bash ~/start_bag_rgb.sh <RUN>` (10~20 s, 약 35 MB/s) | 입력 재처리용 원본 |
| R1 (가림) | `bash ~/start_bag.sh <RUN>` | 검출·상태·명령·관절·IMU |
| 모의 입력 5종 | perception 끄고, 팔 정면 [0, 0]° → `motor_enable:=false` 상태에서 `python3 tools/inject/mock_inputs.py 0.4` | `/detection`에 20 Hz로 IN1~IN5 순서 발행 |

- bag은 `~/experiment_logs/<RUN>/bag`, 같은 이름이 있으면 덮어쓰지 않고 멈춘다(회차를 지우지 않는다).
- 각 회차 폴더에 `meta.txt`(run_id·시작 시각·토픽)와 `code_fingerprint.txt`가 함께 남는다.
- control CSV(`results/logs/control_<날짜시간>.csv`)는 control_master가 자동으로 50 Hz 기록한다.
- 끝나면 bag·control CSV·planning CSV를 PC의 `results/logs/<RUN>/`(작은 파일)과 Drive(bag)로 옮긴다.

## 7. 재현 (PC, 모터 출력 없이)

### 7.1 bag 받기·확인

Drive [`LiveEat_drive_upload`](https://drive.google.com/drive/folders/1R_BAn832ETpi6gQRwt-JQcdjtKF2fso_)를 통째로 받아 압축을 풀고 그 안에서:

```bash
sha256sum -c SHA256SUMS.txt          # 모두 OK 여야 한다
```

이하 `bags/`는 받은 Drive 폴더 안의 `bags/`다.

### 7.2 재생 (로봇과 섞이지 않게)

```bash
export ROS_DOMAIN_ID=99 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
source /opt/ros/lyrical/setup.bash
ros2 bag info bags/B0-01
ros2 bag play bags/BAG-OK_success_trim
ros2 topic echo /tracking_status std_msgs/msg/String --field data     # 다른 터미널
bash bags/M-T_split/play.sh all                                        # M-T1~M-T8 상태 재생
```

- 재생할 때 control_master와 OpenCR은 켜지 않는다. 기록된 `/planning/cmd_vel`을 모터로 다시 보내지 않기 위해서다.

### 7.3 결과 재분석 — bag만 읽어 지표 재계산

```bash
python3 tools/experiment/analyze_1008.py inputs      bags/IN-02              # 모의 입력 5종 (평가 4)
python3 tools/experiment/analyze_1008.py transitions bags/M-T_SC-01          # 상태 전이 M-T1~6 (평가 6)
python3 tools/experiment/analyze_1008.py transitions bags/M-T_SC-01b         # M-T7~8 (평가 7)
python3 tools/experiment/analyze_1008.py tracking    bags/B0-01              # 30 s 정상 추적: TRACKING 비율·FPS·RMSE (평가 8)
python3 tools/experiment/analyze_1008.py tracking    bags/BAG-OK_success_trim
python3 tools/experiment/analyze_1008.py transitions bags/BAG-LOST
python3 tools/experiment/analyze_1008.py replay      bags/RAW-01 bags/RE2-01 # 입력 재처리 비교 (평가 9)
```

가림 R1은 `analyze_run.py`가 `<run_dir>/bag/` 구조를 읽으므로 폴더를 하나 만들어 연결한다. 결과(`timeseries.csv`·`events.csv`·`summary.json`)는 그 폴더에 생긴다.

```bash
for r in R1-02 R1-03 R1-04 R1-05 R1-06; do
  mkdir -p r1_out/$r && ln -sfn "$PWD/bags/$r" r1_out/$r/bag
  python3 tools/experiment/analyze_run.py r1_out/$r --kind r1
  python3 tools/experiment/compute_ex_rmse.py r1_out/$r     # e_x RMSE
done
```

같은 값이 [`results/README.md`](results/README.md)와 [`results/metrics.csv`](results/metrics.csv)에 있다. 기대값 몇 가지:

| 명령 | 나와야 하는 값 |
|---|---|
| `inputs IN-02` | x=+0.4 → ω −0.20, x=−0.4 → ω +0.20, 발행 중단 → 0.505 s 뒤 FAULT |
| `tracking B0-01` | FAULT·SEARCHING 0, TRACKING 100 %, 8.64 Hz, RMSE 0.0003 |
| `replay RAW-01 RE2-01` | 같은 촬영 시각 172 / 172 검출 일치, x·y·z 차이 0 |

### 7.4 입력 재처리 — bag 영상을 검출기에 다시 넣기

perception이 빌드된 환경(로봇 또는 perception을 빌드한 PC)에서. 새 결과는 저장된 `/detection`과 섞이지 않게 `/target_replay`로 낸다.

```bash
M=$(ros2 pkg prefix perception)/share/perception/models/target_blue_v4_192
ros2 run perception perception_master --ros-args --params-file config/perception.yaml \
  -p model_param:=$M/model.ncnn.param -p model_bin:=$M/model.ncnn.bin -p model_onnx:=$M/model.onnx \
  -p output_topic:=/target_replay -p use_sim_time:=true -p usb_check:=false
ros2 bag record -e '^/target_replay$' -o RE2-new                          # 새 결과는 새 폴더에
ros2 bag play bags/RAW-01 --clock -r 0.3 \
  --topics /camera/camera/color/image_raw /camera/camera/aligned_depth_to_color/image_raw
python3 tools/experiment/analyze_1008.py replay bags/RAW-01 RE2-new        # 비교
```

- `--clock`과 `use_sim_time:=true`를 함께 건다. 과거 bag 시각과 현재 벽시계를 섞지 않기 위해서다.
- planning을 bag 입력으로 다시 돌릴 때도 같다: `ros2 run planning planning_master --ros-args --params-file config/planning.yaml -p use_sim_time:=true` + `ros2 bag play <bag> --clock --topics /detection /control/joint_states /control/imu /control/odom_yaw_deg`

### 7.5 재현 확인 기록

| 확인자 | 날짜 | 기준 코드 | 실행한 것 | 결과 |
|---|---|---|---|---|
| 권혁무, 정수용, 박준명, 김민식 | 2026-10-08 | `results/README.md` 0절 (회차별 `code_fingerprint.txt`) | 7.3 결과 재분석, 7.4 입력 재처리 | 성능표와 같은 값 |
| 정수용 (작성자가 아닌 팀원) | 2026-10-08 | 제출 태그 `lv2-module5-submit` | 이 README만 보고 7.1~7.3 | `tracking B0-01`: RMSE 0.0003 (일치), 처리 FPS 8.64 Hz (일치), `replay RAW-01 RE2-01`: 172 / 172 일치 |

## 8. 결과 위치

| 무엇 | 어디 |
|---|---|
| 실험별 원본 (control CSV·planning CSV·상태 전이·meta·지문) | `results/logs/<RUN>/` (B0-01, BAG-OK, BAG-LOST, IN-01, IN-02, M-C2, M-T/SC-01·SC-01b, R1/R1-02~06, RAW-01, RE2-01) |
| 지표 요약 | `results/metrics.csv`, `results/README.md` 1절, `results/plots/summary_1008.png` |
| 검출 3장면 | `results/images/scene_{normal,none,occluded}_{raw,det}.jpg` |
| 검출률 30프레임·오검출 판정 | `results/images/detection_30/`, `results/images/no_target_20/` |
| 그래프 | `results/plots/` |
| 인지 속도 측정 | `results/realtime_ncnn_vs_onnx.md`, `results/fps_root_cause.md`, `results/logs/controlled_*` |
| 제어 단독 시험 발췌 | `results/logs/excerpts/` |
| bag·영상 | Drive — [`recordings/README.md`](recordings/README.md) |
