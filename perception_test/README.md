# perception_test — 인지(minsikim) PC 실험·학습 작업 공간

와플파이 배포 빌드(`lv2_module5/`)는 이 폴더에 의존하지 않는다.
배포 모델은 `lv2_module5/ros2_ws/src/perception/models/`에 복사되어 있고, 여기는 그 원본(학습·export)과 PC 시험 도구만 둔다.

## 폴더 구조

```
perception_test/
├── filter_FOV.py            RealSense D435 RGB/Depth FOV 시각화
└── yolo/
    ├── realsense.py         실행: 최신 설정만 (Pi와 같은 320x180 · v4 · 왼쪽 끝 depth 거부) + 녹화(webm)
    ├── realsense_test.py    시험: 지금까지 시험한 기능 전부 (모델·크기·해상도·HSV·형상 검사·크기 검사·스냅샷)
    ├── tuto.py              RealSense 마우스 위치 3D 좌표 예제
    ├── tools/
    │   ├── prepare_dataset.py   makesense YOLO zip → dataset/
    │   └── make_169_dataset.py  dataset/ (4:3) → dataset_169/ (16:9, D435 기하 그대로)
    ├── data/labels/         makesense export zip        (git 제외)
    ├── models/              사전학습 모델 yolo26n.pt     (git 포함)
    ├── dataset/             학습용 데이터, 자동 생성      (git 제외)
    ├── dataset_169/         dataset/의 16:9 변환본        (git 제외)
    ├── runs/                학습 결과 (best.pt, NCNN·ONNX export만 git 포함 → 배포 모델의 원본)
    └── .yolo/               가상환경                    (git 제외)
```

## 실행

```
cd perception_test/yolo
source .yolo/bin/activate

python realsense.py                  # 최신 설정 (Pi와 같음): color 320x180@30, v4 320x192

python realsense_test.py                         # 시험 도구, 320x180 · v4로 시작
python realsense_test.py --model v2              # v2로 시작
python realsense_test.py --res 640x360           # 같은 화각, 2배 화질
python realsense_test.py --res 640x480 --model v3 --fps 15   # 예전 4:3 카메라
python realsense_test.py --reject shape          # 이전에 시도한 깊이 형상 검사로 시작
```

`realsense.py` 키: `D` 검출 ON/OFF · `R` 녹화 (검출 OFF면 학습용 원본 320x180) · `Q` 종료

`realsense_test.py` 키: `D` 검출 · `M` YOLO/HSV · `V` v1→v2→v3→v4 · `I` 입력 크기 · `E` depth 거부 방식 (왼쪽 끝 → 형상 검사 → 끔) · `Z` 박스 크기 검사 ON/OFF · `F` 형상 점수 fill 포함 · `1~9` 형상 허용오차 cm · `S` 스냅샷(npz) · `R` 녹화 · `Q` 종료
(depth는 항상 color에 정렬. 정보 창에 박스 px·추정 실제 크기와 기대값(퍽 긴 변 3.0~6.7 cm) 표시, 보라 사각형 = 그 depth에서의 기대 박스 크기.
종료 시 모델·입력 크기별 속도·검출률, 정렬 시간·지연 출력. 320x180은 화면만 2배로 그림)

| 모델 | 경로 | 데이터 |
|---|---|---|
| v1 | `runs/target_blue/weights/best.pt` | record 영상 1개 |
| v2 | `runs/target_blue_v2/weights/best.pt` | + YOLO_DARK/EAST/ROOM2~5, 라벨 수정 (542장, `data/labels/labels_v2.zip`) |
| v3 | `runs/target_blue_v3/weights/best.pt` | + YOLO_BLUE0(퍽처럼 생긴 방해물, 일부러 라벨 없음)/BLUE2 (628장, `data/labels/labels_v3.zip`) |
| v4 | `runs/target_blue_v4_169/weights/best.pt` | v3와 같은 628장을 16:9로 변환(`dataset_169/`), `imgsz=320`. **Pi 배포 모델** (320x192) |

v1~v3는 `imgsz=640`, v4는 `imgsz=320`으로 학습했다. v3로 320x256·256x192·160x128 (16:9면 320x192·256x160·160x96) 입력을 고르면 그 크기로 학습한 모델이 자동으로 쓰인다 (`runs/target_blue_v3_imgsz320`·`imgsz256`·`imgsz160`, 화면에 `v3@320`·`v3@256`·`v3@160`으로 표시). v3@320은 4:3 시절 Pi 배포 모델과 같은 가중치다. 설정: `realsense_test.py`의 `SIZE_MODELS`

## 16:9 카메라 (2026-10-07~)

Pi 카메라 color를 640x480@15 → **320x180@30(16:9)**로 바꿨다. 모델 입력 너비와 같아 축소 없이 추론한다 (180은 32의 배수가 아니라 실제 모델 입력은 320x192, 위아래 6px 패딩).
color 320x180은 6/30/60 FPS만 지원해서 30을 쓴다. depth는 424x240@30 (align이 320x180에 맞춤).

D435 color는 16:9 센서라서 640x480은 센서 가운데를 자른 것(fx 608, HFOV 55.5°)이고 640x360은 센서 전체를 0.75배 줄인 것(fx 456, HFOV 70.1°)이다 (pyrealsense2로 실측, ppx 327.5×0.75+80 = 325.6으로 일치).
320x180은 그것을 다시 절반으로 줄인 것(fx 228)이다.
그래서 기존 4:3 프레임을 0.75배(480x360) 줄여 좌우에 80px 회색을 붙이면 16:9 화면의 가운데와 픽셀 단위로 같다 (학습 때 imgsz=320으로 다시 절반이 되므로 320x180 카메라 영상과 같은 크기) → **라벨을 새로 할 필요 없이** 좌표 변환만 하면 된다 (`tools/make_169_dataset.py`).
한계: 좌우 80px(화각 바깥쪽 각 7°)는 학습 때 본 적이 없다. 16:9로 새로 녹화한 영상이 생기면 라벨을 더해 v5로 학습하는 것이 좋다.

```
python tools/make_169_dataset.py        # dataset/ → dataset_169/
yolo detect train model=models/yolo26n.pt data=dataset_169/data.yaml imgsz=320 epochs=100 project=$PWD/runs name=target_blue_v4_169
yolo export model=runs/target_blue_v4_169/weights/best.pt format=ncnn imgsz=192,320
```

## 학습

```
python tools/prepare_dataset.py data/labels/<export>.zip <프레임 폴더>
yolo detect train model=models/yolo26n.pt data=dataset/data.yaml imgsz=640 epochs=100 project=$PWD/runs name=target_blue
```

`project=`는 절대 경로로 줘야 함 (ultralytics 전역 설정 runs_dir가 다른 경로를 가리킴)

## 마지막 커밋(1002) 인수인계
이 PC에서 아직 없는 것

ROS2(lyrical)와 colcon은 있어요. 하지만 이 C++ 코드를 빌드하는 데 필요한 것 중 아래 네 가지가 설치되어 있지 않아요. 설치하려면 sudo 권한이 필요해서 직접 해 주셔야 해요. Pi에도 똑같이 필요해요.

필요한 것	설치 방법
OpenCV 개발 파일	sudo apt install libopencv-dev
cv_bridge	sudo apt install ros-lyrical-cv-bridge
realsense2_camera	sudo apt install ros-lyrical-realsense2-camera, 배포판에 없으면 소스 빌드
ncnn (C++)	소스 빌드: cmake -DNCNN_VULKAN=OFF -DNCNN_BUILD_EXAMPLES=OFF .., 그다음 make install
진행 순서
Detector 단독 시험: ROS 없이 perception_core와 시험용 main을 만들어요. val 이미지에서 나온 박스를 Python NCNN 결과와 비교해요(IoU 0.99 수준이면 성공).
노드 연결: perception_master에 연결하고, 녹화해 둔 rosbag이나 실제 카메라로 /detection이 나오는지 확인해요.
Pi4에서 측정: 같은 코드를 Pi에서 빌드하고 처리 속도를 재요. 그 결과로 input_size(640 또는 320)를 정해요.

원하시면 위 파일들(detector, depth_extractor, perception_master, CMakeLists, yaml)을 lv2_module5/ros2_ws/src/perception에 실제로 작성해 드릴게요. 이 PC에 필요한 라이브러리를 설치해 주시면 빌드와 결과 비교까지 할 수 있어요.