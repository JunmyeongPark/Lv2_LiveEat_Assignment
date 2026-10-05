# perception_test — 인지(minsikim) PC 실험·학습 작업 공간

와플파이 배포 빌드(`lv2_module5/`)는 이 폴더에 의존하지 않는다.
배포 모델은 `lv2_module5/ros2_ws/src/perception/models/`에 복사되어 있고, 여기는 그 원본(학습·export)과 PC 시험 도구만 둔다.

## 폴더 구조

```
perception_test/
├── filter_FOV.py            RealSense D435 RGB/Depth FOV 시각화
└── yolo/
    ├── realsense_gpt.py     실행: YOLO/HSV 검출 + 깊이 + 녹화(webm)
    ├── tuto.py              RealSense 마우스 위치 3D 좌표 예제
    ├── tools/
    │   └── prepare_dataset.py   makesense YOLO zip → dataset/
    ├── data/labels/         makesense export zip        (git 제외)
    ├── models/              사전학습 모델 yolo26n.pt     (git 포함)
    ├── dataset/             학습용 데이터, 자동 생성      (git 제외)
    ├── runs/                학습 결과 (best.pt, NCNN·ONNX export만 git 포함 → 배포 모델의 원본)
    └── .yolo/               가상환경                    (git 제외)
```

## 실행

```
cd perception_test/yolo
source .yolo/bin/activate

python realsense_gpt.py
```

키: `D` 검출 ON/OFF · `M` YOLO/HSV 전환 · `R` 녹화 · `Q` 종료

YOLO 모델: `runs/target_blue/weights/best.pt`

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