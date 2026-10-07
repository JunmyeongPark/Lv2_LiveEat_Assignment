# perception

RealSense color·depth → YOLO(NCNN) 검출 → depth 추출 → `geometry_msgs/PointStamped` 발행.
출력 규약은 `src/perception_master.cpp` 맨 위 주석 참고 (`z=0` 미검출, `z=NaN` depth 실패).

## 파일 구분

| 구분 | 경로 | 내용 |
|---|---|---|
| **배포 (와플파이)** | `src/`, `include/` | `perception_master` 노드, Detector(NCNN), DepthExtractor |
| | `models/target_blue_v4_192/` | 배포 모델 v4 (기본, 320x192 NCNN + ONNX). 카메라 color 640x360(16:9)을 320x180으로 축소 + 위아래 6px 패딩. v3 데이터를 16:9로 변환해 `imgsz=320`으로 학습한 `runs/target_blue_v4_169/weights/best.pt`에서 export, ncnn.bin md5 `0dc58597…` (변환 방법: `perception_test/README.md` 16:9 절) |
| | `models/target_blue_v3_256/` | 이전 모델 v3 (4:3 카메라용 320x256 NCNN + ONNX, `model:=v3`로 선택). 입력 크기에 맞춰 `imgsz=320`으로 학습한 `runs/target_blue_v3_imgsz320/weights/best.pt`에서 export, ncnn.bin md5 `021b3c49…`. 빌드 시 `share/perception/models/`로 설치 |
| | `models/target_blue_v2_256/` | 이전 모델 v2 (같은 형식, `imgsz=640` 학습 `runs/target_blue_v2/weights/best.pt`, ncnn.bin md5 `436893bb…`). `model:=v2`로 선택 |
| | `models/target_blue_256/` | 이전 모델 v1 (같은 형식, `runs/target_blue/weights/best.pt`, ncnn.bin md5 `4ceb5c3d…`). `model:=v1`로 선택 |
| | `../../../config/perception.yaml` | 노드 파라미터 (입력 320x192, launch가 모델에 맞춰 높이를 덮어씀) |
| | `../../../launch/perception.launch.py` | 카메라(color 640x360x30, depth 424x240x30) + 인지 노드 실행 |
| **실행** | `../../../scripts/build_pi.sh` | 배포 빌드 (NCNN만, perception·planning·control) |
| | `../../../scripts/run_perception.sh` | 카메라 + 인지 노드 실행 |
| **테스트·측정** | `src/onnx_detector.cpp` | ONNX Runtime 백엔드 (`-DPERCEPTION_WITH_ONNX=ON`일 때만 빌드) |
| | `tools/detector_bench.cpp` | bag 오프라인 벤치마크 (`-DPERCEPTION_BUILD_BENCH=ON`일 때만 빌드) |
| | `../../../tools/benchmark/` | 백엔드 비교 스크립트, `realtime/` 실시간 측정 스크립트 |
| | `../../../results/` | 측정 결과·로그 |

학습·export 원본(`best.pt`, 다른 해상도 모델)은 `perception_test/yolo/runs/target_blue/weights/`에 있다.

## 와플파이에서

```bash
# 1회: ncnn 소스 빌드 (tools/benchmark/README.md 3절의 NCNN 부분만)
./scripts/build_pi.sh
./scripts/run_perception.sh                       # 카메라 + 인지 (모델 v4)
./scripts/run_perception.sh model:=v3             # 이전 모델 v3 (4:3용, 16:9 영상에선 위아래 패딩으로 동작)
./scripts/run_perception.sh model:=v2             # 이전 모델 v2
./scripts/run_perception.sh model:=v1             # 이전 모델 v1
./scripts/run_perception.sh output_topic:=/target # 판단 노드 구독 토픽에 맞출 때
```

모델 교체: `imgsz=[H,W]`로 export한 NCNN 폴더를 `models/`에 넣고 `model_dir:=<폴더> input_height:=<H>`로 실행하거나
launch의 `MODELS`에 (폴더, 높이)를 추가한다. 너비는 `perception.yaml`의 `input_width`(320).
