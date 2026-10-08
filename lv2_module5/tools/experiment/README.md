# tools/experiment — 실험 기록 · 분석

로봇(라즈베리파이)에서 bag 을 기록하고, PC 에서 bag 만 읽어 지표를 계산한다. 결과 정리는 `results/README.md`.

| 파일 | 어디서 | 하는 일 |
|---|---|---|
| `start_bag.sh <RUN>` | 로봇 | R1 용 bag (검출 · 상태 · 명령 · 관절 · IMU) |
| `start_bag_inject.sh <RUN>` | 로봇 | 일반 시험 bag (+ planning · control 전체, 진단, `/inject/event`) |
| `start_bag_jpeg.sh <RUN>` | 로봇 | 위 + 컬러 JPEG 5 fps (`tools/inject/jpeg_tap.py` 를 먼저 켬) |
| `start_bag_rgb.sh <RUN>` | 로봇 | 위 + 원본 컬러 · depth (입력 재처리용. 기록 부하가 커서 카메라 + perception 만 켜고 10 ~ 20 s) |
| `analyze_run.py`, `compute_ex_rmse.py` | PC | R1 분석 (bag → timeseries.csv · events.csv · summary.json, e_x RMSE) |
| `analyze_1008.py` | PC | bag 만으로 상태 전이 · 모의 입력 · 정상 추적 지표 · 입력 재처리 비교 재계산 |

- bag 스크립트는 `~/experiment_logs/<RUN>/` 에 저장하고, 같은 이름이 있으면 덮어쓰지 않고 멈춘다 (회차를 지우지 않기 위해).
- 시작할 때 `~/code_fingerprint.txt` (실행 코드 sha256) 를 함께 복사한다. 로봇에 git 이 없어서 어떤 코드였는지 이 파일로 확인한다.
- 모의 입력 · 장애 주입: `tools/inject/mock_inputs.py`, `tools/inject/fault_injector.py`.
