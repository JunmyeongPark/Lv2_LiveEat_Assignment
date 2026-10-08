# recordings — bag · 영상 위치

큰 파일은 저장소 대신 드라이브에 둔다. 폴더 구조는 아래 표와 같고, 모든 파일의 sha256 은 [`SHA256SUMS.txt`](SHA256SUMS.txt) 에 있다.

- **드라이브 폴더 링크**: https://drive.google.com/drive/folders/1R_BAn832ETpi6gQRwt-JQcdjtKF2fso_?usp=sharing (최상위 폴더 `LiveEat_drive_upload` 안에 `bags/`, `videos/`, `SHA256SUMS.txt`)
- 공유: 링크가 있는 모든 사용자 · 뷰어
- 받은 뒤 확인: 드라이브 폴더를 통째로 받아 그 안에서 `sha256sum -c SHA256SUMS.txt`

## bag (`bags/`, 각 폴더 = `metadata.yaml` + `.mcap`)

| 폴더 | 크기 | 내용 | 결과 문서 |
|---|---|---|---|
| `IN-02` | 6.9 MB | 모의 입력 5종 (모터 OFF) | `results/README.md` 2절 |
| `IN-01` | 8.7 MB | 모의 입력 (조건 미충족 회차) | `results/logs/IN-01/` |
| `M-T_SC-01` | 7.3 MB | 상태 전이 M-T1 ~ M-T6 (85 s) | 3절 |
| `M-T_SC-01b` | 2.8 MB | 상태 전이 M-T7 ~ M-T8 (31 s) | 3절 |
| `M-T_split/M-T1` ~ `M-T8` | 7.9 MB | 위 두 bag 을 시험별로 자른 것 (+ `play.sh`: 상태 재생) | 3절 |
| `R1-02` ~ `R1-06` | 1 MB 내외 | 2초 가림 × 5 (10/7 오후) | 4절 |
| `M-C2` | 0.9 MB | 제어 통신 중단 (`kill -9`) | 5절 |
| `B0-01` | 3.6 MB | 30 초 정상 추적 (영상 없음) | 6절 |
| `BAG-OK` | 14 MB | 성공 장면 원본 53.6 s (JPEG 5 fps 포함) | 7절 |
| `BAG-OK_success_trim` | 5.8 MB | 위에서 TRACKING 구간 24.1 s 만 자른 것 (**M-BAG 대표**) | 7절 |
| `BAG-LOST` | 6.2 MB | 소실 · 복귀 장면 30.1 s (JPEG 5 fps 포함, **M-BAGL 대표**) | 7절 |
| `RAW-01` | 597 MB | 원본 컬러 + depth 20.8 s (카메라 + perception 만) — 입력 재처리 원본 | 7절 |
| `RE2-01` | 68 KB | `RAW-01` 재처리 결과 `/target_replay` | 7절 |

공통 토픽: `/detection`, `/tracking_status`, `/planning/cmd_vel`, `/planning/arm_command`, `/control/*` (joint_states, imu, odom, 진단), `/perception/camera_health`, `/inject/event`.
영상 토픽: `BAG-OK` · `BAG-LOST` 는 `/camera/color/jpeg` (CompressedImage, 5 fps), `RAW-01` 은 `/camera/camera/color/image_raw` · `/camera/camera/aligned_depth_to_color/image_raw`.
기준 코드 · 조건: 각 bag 의 `results/logs/<run>/meta.txt` · `code_fingerprint.txt`.

## 영상 (`videos/`)

| 파일 | 크기 | 내용 |
|---|---|---|
| `M-T01.mp4` ~ `M-T08.mp4` | 1.3 ~ 11 MB | M-T 시험별 클립 (자막: 전이 · bag 측정값) |
| `IMG_9507.MOV`, `IMG_9508.MOV` | 37 MB, 6.3 MB | 위 클립의 원본 휴대폰 영상 |
| `BAG-OK_success.mp4` | 1.9 MB | `BAG-OK_success_trim` 의 JPEG 를 영상으로 (상태 · 검출 · 팔 각도 표시) |
| `DEMO_녹화명령재생_M-T1-8.mp4` | 38 MB | **시연용 · 재현 증빙 아님.** M-T1 ~ M-T8 bag 의 바퀴 · 팔 명령을 control_master 에 다시 보내 그때 동작을 재생한 장면 (영상 위에 같은 문구 표시) |

## 재생 (모터 출력 없이)

PC 에서 재생한다. 로봇 control_master 가 기록된 명령을 받지 않게 다른 도메인 · 로컬 전용으로 둔다.

```bash
export ROS_DOMAIN_ID=99 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
source /opt/ros/lyrical/setup.bash
ros2 bag info bags/B0-01
ros2 bag play bags/BAG-OK_success_trim
ros2 topic echo /tracking_status std_msgs/msg/String --field data   # 다른 터미널
```

- 지표 재계산 (노드 없이 파일만 읽음): `python3 tools/experiment/analyze_1008.py tracking bags/B0-01` 등 (`results/README.md` 0절)
- 입력 재처리 명령: `results/README.md` 7절 (`--clock` + `use_sim_time`, 출력은 `/target_replay` 로 분리)
- M-T 상태 재생: `bash bags/M-T_split/play.sh all`
