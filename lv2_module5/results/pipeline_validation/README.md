# 파이프라인 검증 — 2026-10-05 (갱신: 같은 날 23시)

대상: perception → planning_master.py → control → OpenCR 펌웨어 (로컬 작업 트리 기준).

결론: 계산 모듈 검사와 가상 노드 시뮬레이션은 통과했어요. 실제 하드웨어(카메라·모터)와 실제 DDS 통신 시험은 아직 안 했어요. 남은 문제는 health 진단 발행 측 미구현, 펌웨어 IMU 미통합, 근거리 소실 후 재탐색 불가, 가림 복구 지연이에요.

> 갱신 메모
> - 처음 버전에서 쓴 개별 검사 파일(`liveeat-*-check.*`)과 `control/test/test_arm_hold.cpp`는 정리 차원에서 삭제했어요. 결과는 아래 표에 남겨 뒀어요.
> - 처음 버전의 발견 사항 7개 중 5개는 작업 트리에서 해결됐어요. 다만 이 수정들은 **아직 커밋되지 않은 로컬 변경**이라, 커밋하기 전에 팀이 리뷰해야 해요(아래 "커밋 전 확인" 참고).

## 실행 결과

| 검증 | 결과 |
|---|---|
| ROS 2 Jazzy planning 패키지 빌드 | 통과 (최초 검증) |
| ROS 2 Jazzy control 패키지 빌드 | 통과 (최초 검증) |
| perception 패키지 빌드 | NCNN CMake 패키지 미설치로 실패 (최초 검증) |
| planning health unittest (`planning/test/test_health_integration.py`) | 18개 통과 (최초 검증) |
| planning 좌표·명령 검사 | 6개 통과: 정면, pan 90°, tilt 60°, bbox 부호, 거리 deadband, 측면 물체 회전 (검사 파일 삭제됨) |
| perception DepthExtractor 직접 실행 | 5개 통과: mm→m, float m, 0/NaN 거부, 영상 밖 ROI (검사 파일 삭제됨) |
| control 계산 모듈 직접 실행 | 7개 통과: 직진, 회전 부호, 휠 제한, timeout 감속, odom yaw, 팔 속도/각도 제한 (검사 파일 삭제됨) |
| control 팔 hold 단위 검사 (`test_arm_hold.cpp`) | 통과 기록 없음, 파일 삭제됨 |
| **가상 노드 시뮬레이션 (`tools/sim/`)** | 아래 "시뮬레이션 결과" 참고. 실제 planning 코드 + 가상 인지/제어, `/sim/*` 토픽 |
| 실제 DDS 노드 간 송수신, 모델 추론, 카메라/모터 실험 | 미실시 |
| 펌웨어 빌드/업로드 | 미실시: arduino-cli와 하드웨어 없음 |

계산·콜백 테스트와 시뮬레이션 통과가 실제 노드 간 통신이나 하드웨어 동작 성공을 의미하지는 않아요.

## 시뮬레이션 결과 (`lv2_module5/tools/sim/`)

- **구성:**
  - `fake_planning.py`는 실제 `PlanningMaster`를 그대로 import해서 토픽 이름에만 `/sim`을 붙여요.
  - `fake_perception.py`는 가상 퍽을 시야 모델(HFOV 69°, VFOV 42°, depth 0.2~3.0 m, 카메라 높이 0.20 m 가정)에 넣어 `/sim/detection`을 보내요.
  - `fake_control.py`는 차체 가속도 제한, 팔 120°/s, cmd/arm timeout 0.3 s를 흉내 내요.
- **확인 방법:** 메시지 버스를 흉내 낸 오프라인 환경에서 세 파일을 동시에 실행해 확인했어요. 실제 ROS 실행은 아직이에요.

| 시나리오 | 결과 |
|---|---|
| 시작 (퍽 1 m 앞) | IDLE → 0.1초 뒤 TRACKING, 거리 0.42 m에 수렴 |
| 오른쪽 / 왼쪽으로 빠르게 이탈 (5 m/s) | `lost_side=right/left`, 시계/반시계 탐색, 0.8초 만에 재추적 |
| 퍽 제거 | SEARCHING → 약 16초 뒤 LOST → IDLE |
| `/detection` 침묵, NaN, joint 끊김, heading 없음 | LOST(정지) → 복구 후 IDLE → TRACKING |
| IMU 무효 표시 (covariance −1) | 엔코더 yaw로 탐색, 정상 종료 |
| health on, 진단 토픽 없음 | `mcu_fault`로 LOST 고정 |
| health on, 진단 OK 발행 / ERROR / OpenCR 끊김 | 추적 / `mcu_fault` 정지 / `mcu_fault` 정지 |

## 발견 사항

| # | 내용 | 상태 |
|---|---|---|
| 1 | 전체 bringup 미구현 | **해결 (미커밋)**: `launch/bringup.launch.py`에 perception·planning·control 실행 추가 |
| 2 | 팔 명령 타입·단위 불일치 (Float64, rad) | **해결 (미커밋)**: control이 Float32MultiArray deg를 구독해 rad로 변환 |
| 3 | health 발행 측 미구현 | **미해결**: perception/control에 DiagnosticStatus publisher가 없어요. `health.enabled=true`면 시작부터 `mcu_fault`로 정지해요 |
| 4 | 펌웨어 IMU 통합 미완료 (yaw·gyro 항상 0) | **미해결**: control이 정상 쿼터니언으로 보내서 planning이 IMU를 선택하고, searching 누적각이 늘지 않아요. 시뮬레이션에서 30초 이상 탐색이 끝나지 않는 걸 재현했어요. 통합 전까지 `orientation_covariance[0] = -1`로 무효 표시하면 엔코더 fallback이 동작해요 |
| 5 | 모터 읽기 실패가 상위로 전달되지 않음 | **해결 (미커밋)**: 펌웨어가 읽기 실패 시 NaN을 보내고, control은 NaN이면 joint_states/odom을 보내지 않아요 |
| 6 | control이 stale 피드백으로 계속 구동 | **해결 (미커밋)**: 상태가 무효이거나 0.5 s 넘게 끊기면 휠 정지, 팔 명령 취소 |
| 7 | 팔 명령 소실 watchdog 없음 | **부분 해결 (미커밋)**: control이 `arm_cmd_timeout_s`(0.3 s) 뒤 측정 자세로 hold해요. 시리얼 자체가 끊기면 펌웨어는 마지막 GoalPosition을 유지해요 |
| 8 | 근거리 소실 후 재탐색 불가 (시뮬레이션에서 발견) | **미해결**: 0.42 m에서 LOST → IDLE이 되면 팔이 nominal(tilt 0°)로 돌아가요. 그러면 바닥의 퍽이 화면 아래(앙각 25° > VFOV/2 21°)로 벗어나 IDLE에 머물러요. 실제 카메라 높이로 재확인이 필요하고, nominal tilt를 아래로 두거나 IDLE에서도 탐색하는 방안을 검토해야 해요 |
| 9 | 짧은 가림 후 복구 지연 (시뮬레이션에서 발견) | **미해결**: 2초 가림에도 searching이 0.8 rad/s로 돌아버려 재추적까지 약 11초 걸려요. 발제의 "3초 이내 복구" 지표에 불리해요. 미검출 후 일정 시간 정지·대기한 뒤 탐색하는 방안을 검토해야 해요 |

## 커밋 전 확인

git index와 작업 트리를 비교하면 아래 파일이 마지막 pull(21:55) 이후 로컬에서 바뀌었거나 삭제됐어요. 리뷰 후 커밋하거나 되돌려야 해요.

- **수정됨:**
  - `config/control.yaml`, `config/planning.yaml`
  - `launch/bringup.launch.py`
  - `firmware/opencr_firmware/motor_driver.{cpp,h}`
  - `control/include/control/{arm_command,base_kinematics}.hpp`
  - `control/src/{arm_command,control_master}.cpp`
  - `planning/planning/{planning_master,health_monitor}.py`
- **삭제됨:** `planning/planning/{angle_error,distance_controller,rotation_allocator,target_estimator}.py`
- **새 파일 (git 미추적):** `planning/test/test_health_integration.py`, `tools/sim/`, 이 보고서

## 일치하는 인터페이스

- RealSense aligned depth/color 토픽 이름과 perception 기본 입력이 일치해요.
- perception `/detection`: PointStamped, 정규화 x/y는 오른쪽·아래가 양수, z는 m, 미검출(및 depth 실패)은 0이에요. planning 입력과 타입·부호·단위·best-effort QoS가 일치해요.
- `/control/imu`(Imu), `/control/odom_yaw_deg`(Float32 deg)는 planning 구독과 일치해요. 단, 위 4번 IMU 값 문제는 별도예요.
- `/control/joint_states`: 관절 이름 `arm_yaw_joint`/`arm_pitch_joint`, rad→deg 변환이 일치해요. tilt는 위쪽이 +예요.
- `/planning/cmd_vel`: Twist m/s·rad/s가 일치해요. control은 가속도 제한 후 휠 속도를 계산해요.
- `/planning/arm_command`: Float32MultiArray [pan, tilt] deg가 일치해요(2번 해결 후).
- 시리얼 명령/상태의 필드 순서·float 개수·체크섬 정의가 양측 소스에서 일치해요. 실제 전송 검증은 미실시예요.
- pan ±120°, tilt 하드웨어 −80..85°가 일치해요. planning의 추가 tilt 상한 69°(= 90 − VFOV/2)는 더 좁은 명령 범위예요.

## 정지 동작 해석

planning은 장애 판정 시 0 속도를 보내요. control은 이를 바로 휠 0으로 바꾸지 않고 가속도 제한으로 감속해요.

- 기본 `max_acc_v=0.5 m/s²`에서 0.2 m/s → 0은 약 0.4초예요.
- `max_acc_w=3 rad/s²`에서 1.665 → 0은 약 0.555초예요.
- 둘 다 0 명령을 받은 뒤의 계산값이고, 실제 제동은 측정하지 않았어요.
- cmd_vel이 끊기면 별도로 0.3초 timeout이 있어요.

그래서 "즉시 물리적 정지"로 해석하면 안 돼요.

## 재현 명령

저장소 루트에서 실행해요.

```bash
# 빌드 (시스템 Python 지정 필요: 사용자 Python 3.10에는 catkin_pkg 없음)
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 -m colcon --log-base /tmp/liveeat-pipeline-log build \
  --base-paths lv2_module5/ros2_ws/src \
  --build-base /tmp/liveeat-pipeline-build \
  --install-base /tmp/liveeat-pipeline-install \
  --continue-on-error --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3

# planning health unittest
export ROS_LOG_DIR=/tmp/liveeat-health-ros-logs
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export PYTHONPATH="$PWD/lv2_module5/ros2_ws/src/planning:$PWD/lv2_module5/ros2_ws/src/planning/test:$PYTHONPATH"
/usr/bin/python3 -m unittest discover -s lv2_module5/ros2_ws/src/planning/test -v

# 가상 노드 시뮬레이션 (터미널 3개, 키보드로 장애 주입)
python3 lv2_module5/tools/sim/fake_control.py
python3 lv2_module5/tools/sim/fake_perception.py
python3 lv2_module5/tools/sim/fake_planning.py     # health 검사: --health
```
