# 장애 주입 시연 — 순서 · 체크표

> 실제 로봇이 진짜 퍽을 따라가는 중에 `fault_injector.py` 키로 고장을 넣고, 반응과 복구를 확인한다.
> 퍽은 손으로 움직인다. 키 설명은 `fault_injector.py` 맨 위 주석을 참고한다.
> 판정은 `/tracking_status`(주입기 화면 맨 위 "상태" 줄)와 로봇 동작으로 한다.
> 상태 줄 형식: `상태|플래그 reason=원인 heading=imu|encoder|none diag=on`

---

## 0. 준비 (PC → 로봇 반영, 한 번만)

| ☐ | 할 일 | 명령 / 확인 |
|---|---|---|
| ☐ | 바뀐 파일 복사 | `control_master.cpp`, `config/control.yaml`, `planning/dashboard.py`, `launch/bringup.launch.py`, `launch/perception.launch.py`, `tools/inject/` 3개, `experiment_logs/tools/start_bag_inject.sh` |
| ☐ | control · planning 다시 빌드 | 로봇에서 `cd ~/lv2_module5/ros2_ws && colcon build --packages-select control planning` (perception 은 안 바뀜) |
| ☐ | **펌웨어 업로드는 안 함** | OpenCR 코드는 바뀐 게 없음 |
| ☐ | 새 코드 확인 | 실행 후 `ros2 param get /control_master imu_ignore` → `False` 가 나오면 새 코드 |
| ☐ | 코드 지문 갱신 | 로봇 `~/lv2_module5`에서 아래 명령 |

```bash
sha256sum config/planning.yaml config/control.yaml launch/bringup.launch.py ros2_ws/src/control/src/control_master.cpp ros2_ws/src/planning/planning/planning_master.py ros2_ws/src/planning/planning/health_monitor.py ros2_ws/src/planning/planning/dashboard.py launch/perception.launch.py tools/inject/fault_injector.py tools/inject/planning_inject.yaml > ~/code_fingerprint.txt
```

bag 스크립트는 기존 `start_bag.sh` 처럼 로봇 `~/` 에 둔다 (PC 에서):

```bash
scp ~/Lv2_LiveEat_Assignment/experiment_logs/tools/start_bag_inject.sh pa24@pa24.local:~/
```

---

## 1. 실행 (로봇 터미널 4개, 모두 `~/lv2_module5` 에서)

| ☐ | 터미널 | 명령 |
|---|---|---|
| ☐ | ① 전체 (planning 제외) | `ros2 launch launch/bringup.launch.py planning:=false` |
| ☐ | ② planning (토픽만 /inject 로) | 아래 명령 |
| ☐ | ③ 주입기 | `python3 tools/inject/fault_injector.py` |
| ☐ | ④ bag | `bash ~/start_bag_inject.sh INJ-01` (회차마다 번호 올림, 지우지 않음) |

```bash
ros2 run planning planning_master --ros-args --params-file config/planning.yaml --params-file tools/inject/planning_inject.yaml -p dashboard:=true -p event_log:=results/logs/planning_events.txt -p csv_log:=results/logs/planning_INJ-01.csv -p run_id:=INJ-01
```

- `csv_log`: 발제 권장 열 CSV (제어 주기 30 Hz 마다 한 줄: run_id, time_s, frame_id, detected, ex, ey, depth_m, state, reason, …, cmd_v, cmd_w, 팔 명령). 회차마다 파일명과 `run_id` 를 같이 바꾼다.
- `run_id` 는 `INJ-01` 처럼 글자를 섞는다 (`01` 만 쓰면 숫자로 읽혀 CSV 에 `1` 로 적힘).

- ② 화면이 planning 대시보드가 된다 (상태 · reason · 입력별 마지막 수신 시간 · 상태 전이 기록).
  대시보드는 planning 이 받은 값을 보여주므로 **주입 결과가 그대로 보인다** (줄 이름은 원래 토픽 이름 `/detection`, `/control/imu` 로 표시되지만 실제로는 `/inject/...` 값).
- `event_log` 파일에는 상태 · reason 이 바뀔 때마다 한 줄씩 남는다 (bag 없이도 전이 순서 확인용).
- 시연 화면 배치: ② 대시보드(반응) 옆에 ③ 주입기(누른 키)를 나란히 띄우면 "주입 → 반응"이 한눈에 보인다.

- 같은 와이파이에 시뮬이 켜져 있으면 노드가 섞인다. `ros2 node list` 에 fake_* 가 보이면 시뮬을 끄거나 모든 터미널에서 `export ROS_DOMAIN_ID=77`.
- ③ 주입기를 끄면 planning 명령이 control 로 안 간다 → control 이 0.3 s 뒤 감속 정지 (정상).
- 주입기는 끌 때 control 의 `imu_ignore=false`, `motor_enable=true` 로 되돌린다.

### 1-1. 기록 파일 — 어디에 무엇이 남나

모두 **명령을 실행한 폴더**(`~/lv2_module5`) 기준 `results/logs/` 에 쌓인다.

| 파일 | 만드는 곳 | 언제 | 내용 |
|---|---|---|---|
| `results/logs/planning_INJ-01.csv` | planning (`csv_log`) | 제어 주기 30 Hz 마다 한 줄 | 발제 권장 열: 검출 · 상태 · reason · 명령 |
| `results/logs/planning_events.txt` | planning (`event_log`) | 상태 · reason 이 바뀔 때만 | 전이 순서 (사람이 읽기용) |
| `results/logs/control_날짜시간.csv` | control_master (자동) | 50 Hz | 바퀴 목표 · 실제, 팔 명령 · 실제, IMU 원본, 엔코더 odom |
| `results/logs/inject_날짜시간.csv` | 주입기 (자동) | 키 누를 때 | 누른 키 · 시각 · 내용 |
| `~/experiment_logs/INJ-01/bag` | start_bag_inject.sh | 계속 | 위 전부의 원본 토픽 |

### 1-2. planning CSV 예시 (PC 모의 실행)

> ⚠️ **모의 결과다. 제출용 측정값이 아니다.** PC 에서 planning 만 띄우고 `/detection` 을 10 Hz 로 2 초 보냈다
> (앞 1 초 z=1.0 검출, 뒤 1 초 z=0 미검출, x=0.2). control 이 없어서 상태는 계속 `fault / control_timeout` 이다.
> 실제 로봇에서는 `tracking`, `searching` 등과 실제 명령값이 찍힌다.

```bash
ros2 run planning planning_master --ros-args --params-file config/planning.yaml -p health.enabled:=false -p csv_log:=results/logs/planning_INJ-01.csv -p run_id:=INJ-01
```

```csv
run_id,time_s,frame_id,detected,ex,ey,depth_m,state,reason,miss_streak,detect_streak,heading_source,cmd_v,cmd_w,arm_pan_cmd,arm_tilt_cmd,command_unit
INJ-01,1791422929.7946,1,1,0.2000,-0.1000,1.000,fault,control_timeout,0,0,none,0.0000,0.0000,0.00,0.00,v:m/s;w:rad/s;arm:deg
INJ-01,1791422930.6946,10,1,0.2000,-0.1000,1.000,fault,control_timeout,0,0,none,0.0000,0.0000,0.00,0.00,v:m/s;w:rad/s;arm:deg
INJ-01,1791422930.7946,11,0,0.2000,-0.1000,0.000,fault,control_timeout,1,0,none,0.0000,0.0000,0.00,0.00,v:m/s;w:rad/s;arm:deg
INJ-01,1791422930.9946,13,0,0.2000,-0.1000,0.000,fault,control_timeout,3,0,none,0.0000,0.0000,0.00,0.00,v:m/s;w:rad/s;arm:deg
INJ-01,1791422931.6946,20,0,0.2000,-0.1000,0.000,fault,control_timeout,3,0,none,0.0000,0.0000,0.00,0.00,v:m/s;w:rad/s;arm:deg
```

읽는 법
- `time_s`: ROS 시각 (epoch 초). bag · control CSV · inject CSV 와 같은 시계라 그대로 맞춰 붙일 수 있다.
- `frame_id`: 받은 `/detection` 메시지 번호. 제어 주기가 더 빨라 **같은 번호가 4~5 줄 반복**된다 → 프레임 단위 지표(검출률 · RMSE)는 `frame_id` 가 바뀌는 줄만 쓴다.
- `detected`: 1 = 유효한 검출(depth > 0), 0 = 미검출 · NaN. **0 인 줄의 `ex`, `ey` 는 쓰지 않는다** (인지가 보낸 값을 그대로 적을 뿐, 발제 규약상 z=0 이면 x·y 로 제어하지 않음).
- `miss_streak` 가 3 이 되는 줄 = SEARCHING 으로 넘어갈 조건이 찬 순간 (위 예시는 FAULT 라 상태는 안 바뀜).
- `cmd_v`, `cmd_w`, `arm_*_cmd` 는 **명령값**이다. 실제 바퀴 · 팔 값은 control CSV 에 있다 (발제: 명령을 실제 위치처럼 표시하지 않음).

---

## 2. 시작 전 확인 (키 누르기 전 = 평소와 같아야 함)

| ☐ | 확인 | 기대 |
|---|---|---|
| ☐ | `ros2 node info /planning_master` 구독 | `/inject/detection`, `/inject/imu`, `/inject/odom_yaw_deg`, `/inject/joint_states`, `/inject/health/*` 5개 |
| ☐ | 주입기 화면 "검출 전달" 횟수 | 계속 올라감 (약 7 Hz) |
| ☐ | 주입기 화면 "명령" | `planning 전달`, planning v/ω 값이 바뀜 |
| ☐ | 퍽을 1 m 앞에 둠 | 상태 `TRACKING reason=ok heading=imu diag=on` |
| ☐ | 퍽을 좌우·앞뒤로 움직임 | 몸통이 돌며 따라가고, 가까우면 뒤로 / 멀면 앞으로 |

여기서 이상하면 주입 시연 전에 원인부터 찾는다 (주입기 없이 bringup 기본 실행과 비교).

---

## 3. 시연 순서 · 체크표

공통 진행: **퍽 추종 중(TRACKING) → 키 → 반응 확인 → 같은 키(또는 0)로 해제 → 퍽을 다시 따라가는지 확인.**
FAULT 에서 풀리면 `IDLE reason=recovered_from:<원인>` → 퍽이 보이면 `TRACKING`.
각 회차 끝나면 `0`(모든 주입 해제)을 눌러 초기화한다.

### A. 인지 입력 (/detection)

| ☐ | 번호 | 키 | 무엇을 넣나 | 기대 반응 (상태 · 로봇) | 복구 기대 | 결과 · 메모 |
|---|---|---|---|---|---|---|
| ☐ | A1 | `t` | 2초 가림 (z=0 2초) | 약 0.3~0.5 s 뒤 `SEARCHING`, 제자리 회전 시작 | 2초 뒤 퍽이 시야에 있으면 `TRACKING` 복귀 | |
| ☐ | A2 | `o` (유지) | 계속 가림 | `SEARCHING` → 3바퀴(약 24 s) 회전 후 `LOST reason=target_lost` → `IDLE` | `o` 해제 + 퍽을 앞에 → `TRACKING` | |
| ☐ | A3 | `f` | 노이즈 (±0.01, 2 % 미검출) | **상태 유지** `TRACKING` (한 프레임 미검출로는 SEARCHING 안 감, 3프레임 연속 필요) | 해제해도 변화 없음 | |
| ☐ | A4 | `n` | NaN 검출값 | `FAULT reason=detection_invalid`, 바퀴 정지, 팔 [0, 0] | 해제 → `IDLE recovered_from:detection_invalid` → `TRACKING` | |
| ☐ | A5 | `p` | /detection 침묵 + camera health STALE | `FAULT reason=camera_diag_error` (+ `DETECTION_TIMEOUT` 플래그) | 해제 → `camera_diag_recovering` → `IDLE` → `TRACKING` | |

(`g` 퍽 제거는 실제 로봇에서 `o` 와 같은 효과라 따로 안 해도 됨)

### B. 방향(heading) — IMU · 엔코더

| ☐ | 번호 | 키 | 무엇을 넣나 | 기대 반응 | 복구 기대 | 결과 · 메모 |
|---|---|---|---|---|---|---|
| ☐ | B1 | `i` | IMU 끊김 (control `imu_ignore=true` + planning 쪽 IMU 끊김) | **추종 계속**, `reason=imu_fallback heading=encoder`, 플래그 `IMU_TIMEOUT`. ① 터미널에 `imu_ignore = true (실행 중 변경)` 로그 | 해제 → `heading=imu`, 방향 값이 IMU 로 다시 맞춰짐 | |
| ☐ | B2 | `i` 상태에서 `a` 또는 퍽 가림 `o` | IMU 없이 탐색 회전 | `SEARCHING` 회전량을 엔코더로 셈 → 3바퀴 후 `LOST` (IMU 와 몇 도 차이 나는지 CSV 로 확인) | `0` | |
| ☐ | B3 | `i` + `u` | IMU · 엔코더 방향 둘 다 끊김 | `FAULT reason=heading_timeout`, 정지 | `u` 해제 → `imu_fallback` 으로 복귀 | |
| ☐ | B4 | `u` (단독) | 엔코더 방향만 끊김 | **변화 없음** (`heading=imu` 그대로) — 예비가 끊겨도 주 센서로 동작 | — | |
| ☐ | B5 | `v` | IMU 무효 표시 (covariance −1) | `reason=imu_fallback heading=encoder`, 추종 계속 (planning 만 IMU 를 버림) | 해제 → `heading=imu` | |
| ☐ | B6 | `y` + 가림 `o` | IMU yaw 0 고정 (값은 정상처럼 옴) | `SEARCHING` 회전은 하지만 회전량이 0 으로 계산 → **3바퀴가 끝나지 않음** = 진단으로 못 잡는 고장 (한계 확인용) | `y` 해제 → 정상 계산 | |

### C. 제어 피드백 · 진단 (OpenCR · 모터)

| ☐ | 번호 | 키 | 무엇을 넣나 | 기대 반응 | 복구 기대 | 결과 · 메모 |
|---|---|---|---|---|---|---|
| ☐ | C1 | `j` | joint_states 끊김 | `FAULT reason=joint_timeout`, 정지, 팔 [0, 0] 명령 | 해제 → `IDLE` → `TRACKING` | |
| ☐ | C2 | `m` | 모터 OFF (control `motor_enable=false`) | 로봇이 **바로 멈춤**, `FAULT reason=arm_motor_diag_error`. ① 터미널에 `motor_enable = false` 로그 | 해제 → 모터 켜짐, 0 부터 가속 → `IDLE` → `TRACKING` | |
| ☐ | C3 | `e` | control health 4개 ERROR | `FAULT reason=mcu_diag_error` (mcu 가 최우선) | 해제 → `mcu_diag_recovering` → `IDLE` → `TRACKING` | |
| ☐ | C4 | `h` | health 5개 전부 침묵 | 0.5 s 뒤 `FAULT reason=mcu_diag_timeout` | 해제 → 복구 | |
| ☐ | C5 | `c` | OpenCR 끊김 (planning 이 보기에: 상태 · IMU 끊김, mcu ERROR, 명령 전달 중단) | `FAULT reason=mcu_diag_error` + `CONTROL_TIMEOUT` 플래그, 로봇은 control 명령 타임아웃(0.3 s)으로 감속 정지 | 해제 → 복구 | |
| ☐ | C6 | (키 없음) USB 뽑기 | 진짜 OpenCR 단절 | 보드 watchdog 으로 바퀴 정지, planning `mcu_diag_error` | 다시 꽂고 bringup 재시작 | 선택 (T-시험에서 한 항목) |

### D. 명령 (/planning/cmd_vel → control)

| ☐ | 번호 | 키 | 무엇을 넣나 | 기대 반응 | 복구 기대 | 결과 · 메모 |
|---|---|---|---|---|---|---|
| ☐ | D1 | `x` | 명령 끊김 | 상태는 `TRACKING` 그대로인데 **로봇은 0.3 s 뒤 감속 정지** (control 안전장치) | 해제 → 다시 따라감 | |
| ☐ | D2 | `z` | 명령 NaN | 로봇 감속 정지, ① 터미널에 `/planning/cmd_vel 비정상 값 … 감속 정지` 경고 (PR #28) | 해제 → 다시 따라감 | |

### E. 수동 조작 (방해 후 다시 찾기)

| ☐ | 번호 | 키 | 무엇을 넣나 | 기대 반응 | 복구 기대 | 결과 · 메모 |
|---|---|---|---|---|---|---|
| ☐ | E1 | `a` 또는 `d` 2~3 s → `space` | 로봇을 퍽에서 강제로 돌려 놓음 | 돌아가는 동안 퍽이 화면에서 벗어나 `SEARCHING` | `space` (planning 복귀) → 탐색 회전으로 퍽을 다시 찾아 `TRACKING` | |
| ☐ | E2 | `k` 상태에서 퍽을 좌우로 | 몸통은 붙잡아 두고 퍽만 이동 | 바퀴는 0, **팔(pan)만 퍽을 따라감** (팔 명령은 그대로 연결) | `space` → 몸통도 같이 따라감 | |
| ☐ | E3 | `w` / `s` + `+` `-` | 수동 전진 · 후진, 속도 단계 조절 | 화면 "수동 속도 n단계" 값대로 움직임 (1단계 = v 0.026 m/s, ω 0.182 rad/s, 최대 10단계) | `space` | |

---

## 3-1. perception 스레드 2 / 4 비교 (planning 대시보드 "라즈베리파이 · 인지 처리" 칸)

스레드 수는 perception 이 시작할 때만 읽으므로 **① bringup 을 다시 켜서** 바꾼다. 나머지 터미널은 그대로 둬도 된다.

```bash
ros2 launch launch/bringup.launch.py planning:=false num_threads:=2
```
```bash
ros2 launch launch/bringup.launch.py planning:=false num_threads:=4
```

- 대시보드의 `perception 스레드` 값이 바꾼 숫자로 나오는지 먼저 확인 (perception_master 파라미터를 3 초마다 읽음).
- 같은 조건으로 비교: 퍽 1 m 앞 정상 추종(TRACKING) 1 분 유지 후 값을 읽는다. 주입 키는 누르지 않는다.
- `pin_cpu:=true` 와 같이 쓰면 `num_threads` 로 준 값이 우선이다.

| ☐ | 스레드 | 코어 0 / 1 / 2 / 3 (%) | CPU 전체 (%) | /detection 평균 Hz | 최대 간격 (s) | 온도 · 클럭 | 상태 전이 (FAULT·SEARCHING 이 생겼나) |
|---|---|---|---|---|---|---|---|
| ☐ | 2 | | | | | | |
| ☐ | 4 | | | | | | |

참고 (10/7 측정, `ros2 topic hz`): 스레드 4 → 3.15 Hz · 최대 0.492 s, 스레드 2 → 6.94 Hz · 최대 0.355 s. 이번 값이 비슷하게 나오는지 본다.
대시보드의 최대 간격은 최근 10 초 기준이라 `ros2 topic hz` 의 전체 구간 max 와 조금 다를 수 있다.

---

## 4. 끝내기 · 기록 정리

| ☐ | 할 일 | 확인 |
|---|---|---|
| ☐ | `0` → `q` 로 주입기 종료 | 화면에 종료, `results/logs/inject_날짜시간.csv` 생성 |
| ☐ | control 설정 되돌아갔는지 | `ros2 param get /control_master imu_ignore` → `False`, `motor_enable` → `True` |
| ☐ | bag 종료 (Ctrl+C) | `~/experiment_logs/INJ-xx/bag` |
| ☐ | PC 로 복사 | bag, inject CSV, control CSV (`results/logs/control_*.csv`), planning CSV (`results/logs/planning_INJ-xx.csv`) |
| ☐ | 실패 · 잘못 시작한 회차도 지우지 않고 메모에 invalid 로 남김 | |

### 기록으로 재는 값 (bag)

- **반응 시간** = `/tracking_status` 가 바뀐 시각 − `/inject/event` 시각 (같은 bag 안이라 시계가 같음)
- **복구 시간** = 해제 이벤트 시각 → `TRACKING` 복귀 시각
- **IMU 끊김 동안 방향 오차** = control CSV 의 `imu_yaw`(원본, 끊긴 동안에도 기록) − `odom_yaw`(엔코더로 이어간 값)

### 발표 때 같이 적을 것

- 주입은 실제 노드와 planning 사이에서 메시지를 바꾸는 방식이라 전달 지연이 몇 ms 더 있다.
- `c`(OpenCR 끊김)는 planning 쪽에서 본 단절이다. 보드 자체 정지는 C6(USB 뽑기)로 확인한다.
- `m` 의 모터 진단 ERROR 는 주입기가 만든 것이다. control 은 모터 OFF 를 진단에 아직 싣지 않는다 (판단 쪽 이슈 2 와 같은 내용).
