# 제어 시험 CSV 발췌

control_master가 남긴 원본 CSV에서 시험 구간만 잘라낸 파일입니다 (50 Hz, 1행 = 0.02초).
원본 파일은 용량이 커서 저장소에 올리지 않았습니다.

| 파일 | 원본 / 구간 (시작 기준 초) | 시험 | 확인 값 |
|---|---|---|---|
| `T2_cmdvel_stop_20261003.csv` | `control_20261003_123152.csv` 183~187s | `/cmd_vel` 발행 중단 | 0.05 m/s → 0.1초 동안 감속 후 정지 (`v_ref`) |
| `T4_wheel_limit_20261003.csv` | `control_20261003_123932.csv` 888.5~892s | `x: 1.0` 요청 | `v_ref` 0.26 m/s, 바퀴 7.8 rad/s에서 제한, 가속 약 0.4초 |
| `T4_arm_limit_20261003.csv` | `control_20261003_131244.csv` 19.5~22s | 팔 `[2.0, 0.0]` 요청 | yaw 90°에서 정지, 120°/s로 이동 (`yaw_cmd`, `yaw`) |
| `odom_rotate_20261004.csv` | `control_20261004_180829.csv` 225~239.5s | `angular.z: 0.5` 회전 | `odom_w` 0.498, `yaw_total` 0 → 6.066 rad (명령 적분 6.050과 0.3% 차이) |
| `odom_straight_20261004.csv` | `control_20261004_180829.csv` 589.5~596s | `linear.x: 0.1` 직진 | `odom_v` 0.0985, 거리 0.447 m (명령 0.450), `yaw_total` 변화 0.08° |

- 시험 조건: 노트북 + OpenCR(USB), 바퀴를 띄운 상태, IMU 통합 전 (`from_imu` = 0)
- 10/3 파일과 10/4 파일은 칸 구성이 다릅니다. 10/4부터 각도 보정 칸(`thL_err` 등)이 빠지고 `odom_v, odom_w, yaw_total, from_imu`가 추가됨
- 10/5부터는 `yaw_total, from_imu` 대신 `odom_yaw`(엔코더 기반, ±π) 칸으로 기록됩니다. IMU yaw 는 원래대로 `imu_yaw` 칸에 있음 (토픽도 `/control/imu`, `/control/odom` 으로 변경)
- 각도 단위: rad, 속도: m/s · rad/s
