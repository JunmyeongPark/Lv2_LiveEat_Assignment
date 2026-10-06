// IMU Reader: OpenCR 내장 IMU 로 yaw · 각속도 z 계산 (loop 마다 update)
//
//   - OpenCR 보드 라이브러리의 cIMU 사용 (TurtleBot3 펌웨어와 같은 방식)
//   - 시작 직후 자이로 보정(로봇 정지 필요) + 필터 안정화 시간 동안은 "준비 안 됨"
//     → 상태 패킷에 NaN 을 보내고, control 은 /control/imu 를 발행하지 않는다
//       (planning 은 IMU timeout → 엔코더 heading 으로 자동 대체)
//   - yaw [rad] : -π ~ π, 반시계 + (ROS 규약), 기준 0 은 전원/리셋 시점 방향
//   - gyro_z [rad/s] : 반시계 +
#pragma once

#include <Arduino.h>

void imu_setup();

// 매 loop 호출 (자주 부를수록 자세 계산이 정확). 내부적으로 200Hz 로 갱신
void imu_update();

// 보정·안정화가 끝났는지
bool imu_ready();

// 준비 전이면 NaN
float imu_yaw();
float imu_gyro_z();
