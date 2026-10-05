// Motor Driver: 바퀴 2 + 팔 2 구동 (~100Hz)
//   바퀴: XM430-W210-T Velocity 모드 (속도 PID 는 모터 내부)
//   팔  : XM430-W350-T Position 모드 (위치 PID 는 모터 내부, 한 바퀴 0~4095 안에서만 움직임)
//         → 모터 EEPROM 의 Min/Max Position Limit 에도 팔 한계를 써서 이중으로 막음
//   둘 다 X 시리즈라 컨트롤 테이블 주소·단위가 같음
//
// [실물에서 확인해서 고칠 값]  ← 다른 팀 값 복사 금지 (발제)
//   motor_driver.cpp 의 ID_*, DXL_BAUD, WHEEL_DIR, ARM_DIR, ARM_ZERO, 팔 한계
#pragma once

#include <Arduino.h>

// 모터 상태 (rad, rad/s)
struct MotorState
{
  float wheel_pos[2] = {0, 0};   // 왼쪽, 오른쪽. Velocity 모드에선 여러 바퀴 누적됨
  float wheel_vel[2] = {0, 0};
  float arm_pos[2] = {0, 0};     // yaw, pitch
  float arm_vel[2] = {0, 0};
};

// 통신 시작 + 바퀴 Velocity 모드 / 팔 Position 모드 설정
void motor_setup();

// 바퀴·팔 현재 위치·속도 읽기 (실패한 모터는 위치·속도 NaN으로 표시)
void motor_read();
const MotorState &motor_state();

// wheel: 목표 바퀴 속도 [rad/s], arm: 목표 팔 각도 [rad]
// timed_out 이면 바퀴 0, 팔은 새 목표를 안 씀 (그 자리 유지)
void motor_write(const float wheel[2], const float arm[2], bool arm_valid, bool timed_out);
