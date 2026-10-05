// Serial 프로토콜: 명령 수신 · 상태 송신 형식
//
// [패킷]  라즈베리파이 control/serial_bridge.hpp 와 똑같이 맞출 것. 리틀 엔디언, float = 4바이트
//   공통        : 0xAA 0x55 | 종류(1) | 데이터 | 체크섬(1) = (종류 + 데이터 바이트 합) & 0xFF
//   Pi → OpenCR  0x01 : seq(uint8) + float[4]
//                      {왼쪽 바퀴 rad/s, 오른쪽 바퀴 rad/s, 팔 yaw rad, 팔 pitch rad}
//                      seq == 0 이면 팔 값은 무시 (Pi 가 팔 현재 자세를 아직 모를 때)
//   OpenCR → Pi  0x02 : 시간 ms(uint32) + float[10]
//                      {IMU yaw, IMU 각속도 z, 바퀴각 L, 바퀴각 R, 바퀴속도 L, 바퀴속도 R,
//                       팔 yaw, 팔 pitch, 팔 yaw 속도, 팔 pitch 속도}
//   rad ↔ 다이나믹셀 단위 변환은 motor_driver 에서 한다.
#pragma once

#include <Arduino.h>

#include "motor_driver.h"

// 마지막으로 받은 명령
struct Command
{
  float wheel[2] = {0, 0};   // 목표 바퀴 속도 [rad/s] (왼쪽, 오른쪽)
  float arm[2] = {0, 0};     // 목표 팔 각도 [rad] (yaw, pitch)
  bool arm_valid = false;    // 팔 명령(seq != 0)을 한 번이라도 받았는지
};

void protocol_setup(uint32_t baud);

// 들어온 바이트를 처리. 완성된 명령 패킷이 있으면 cmd 를 갱신하고 true
bool protocol_receive(Command &cmd);

// 상태 패킷 송신
void protocol_send_state(uint32_t ms, float imu_yaw, float imu_gyro_z, const MotorState &m);
