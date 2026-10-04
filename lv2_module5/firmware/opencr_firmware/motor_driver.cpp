// Motor Driver: 바퀴 2 + 팔 2 구동 (~100Hz)
#include "motor_driver.h"

#include <Dynamixel2Arduino.h>

using namespace ControlTableItem;

namespace
{
// ---------------- 장치 설정 ----------------
#define DXL_SERIAL   Serial3        // OpenCR 다이나믹셀 포트
const int DXL_DIR_PIN = 84;         // OpenCR 방향 핀
const uint32_t DXL_BAUD = 1000000;  // 실제 모터 baud 로 맞출 것

const uint8_t ID_WHEEL_L = 1;       // XM430-W210-T, 터틀봇3 기본: 왼쪽 1, 오른쪽 2
const uint8_t ID_WHEEL_R = 2;
const uint8_t ID_ARM_YAW   = 11;     // XM430-W350-T, (스캔으로 확인)
const uint8_t ID_ARM_PITCH = 12;     // XM430-W350-T

// 장착 방향: +명령일 때 바퀴가 앞으로 / 팔이 왼쪽(yaw)·위(pitch)로 가도록 ±1 로 맞춤
const float WHEEL_DIR[2] = {+1.0f, +1.0f};
const float ARM_DIR[2]   = {+1.0f, -1.0f}; 
const int32_t ARM_ZERO[2] = {2048, 2048};   // 팔이 0 rad 일 때의 raw 위치

// 안전 한계 (Pi 의 Arm Command 가 먼저 걸지만, 보드에서 한 번 더)
const float MAX_WHEEL   = 7.8f;                  // rad/s (W210 무부하 약 77rpm=8.1rad/s @12V 보다 아래)
const float ARM_YAW_LIM = 90.0f * DEG_TO_RAD;    // ±
const float ARM_PITCH_MIN = -30.0f * DEG_TO_RAD;
const float ARM_PITCH_MAX =  60.0f * DEG_TO_RAD;
const float ARM_PROFILE_DPS = 150.0f;           // 모터 자체 최대 속도: Pi 의 arm_max_dps(120) 보다 약간 높게 → 평소엔 안 걸리고
                                                // Pi 쪽 버그로 큰 점프가 와도 확 튀지 않게 막는 백업 (W350 무부하 약 46rpm=276°/s)

// ---------------- 단위 변환 ----------------
const float RAW_PER_REV = 4096.0f;                     // 위치 1바퀴 = 4096
const float RPM_PER_RAW = 0.229f;                      // 속도 1 = 0.229 rpm
const float RAD_PER_POS = 2.0f * PI / RAW_PER_REV;
const float RADS_PER_VEL = RPM_PER_RAW * 2.0f * PI / 60.0f;

// X 시리즈 컨트롤 테이블: Present Velocity(128, 4B) + Present Position(132, 4B) 를 한 번에 읽음
const uint16_t ADDR_PRESENT_VELOCITY = 128;

const uint8_t WHEEL_ID[2] = {ID_WHEEL_L, ID_WHEEL_R};
const uint8_t ARM_ID[2] = {ID_ARM_YAW, ID_ARM_PITCH};

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
MotorState state;

float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

int32_t arm_raw(int i, float rad)   // 팔 각도 rad → raw 위치
{
  return ARM_ZERO[i] + (int32_t)lroundf(ARM_DIR[i] * rad / RAD_PER_POS);
}

void setup_wheel(uint8_t id)
{
  dxl.torqueOff(id);
  dxl.setOperatingMode(id, OP_VELOCITY);
  dxl.torqueOn(id);
}

// i = 0(yaw) / 1(pitch), lo·hi = 관절 한계 [rad]
void setup_arm(int i, uint8_t id, float lo, float hi)
{
  dxl.torqueOff(id);                                   // EEPROM 은 토크 꺼진 상태에서만 쓸 수 있음
  dxl.setOperatingMode(id, OP_POSITION);
  int32_t a = arm_raw(i, lo), b = arm_raw(i, hi);      // 방향이 -1 이면 순서가 뒤집히므로 정렬
  dxl.writeControlTableItem(MIN_POSITION_LIMIT, id, min(a, b)); 
  dxl.writeControlTableItem(MAX_POSITION_LIMIT, id, max(a, b));
  int32_t prof = (int32_t)lroundf(ARM_PROFILE_DPS / 6.0f / RPM_PER_RAW);   // °/s → rpm(÷6) → raw
  dxl.writeControlTableItem(PROFILE_VELOCITY, id, prof);
  dxl.torqueOn(id);
}

// 속도·위치를 한 번에 읽기 (실패하면 false, 이전 값 유지)
bool read_vel_pos(uint8_t id, int32_t &vel, int32_t &pos)
{
  uint8_t buf[8];
  if (dxl.read(id, ADDR_PRESENT_VELOCITY, 8, buf, sizeof(buf), 5) != 8) return false;
  memcpy(&vel, buf, 4);
  memcpy(&pos, buf + 4, 4);
  return true;
}
}  // namespace

void motor_setup()
{
  dxl.begin(DXL_BAUD);
  dxl.setPortProtocolVersion(2.0);
  setup_wheel(ID_WHEEL_L);
  setup_wheel(ID_WHEEL_R);
  setup_arm(0, ID_ARM_YAW, -ARM_YAW_LIM, ARM_YAW_LIM);
  setup_arm(1, ID_ARM_PITCH, ARM_PITCH_MIN, ARM_PITCH_MAX);
}

void motor_read()
{
  int32_t v, p;
  for (int i = 0; i < 2; i++) {
    if (read_vel_pos(WHEEL_ID[i], v, p)) {
      state.wheel_vel[i] = WHEEL_DIR[i] * v * RADS_PER_VEL;
      state.wheel_pos[i] = WHEEL_DIR[i] * p * RAD_PER_POS;
    }
  }
  for (int i = 0; i < 2; i++) {
    if (read_vel_pos(ARM_ID[i], v, p)) {
      state.arm_vel[i] = ARM_DIR[i] * v * RADS_PER_VEL;
      state.arm_pos[i] = ARM_DIR[i] * (p - ARM_ZERO[i]) * RAD_PER_POS;
    }
  }
}

const MotorState &motor_state() { return state; }

void motor_write(const float wheel[2], const float arm[2], bool arm_valid, bool timed_out)
{
  // 바퀴: 타임아웃이면 0
  for (int i = 0; i < 2; i++) {
    float w = timed_out ? 0.0f : clampf(wheel[i], -MAX_WHEEL, MAX_WHEEL);
    dxl.setGoalVelocity(WHEEL_ID[i], (int32_t)lroundf(WHEEL_DIR[i] * w / RADS_PER_VEL), UNIT_RAW);
  }
  // 팔: 명령을 받은 적 없거나 타임아웃이면 새 목표를 안 씀 (Pi 가 한 주기씩 조금씩 보내므로 그 자리에서 멈춤)
  if (!arm_valid || timed_out) return;
  float q[2] = {clampf(arm[0], -ARM_YAW_LIM, ARM_YAW_LIM),
                clampf(arm[1], ARM_PITCH_MIN, ARM_PITCH_MAX)};
  for (int i = 0; i < 2; i++) {
    dxl.setGoalPosition(ARM_ID[i], arm_raw(i, q[i]), UNIT_RAW);
  }
}
