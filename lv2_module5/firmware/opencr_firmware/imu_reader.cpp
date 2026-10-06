// IMU Reader: OpenCR 내장 IMU 로 yaw · 각속도 z 계산 (loop 마다 update)
#include "imu_reader.h"

#include <IMU.h>      // OpenCR 보드 패키지 내장 라이브러리 (cIMU)
#include <math.h>

namespace
{
cIMU imu;

// TurtleBot3 OpenCR 펌웨어와 같은 자이로 환산값: raw(±2000 dps) → rad/s
const float GYRO_FACTOR = 0.0010642f;

const uint32_t CALI_TIMEOUT_MS = 5000;   // 자이로 보정 최대 대기
const uint32_t SETTLE_MS = 1000;         // 보정 후 자세 필터 안정화 시간

bool cali_done = false;
uint32_t cali_start_ms = 0;
uint32_t cali_done_ms = 0;
}  // namespace

void imu_setup()
{
  imu.begin();
  imu.SEN.gyro_cali_start();             // 이 동안 로봇이 움직이면 yaw 가 흐른다
  cali_start_ms = millis();
}

void imu_update()
{
  imu.update();
  if (!cali_done) {
    uint32_t now = millis();
    if (imu.SEN.gyro_cali_get_done() || now - cali_start_ms > CALI_TIMEOUT_MS) {
      cali_done = true;
      cali_done_ms = now;
    }
  }
}

bool imu_ready()
{
  return cali_done && (millis() - cali_done_ms > SETTLE_MS);
}

float imu_yaw()
{
  if (!imu_ready()) return NAN;
  // quat = {w, x, y, z}
  const float w = imu.quat[0], x = imu.quat[1], y = imu.quat[2], z = imu.quat[3];
  return atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
}

float imu_gyro_z()
{
  if (!imu_ready()) return NAN;
  return imu.SEN.gyroADC[2] * GYRO_FACTOR;
}
