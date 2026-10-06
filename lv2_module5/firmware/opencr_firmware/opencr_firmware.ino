// firmware loop(): millis 스케줄링 ① Watchdog → ② Motor Driver → ③ IMU
//
// [빌드·업로드]  라즈베리파이에서 (SSH)
//   arduino-cli compile --fqbn OpenCR:OpenCR:OpenCR firmware/opencr_firmware
//   arduino-cli upload  --fqbn OpenCR:OpenCR:OpenCR -p /dev/ttyACM0 firmware/opencr_firmware
//   ※ 업로드 중엔 control_master 를 끌 것 (같은 포트 동시 사용 금지)
//   필요: OpenCR 보드 패키지, Dynamixel2Arduino 라이브러리
//
// [IMU]  ③ imu_reader: OpenCR 내장 IMU. 전원/리셋 후 약 2~6초 자이로 보정 동안 로봇을 정지시킬 것.
//        보정 전에는 상태 패킷 IMU 칸에 NaN → control 이 /control/imu 미발행 → planning 은 엔코더 heading 사용
#include "motor_driver.h"
#include "serial_protocol.h"
#include "imu_reader.h"
#include "watchdog.h"

const uint32_t PC_BAUD = 1000000;   // control_master 의 baud 와 같게
const uint32_t LOOP_MS = 10;        // 100Hz

Command cmd;
uint32_t last_loop_ms = 0;

void setup()
{
  protocol_setup(PC_BAUD);
  motor_setup();
  watchdog_setup();
  imu_setup();
}

void loop()
{
  if (protocol_receive(cmd)) {      // 바이트는 매 루프 처리 (놓치지 않게)
    watchdog_feed(millis());
  }
  imu_update();                     // IMU 는 매 루프 (자주 불러야 자세 계산이 맞음)

  uint32_t now = millis();
  if (now - last_loop_ms < LOOP_MS) return;
  last_loop_ms = now;

  bool timed_out = watchdog_check(now);                       // ①
  motor_read();                                               // ②
  motor_write(cmd.wheel, cmd.arm, cmd.arm_valid, timed_out);
  protocol_send_state(now, imu_yaw(), imu_gyro_z(), motor_state());   // ③ 준비 전이면 IMU 칸 NaN
}
