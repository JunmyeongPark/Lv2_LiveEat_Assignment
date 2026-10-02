// firmware loop(): millis 스케줄링 ① Watchdog → ② Motor Driver → ③ IMU
//
// [빌드·업로드]  라즈베리파이에서 (SSH)
//   arduino-cli compile --fqbn OpenCR:OpenCR:OpenCR firmware/opencr_firmware
//   arduino-cli upload  --fqbn OpenCR:OpenCR:OpenCR -p /dev/ttyACM0 firmware/opencr_firmware
//   ※ 업로드 중엔 control_master 를 끌 것 (같은 포트 동시 사용 금지)
//   필요: OpenCR 보드 패키지, Dynamixel2Arduino 라이브러리
//
// [IMU]  ③ 은 imu_reader (수영 님 담당) 를 통합할 때 채움. 지금은 상태 패킷의 IMU 칸에 0 을 보냄
#include "motor_driver.h"
#include "serial_protocol.h"
#include "watchdog.h"
// TODO(통합): #include "imu_reader.h"

const uint32_t PC_BAUD = 1000000;   // control_master 의 baud 와 같게
const uint32_t LOOP_MS = 10;        // 100Hz

Command cmd;
uint32_t last_loop_ms = 0;

void setup()
{
  protocol_setup(PC_BAUD);
  motor_setup();
  watchdog_setup();
  // TODO(통합): IMU 초기화
}

void loop()
{
  if (protocol_receive(cmd)) {      // 바이트는 매 루프 처리 (놓치지 않게)
    watchdog_feed(millis());
  }
  // TODO(통합): IMU 업데이트 (자주 불러줘야 자세 계산이 맞음)

  uint32_t now = millis();
  if (now - last_loop_ms < LOOP_MS) return;
  last_loop_ms = now;

  bool timed_out = watchdog_check(now);                       // ①
  motor_read();                                               // ②
  motor_write(cmd.wheel, cmd.arm, cmd.arm_valid, timed_out);
  // TODO(통합): IMU yaw [rad], 각속도 z [rad/s] 를 imu_reader 값으로 교체 (지금은 0)   ③
  protocol_send_state(now, 0.0f, 0.0f, motor_state());
}
