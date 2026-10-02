// Serial 프로토콜: 명령 수신 · 상태 송신 형식
#include "serial_protocol.h"

namespace
{
const uint8_t TYPE_CMD = 0x01, TYPE_STATE = 0x02;
const uint8_t CMD_DATA_LEN = 1 + 4 * 4;      // seq + float 4개
const uint8_t STATE_DATA_LEN = 4 + 4 * 10;   // ms + float 10개

void handle_cmd(const uint8_t *d, Command &cmd)
{
  uint8_t seq = d[0];
  float f[4];
  memcpy(f, d + 1, 16);
  cmd.wheel[0] = f[0];
  cmd.wheel[1] = f[1];
  if (seq != 0) {                  // seq 0: 팔 값 무시
    cmd.arm[0] = f[2];
    cmd.arm[1] = f[3];
    cmd.arm_valid = true;
  }
}
}  // namespace

void protocol_setup(uint32_t baud)
{
  Serial.begin(baud);   // USB 는 값 무관, control_master 의 baud 와 같게 둠
}

// 바이트 단위 상태 기계: 0xAA → 0x55 → 종류 → 데이터 → 체크섬
bool protocol_receive(Command &cmd)
{
  static uint8_t st = 0, type = 0, len = 0, idx = 0, sum = 0;
  static uint8_t data[CMD_DATA_LEN];
  bool got = false;
  while (Serial.available()) {
    uint8_t b = Serial.read();
    switch (st) {
      case 0: st = (b == 0xAA) ? 1 : 0; break;
      case 1: st = (b == 0x55) ? 2 : (b == 0xAA ? 1 : 0); break;
      case 2:
        type = b; sum = b; idx = 0;
        if (type == TYPE_CMD) { len = CMD_DATA_LEN; st = 3; }
        else st = 0;               // 모르는 종류는 버림
        break;
      case 3:
        data[idx++] = b; sum += b;
        if (idx >= len) st = 4;
        break;
      case 4:
        if (b == sum) {            // 체크섬 틀리면 버림
          handle_cmd(data, cmd);
          got = true;
        }
        st = 0;
        break;
    }
  }
  return got;
}

void protocol_send_state(uint32_t ms, float imu_yaw, float imu_gyro_z, const MotorState &m)
{
  uint8_t pkt[2 + 1 + STATE_DATA_LEN + 1];
  pkt[0] = 0xAA; pkt[1] = 0x55; pkt[2] = TYPE_STATE;
  float f[10] = {
    imu_yaw, imu_gyro_z,
    m.wheel_pos[0], m.wheel_pos[1], m.wheel_vel[0], m.wheel_vel[1],
    m.arm_pos[0], m.arm_pos[1], m.arm_vel[0], m.arm_vel[1]};
  memcpy(pkt + 3, &ms, 4);
  memcpy(pkt + 7, f, 40);
  uint8_t sum = 0;
  for (int i = 2; i < 3 + STATE_DATA_LEN; i++) sum += pkt[i];
  pkt[3 + STATE_DATA_LEN] = sum;
  Serial.write(pkt, sizeof(pkt));
}
