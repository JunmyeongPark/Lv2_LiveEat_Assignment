// Serial RX/TX: IMU · 관절 상태 수신 → control_master 가 /control/imu, /control/joint_states 발행 / 명령 송신 (0 포함 heartbeat)
//   여기서는 USB 시리얼 열기 + 패킷 만들기/풀기만 한다. 토픽 발행은 control_master 가 함.
//
// [패킷]  OpenCR 펌웨어(serial_protocol)와 똑같이 맞출 것. 리틀 엔디언, float = 4바이트
//   공통        : 0xAA 0x55 | 종류(1) | 데이터 | 체크섬(1) = (종류 + 데이터 바이트 합) & 0xFF
//   Pi → OpenCR  0x01 : seq(uint8) + float[4]
//                      {왼쪽 바퀴 rad/s, 오른쪽 바퀴 rad/s, 팔 yaw rad, 팔 pitch rad}
//                      seq == 0 이면 펌웨어는 팔 값을 무시 (Pi 가 팔 현재 자세를 아직 모를 때)
//   OpenCR → Pi  0x02 : 시간 ms(uint32) + float[10]
//                      {IMU yaw, IMU 각속도 z, 바퀴각 L, 바퀴각 R, 바퀴속도 L, 바퀴속도 R,
//                       팔 yaw, 팔 pitch, 팔 yaw 속도, 팔 pitch 속도}
//   rad ↔ 다이나믹셀 단위 변환은 펌웨어에서 한다.
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace control
{

struct RobotState
{
  uint32_t ms = 0;                      // OpenCR 시간
  double imu_yaw = 0.0, imu_gyro_z = 0.0;
  double wheel_pos[2] = {0.0, 0.0};     // 왼쪽, 오른쪽 [rad]
  double wheel_vel[2] = {0.0, 0.0};     // [rad/s]
  double arm_pos[2] = {0.0, 0.0};       // yaw, pitch [rad]
  double arm_vel[2] = {0.0, 0.0};       // [rad/s]
};

class SerialBridge
{
public:
  SerialBridge() = default;
  ~SerialBridge();
  SerialBridge(const SerialBridge &) = delete;
  SerialBridge & operator=(const SerialBridge &) = delete;

  // 포트 열기 (실패하면 false, error() 에 이유)
  bool open(const std::string & port, int baud);
  bool is_open() const { return fd_ >= 0; }
  const std::string & error() const { return error_; }

  // 쌓인 바이트에서 완성된 상태 패킷을 모두 꺼내 가장 최근 것을 돌려줌 (없으면 nullopt)
  std::optional<RobotState> receive();

  // 명령 패킷 송신
  bool send_command(uint8_t seq, double wheel_l, double wheel_r, double arm_yaw, double arm_pitch);

private:
  int fd_ = -1;
  std::string error_;
  std::vector<uint8_t> buf_;
};

}  // namespace control
