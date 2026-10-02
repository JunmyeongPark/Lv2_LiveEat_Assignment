// Serial RX/TX: IMU · 관절 상태 수신 → /imu, /joint_states 발행 / 명령 송신 (0 포함 heartbeat)
#include "control/serial_bridge.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>

namespace control
{

namespace
{
constexpr uint8_t HEADER[2] = {0xAA, 0x55};
constexpr uint8_t TYPE_CMD = 0x01;
constexpr uint8_t TYPE_STATE = 0x02;
constexpr size_t CMD_LEN = 1 + 4 * 4;                    // seq + float 4개
constexpr size_t STATE_LEN = 4 + 4 * 10;                 // 시간 ms + float 10개
constexpr size_t STATE_FRAME = 2 + 1 + STATE_LEN + 1;    // 헤더 + 종류 + 데이터 + 체크섬

uint8_t checksum(const uint8_t * p, size_t n)
{
  uint8_t s = 0;
  for (size_t i = 0; i < n; ++i) {
    s += p[i];
  }
  return s;
}

speed_t to_speed(int baud)
{
  switch (baud) {
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    case 1000000: return B1000000;
    case 2000000: return B2000000;
    default: return B0;
  }
}
}  // namespace

SerialBridge::~SerialBridge()
{
  if (fd_ >= 0) {
    ::close(fd_);
  }
}

bool SerialBridge::open(const std::string & port, int baud)
{
  const speed_t speed = to_speed(baud);
  if (speed == B0) {
    error_ = "지원하지 않는 baud: " + std::to_string(baud);
    return false;
  }
  fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);   // 읽을 게 없으면 바로 돌아옴
  if (fd_ < 0) {
    error_ = port + " 열기 실패: " + std::strerror(errno);
    return false;
  }
  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    error_ = std::string("tcgetattr 실패: ") + std::strerror(errno);
    ::close(fd_);
    fd_ = -1;
    return false;
  }
  cfmakeraw(&tty);                   // 바이트를 그대로 주고받음 (줄바꿈 변환 등 없음)
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;
  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    error_ = std::string("tcsetattr 실패: ") + std::strerror(errno);
    ::close(fd_);
    fd_ = -1;
    return false;
  }
  tcflush(fd_, TCIOFLUSH);           // 이전에 쌓인 바이트 버림
  return true;
}

std::optional<RobotState> SerialBridge::receive()
{
  if (fd_ < 0) {
    return std::nullopt;
  }
  uint8_t tmp[512];
  ssize_t n;
  while ((n = ::read(fd_, tmp, sizeof(tmp))) > 0) {
    buf_.insert(buf_.end(), tmp, tmp + n);
  }

  std::optional<RobotState> latest;
  size_t i = 0;
  while (true) {
    // 헤더 찾기
    auto it = std::search(buf_.begin() + i, buf_.end(), HEADER, HEADER + 2);
    if (it == buf_.end()) {
      i = buf_.size() > 0 ? buf_.size() - 1 : 0;   // 헤더 앞부분(0xAA)일 수 있는 마지막 1바이트만 남김
      break;
    }
    i = static_cast<size_t>(it - buf_.begin());
    if (buf_.size() - i < STATE_FRAME) {
      break;                                       // 아직 덜 들어옴
    }
    const uint8_t * body = buf_.data() + i + 2;    // 종류 + 데이터
    if (body[0] == TYPE_STATE && checksum(body, 1 + STATE_LEN) == body[1 + STATE_LEN]) {
      RobotState s;
      float f[10];
      std::memcpy(&s.ms, body + 1, 4);
      std::memcpy(f, body + 5, 40);
      s.imu_yaw = f[0];
      s.imu_gyro_z = f[1];
      s.wheel_pos[0] = f[2];
      s.wheel_pos[1] = f[3];
      s.wheel_vel[0] = f[4];
      s.wheel_vel[1] = f[5];
      s.arm_pos[0] = f[6];
      s.arm_pos[1] = f[7];
      s.arm_vel[0] = f[8];
      s.arm_vel[1] = f[9];
      latest = s;
      i += STATE_FRAME;
    } else {
      i += 1;                                      // 깨진 패킷: 한 바이트 넘기고 다시 찾기
    }
  }
  buf_.erase(buf_.begin(), buf_.begin() + std::min(i, buf_.size()));
  return latest;
}

bool SerialBridge::send_command(
  uint8_t seq, double wheel_l, double wheel_r, double arm_yaw, double arm_pitch)
{
  if (fd_ < 0) {
    return false;
  }
  uint8_t pkt[2 + 1 + CMD_LEN + 1];
  pkt[0] = HEADER[0];
  pkt[1] = HEADER[1];
  pkt[2] = TYPE_CMD;
  pkt[3] = seq;
  const float f[4] = {
    static_cast<float>(wheel_l), static_cast<float>(wheel_r),
    static_cast<float>(arm_yaw), static_cast<float>(arm_pitch)};
  std::memcpy(pkt + 4, f, 16);                     // 라즈베리파이(arm64)는 리틀 엔디언
  pkt[sizeof(pkt) - 1] = checksum(pkt + 2, 1 + CMD_LEN);
  return ::write(fd_, pkt, sizeof(pkt)) == static_cast<ssize_t>(sizeof(pkt));
}

}  // namespace control
