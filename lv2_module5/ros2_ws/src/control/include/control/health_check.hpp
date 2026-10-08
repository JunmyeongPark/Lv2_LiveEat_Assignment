// Health Check: OpenCR, IMU, 모터 상태를 OK / WARN / ERROR / STALE 로 판단(ROS 와 무관)
//   control_master 가 재료를 모아 넘기고, 결과를 diagnostic_msgs/DiagonosticStatus 로 발행한다
#pragma once

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace control
{


// 경고등 색. 숫자는 DianosticStatus 의 level 과 같게 맞춤
enum class HealthLevel : uint8_t { OK = 0, WARN = 1, ERROR =2, STALE = 3 };

// 판단 결과 (경고등 하나)
