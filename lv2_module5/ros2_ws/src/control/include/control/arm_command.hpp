// Arm Command: 관절·속도 제한 적용
//   팔 목표 (yaw, pitch) → 1) 관절 한계로 자르기 (clamp) → 2) 한 주기 이동량 제한 (rate limit)
//   순서가 중요: 먼저 자르면 처음부터 "갈 수 있는 곳" 으로만 부드럽게 움직임.
//   반대로 하면 한계 밖 목표로 가다가 마지막 clamp 에서 "툭" 멈춤.
//   단위: 각도 rad, 속도 rad/s, dt 초 (실제 경과 시간)
#pragma once

#include <optional>

namespace control
{

struct ArmAngles
{
  double yaw = 0.0;
  double pitch = 0.0;
};

struct ArmParams
{
  double yaw_limit = 1.5708;    // 배선 꼬임 한계 ± [rad] (90°)
  double pitch_min = -0.5236;   // [rad] (-30°)
  double pitch_max = 1.0472;    // [rad] (60°)
  double max_rate = 2.0944;     // 팔 최대 속도 [rad/s] (120°/s)
};

class ArmCommand
{
public:
  explicit ArmCommand(const ArmParams & p);

  // goal: Planning 목표 (아직 없으면 nullopt), current: 실제 관절 각도 → 이번 주기 명령
  ArmAngles step(const std::optional<ArmAngles> & goal, const ArmAngles & current, double dt);

private:
  ArmParams p_;
  std::optional<ArmAngles> cmd_;   // 마지막으로 보낸 명령
};

}  // namespace control
