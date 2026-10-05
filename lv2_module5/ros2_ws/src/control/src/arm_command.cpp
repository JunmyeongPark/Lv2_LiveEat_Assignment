// Arm Command: 관절·속도 제한 적용
#include "control/arm_command.hpp"

#include <algorithm>

namespace control
{

static double clamp(double x, double lo, double hi) { return std::max(lo, std::min(hi, x)); }

ArmCommand::ArmCommand(const ArmParams & p)
: p_(p) {}

ArmAngles ArmCommand::hold(const ArmAngles & current)
{
  cmd_ = ArmAngles{
    clamp(current.yaw, -p_.yaw_limit, p_.yaw_limit),
    clamp(current.pitch, p_.pitch_min, p_.pitch_max)};
  return *cmd_;
}

ArmAngles ArmCommand::step(
  const std::optional<ArmAngles> & goal, const ArmAngles & current, double dt)
{
  if (!cmd_) {
    cmd_ = current;                      // 처음엔 현재 자세에서 시작 (튀지 않게)
  }
  if (!goal) {
    return *cmd_;                        // 목표 없으면 지금 명령 유지
  }
  // 1) 관절 한계
  const double yaw_goal = clamp(goal->yaw, -p_.yaw_limit, p_.yaw_limit);
  const double pitch_goal = clamp(goal->pitch, p_.pitch_min, p_.pitch_max);
  // 2) 속도 제한: 한 주기에 최대 max_rate·dt 만큼만 이동
  const double step = p_.max_rate * dt;
  cmd_->yaw += clamp(yaw_goal - cmd_->yaw, -step, step);
  cmd_->pitch += clamp(pitch_goal - cmd_->pitch, -step, step);
  return *cmd_;
}

}  // namespace control
