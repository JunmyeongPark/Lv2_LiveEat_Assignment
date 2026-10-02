// Base Kinematics: (v, ω) → 좌/우 바퀴 속도
#include "control/base_kinematics.hpp"

#include <algorithm>
#include <cmath>

namespace control
{

static double clamp(double x, double lo, double hi) { return std::max(lo, std::min(hi, x)); }

BaseKinematics::BaseKinematics(const BaseParams & p)
: p_(p) {}

WheelCommand BaseKinematics::step(
  double v, double w, bool fresh, double th_left, double th_right, double dt)
{
  WheelCommand out;
  if (!started_) {                       // 목표 각도는 현재 바퀴 각도에서 시작
    th_ref_[0] = th_left;
    th_ref_[1] = th_right;
    started_ = true;
  }

  // 1) 명령이 끊겼으면 정지 목표
  double v_goal = fresh ? v : 0.0;
  double w_goal = fresh ? w : 0.0;

  // 2) 속도 한계 + 가속도 한계
  v_goal = clamp(v_goal, -p_.max_v, p_.max_v);
  w_goal = clamp(w_goal, -p_.max_w, p_.max_w);
  v_ref_ += clamp(v_goal - v_ref_, -p_.max_acc_v * dt, p_.max_acc_v * dt);
  w_ref_ += clamp(w_goal - w_ref_, -p_.max_acc_w * dt, p_.max_acc_w * dt);

  // 3) (v, ω) → 좌/우 목표 바퀴 속도
  double wl = (v_ref_ - w_ref_ * p_.wheel_separation / 2.0) / p_.wheel_radius;
  double wr = (v_ref_ + w_ref_ * p_.wheel_separation / 2.0) / p_.wheel_radius;
  const double k = std::max({1.0, std::abs(wl) / p_.max_wheel, std::abs(wr) / p_.max_wheel});
  wl /= k;                               // 바퀴 한계를 넘으면 둘 다 같은 비율로 줄여 곡률 유지
  wr /= k;

  // 4) 누적 밀림 보정: 목표 각도 vs 엔코더 각도
  th_ref_[0] += wl * dt;
  th_ref_[1] += wr * dt;
  const double el = clamp(th_ref_[0] - th_left, -p_.max_ang_err, p_.max_ang_err);
  const double er = clamp(th_ref_[1] - th_right, -p_.max_ang_err, p_.max_ang_err);
  th_ref_[0] = th_left + el;             // 오차가 상한 이상 쌓이지 않게 목표를 당겨둠
  th_ref_[1] = th_right + er;
  double cl = clamp(wl + p_.kp_ang * el, -p_.max_wheel, p_.max_wheel);
  double cr = clamp(wr + p_.kp_ang * er, -p_.max_wheel, p_.max_wheel);
  if (!fresh && std::abs(v_ref_) < 1e-3 && std::abs(w_ref_) < 1e-3) {
    cl = cr = 0.0;                       // 명령 끊김 + 감속 완료 → 확실히 정지
    th_ref_[0] = th_left;
    th_ref_[1] = th_right;
  }

  out.left = cl;
  out.right = cr;
  out.v_ref = v_ref_;
  out.w_ref = w_ref_;
  out.left_ref = wl;
  out.right_ref = wr;
  out.left_err = el;
  out.right_err = er;
  return out;
}

}  // namespace control
