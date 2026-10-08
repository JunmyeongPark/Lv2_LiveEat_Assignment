// Base Kinematics: (v, ω) → 좌/우 바퀴 속도
#include "control/base_kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace control
{

static double clamp(double x, double lo, double hi) { return std::max(lo, std::min(hi, x)); }

// ---- 회전 미끄러짐 보정 k(v, ω) ----
//   k = 실제 회전(IMU) ÷ 엔코더 회전. 2026-10-06 솜털 바닥 실측 36회, 반지름 0.033 · 간격 0.29 기준
//   칸 사이: 같은 v 줄에서 |ω| 로 선형 보간 → 이웃 두 v 줄 사이를 v 로 선형 보간. 표 밖·|ω|<0.4 는 끝 값 유지
//   다른 바닥이면 calib_run.py 로 다시 재서 아래 표만 바꾸기
namespace
{
struct SlipRow
{
  double v;
  std::vector<double> w, k;   // |ω| 오름차순
};
const std::vector<SlipRow> SLIP_CCW = {   // 반시계 (ω > 0)
  {-0.10, {0.4, 0.8}, {0.9386, 0.9491}},
  {0.00, {0.4, 0.8, 1.2, 1.6}, {0.9656, 0.9551, 0.9733, 0.9443}},
  {0.10, {0.4, 0.8}, {0.9303, 0.9311}},
  {0.15, {0.4}, {0.9199}},
};
const std::vector<SlipRow> SLIP_CW = {    // 시계 (ω < 0)
  {-0.10, {0.4, 0.8}, {0.9067, 0.9247}},
  {0.00, {0.4, 0.8, 1.2, 1.6}, {0.9245, 0.9241, 0.9408, 0.9313}},
  {0.10, {0.4, 0.8}, {0.8791, 0.8980}},
  {0.15, {0.4}, {0.8739}},
};
constexpr double SLIP_REF_RADIUS = 0.033;
constexpr double SLIP_REF_SEPARATION = 0.29;

double slip_along_w(const SlipRow & r, double aw)
{
  if (aw <= r.w.front()) {return r.k.front();}
  if (aw >= r.w.back()) {return r.k.back();}
  for (size_t i = 1; i < r.w.size(); ++i) {
    if (aw <= r.w[i]) {
      const double t = (aw - r.w[i - 1]) / (r.w[i] - r.w[i - 1]);
      return r.k[i - 1] + t * (r.k[i] - r.k[i - 1]);
    }
  }
  return r.k.back();
}

double slip_k(double v, double w)
{
  const std::vector<SlipRow> & rows = (w >= 0.0) ? SLIP_CCW : SLIP_CW;
  const double aw = std::abs(w);
  if (v <= rows.front().v) {return slip_along_w(rows.front(), aw);}
  if (v >= rows.back().v) {return slip_along_w(rows.back(), aw);}
  for (size_t i = 1; i < rows.size(); ++i) {
    if (v <= rows[i].v) {
      const double t = (v - rows[i - 1].v) / (rows[i].v - rows[i - 1].v);
      const double k0 = slip_along_w(rows[i - 1], aw);
      return k0 + t * (slip_along_w(rows[i], aw) - k0);
    }
  }
  return slip_along_w(rows.back(), aw);
}
}  // namespace

BaseKinematics::BaseKinematics(const BaseParams & p)
: p_(p) {}

// 실제 회전 = 배율 × 이 코드의 엔코더 회전 (보정 끄면 1)
//   k 는 반지름 0.033 · 간격 0.29 기준 → 지금 설정(반지름 r, 간격 s)으로 환산: × (0.033 ÷ r) × (s ÷ 0.29)
double BaseKinematics::slip_scale(double v, double w) const
{
  if (!p_.slip_correction) {
    return 1.0;
  }
  return slip_k(v, w) * (SLIP_REF_RADIUS / p_.wheel_radius) * (p_.wheel_separation / SLIP_REF_SEPARATION);
}

WheelCommand BaseKinematics::step(double v, double w, bool fresh, double dt)

{
  WheelCommand out;

  // 1) 명령이 끊겼으면 정지 목표
  double v_goal = fresh ? v : 0.0;
  double w_goal = fresh ? w : 0.0;

  // 2) 속도 한계 + 가속도 한계 (끊기면 이 한계대로 감속해서 0 에 도달)
  v_goal = clamp(v_goal, -p_.max_v, p_.max_v);
  w_goal = clamp(w_goal, -p_.max_w, p_.max_w);
  v_ref_ += clamp(v_goal - v_ref_, -p_.max_acc_v * dt, p_.max_acc_v * dt);
  w_ref_ += clamp(w_goal - w_ref_, -p_.max_acc_w * dt, p_.max_acc_w * dt);

  // 3) (v, ω) → 좌/우 목표 바퀴 속도
  //    미끄러짐 보정: 바퀴 회전 차이를 ÷ 배율 만큼 크게 줘야 목표 ω 가 실제로 나옴
  const double w_wheel = w_ref_ / slip_scale(v_ref_, w_ref_);
  double wl = (v_ref_ - w_wheel * p_.wheel_separation / 2.0) / p_.wheel_radius;
  double wr = (v_ref_ + w_wheel * p_.wheel_separation / 2.0) / p_.wheel_radius;
  wr *= p_.right_wheel_gain;             // 직진 보정: 오른쪽 바퀴만 배율 (바퀴 한계 계산 전에 적용)
  const double k = std::max({1.0, std::abs(wl) / p_.max_wheel, std::abs(wr) / p_.max_wheel});
  wl /= k;                               // 바퀴 한계를 넘으면 둘 다 같은 비율로 줄여 곡률 유지
  wr /= k;

  out.left = wl;
  out.right = wr;
  out.v_ref = v_ref_;
  out.w_ref = w_ref_;
  return out;
}

BaseOdom BaseKinematics::odom(
  double wheel_vel_l, double wheel_vel_r, double pos_l, double pos_r,
  std::optional<double> imu_yaw)
{
  BaseOdom out;

  if (!odom_started_) {
    prev_pos_l_ = pos_l;
    prev_pos_r_ = pos_r;
    odom_started_ = true;
  }
  // 엔코더 yaw 변화: 이번 주기에 두 바퀴가 돈 양의 차이
  const double dpos_l = pos_l - prev_pos_l_;
  const double dpos_r = pos_r - prev_pos_r_;
  double dyaw = (dpos_r - dpos_l) * p_.wheel_radius / p_.wheel_separation;

  // 직진 속도, 회전 속도
  out.v = (wheel_vel_r + wheel_vel_l) * p_.wheel_radius / 2.0;
  out.w = (wheel_vel_r - wheel_vel_l) * p_.wheel_radius / p_.wheel_separation;

  // 미끄러짐 보정: 엔코더 회전 × 배율 = 실제 회전 (보정 끄면 배율 1)
  //   표는 실제 회전으로 찾아야 하므로, 엔코더 회전으로 한 번 보정한 값으로 다시 찾음 (명령 쪽과 같은 자리)
  const double s = slip_scale(out.v, out.w * slip_scale(out.v, out.w));
  out.w *= s;
  dyaw *= s;

  // 몸 방향: 누적한 뒤 [-π, π] 로 (IMU yaw 와 같은 형식)
  //   IMU 가 있으면 IMU 값으로 맞춤, 없으면 마지막 값에서 엔코더 회전만 더해 이어감
  if (p_.yaw_follow_imu && imu_yaw) {
    yaw_total_ = *imu_yaw;
  } else {
    yaw_total_ += dyaw;
  }
  out.yaw = std::atan2(std::sin(yaw_total_), std::cos(yaw_total_));

  prev_pos_l_ = pos_l;
  prev_pos_r_ = pos_r;

  return out;
}

}  // namespace control