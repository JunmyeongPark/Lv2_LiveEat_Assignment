// Base Kinematics: (v, ω) → 좌/우 바퀴 속도
#include "control/base_kinematics.hpp"

#include <algorithm>
#include <cmath>

namespace control
{

static double clamp(double x, double lo, double hi) { return std::max(lo, std::min(hi, x)); }

BaseKinematics::BaseKinematics(const BaseParams & p)
: p_(p) {}

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
  double wl = (v_ref_ - w_ref_ * p_.wheel_separation / 2.0) / p_.wheel_radius;
  double wr = (v_ref_ + w_ref_ * p_.wheel_separation / 2.0) / p_.wheel_radius;
  const double k = std::max({1.0, std::abs(wl) / p_.max_wheel, std::abs(wr) / p_.max_wheel});
  wl /= k;                               // 바퀴 한계를 넘으면 둘 다 같은 비율로 줄여 곡률 유지
  wr /= k;

  out.left = wl;
  out.right = wr;
  out.v_ref = v_ref_;
  out.w_ref = w_ref_;
  return out;
}

BaseOdom BaseKinematics::odom(double wheel_vel_l, double wheel_vel_r, double pos_l, double pos_r, bool imu_ok, double imu_yaw)
{
  BaseOdom out;
  
  if (!odom_started_) {
    prev_pos_l_ = pos_l;
    prev_pos_r_ = pos_r;
    prev_imu_yaw_ = imu_yaw;
    odom_started_ = true;
  }
  // 엔코더 yaw
  const double dpos_l = pos_l - prev_pos_l_;
  const double dpos_r = pos_r - prev_pos_r_;
  const double dyaw_wheel = (dpos_r - dpos_l) * p_.wheel_radius / p_.wheel_separation;

  // IMU yaw
  double dyaw_imu = imu_yaw - prev_imu_yaw_;
  if (dyaw_imu > M_PI) {
    dyaw_imu -= M_PI * 2.0;
  } else if (dyaw_imu < -M_PI) {
    dyaw_imu += M_PI * 2.0;
  }

  // IMU 사용 가능하면 IMU yaw 를 쓰고, 아니면 엔코더 yaw 를 씀
  const double dyaw = imu_ok ? dyaw_imu : dyaw_wheel;

  // 직진 속도, 회전 속도
  out.v = (wheel_vel_r + wheel_vel_l) * p_.wheel_radius / 2.0;
  out.w = (wheel_vel_r - wheel_vel_l) * p_.wheel_radius / p_.wheel_separation;

  // 몸 방향
  yaw_total_ += dyaw;   // ±π 에서 안 끊기고 계속 누적
  out.yaw = std::atan2(std::sin(yaw_total_), std::cos(yaw_total_));
  out.yaw_total = yaw_total_;

  out.from_imu = imu_ok;

  prev_pos_l_ = pos_l;
  prev_pos_r_ = pos_r;
  prev_imu_yaw_ = imu_yaw;

  return out;
}

}  // namespace control