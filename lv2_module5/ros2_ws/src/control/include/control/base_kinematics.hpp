// Base Kinematics: (v, ω) → 좌/우 바퀴 속도
//   1) 명령 끊김(cmd_timeout) → 정지 목표
//   2) 속도 한계 + 가속도 한계 (급출발 / 급회전 방지)
//   3) (v, ω) → 좌/우 목표 바퀴 속도, 바퀴 한계를 넘으면 둘 다 같은 비율로 줄여 곡률 유지
//   속도 자체는 OpenCR 쪽 모터 PID 가 맞추고, 경로 오차는 상위(판단)가 실시간으로 고친다
#pragma once

#include <optional>

namespace control
{

struct BaseParams
{
  double wheel_radius = 0.033;      // 바퀴 반지름 [m]
  double wheel_separation = 0.287;  // 좌우 바퀴 사이 거리 [m]
  double max_v = 0.26;              // 직진 속도 한계 [m/s]
  double max_w = 1.82;              // 회전 속도 한계 [rad/s]
  double max_wheel = 7.8;           // 바퀴 속도 한계 [rad/s]
  double max_acc_v = 0.5;           // 직진 가속도 한계 [m/s^2]
  double max_acc_w = 3.0;           // 회전 가속도 한계 [rad/s^2]
  double right_wheel_gain = 1.0;    // 오른쪽 바퀴 명령 배율 (직진 보정, 1.0 = 보정 없음)
  bool slip_correction = true;      // 회전 미끄러짐 보정 k(v, ω) 사용 (base_kinematics.cpp 의 표)
  bool yaw_follow_imu = true;       // IMU 가 있으면 엔코더 yaw 를 IMU 에 맞추고, 끊기면 그 값에서 엔코더로 이어감
};

// 한 주기 결과 (명령 + 기록용 중간값)
struct WheelCommand
{
  double left = 0.0, right = 0.0;   // OpenCR 로 보낼 바퀴 속도 [rad/s]
  double v_ref = 0.0, w_ref = 0.0;  // 가속도 제한을 거친 목표 (v, ω)
};

// 엔코더(바퀴)로 계산한 몸통 상태
//   yaw: IMU 가 있으면 IMU 값에 맞춤 (yaw_follow_imu), 끊기면 마지막 값에서 엔코더 회전만 더해 유지
//   IMU 는 /control/imu 로도 따로 보낸다. 어느 쪽을 믿을지는 판단부가 정함
struct BaseOdom
{
  double v = 0.0;        // 몸 직진속도, 앞 +(m/s)
  double w = 0.0;        // 몸 회전속도, 반시계 +(rad/s)
  double yaw = 0.0;      // 몸 방향(rad), [-π, π]. IMU 로 보정한 값 (/control/odom_yaw_deg)
};


class BaseKinematics
{
public:
  explicit BaseKinematics(const BaseParams & p);

  void stop() { v_ref_ = 0.0; w_ref_ = 0.0; }

  // v, w: 받은 /cmd_vel,  fresh: cmd_timeout 안에 받은 명령인지,  dt: 실제 경과 시간 [s]
  WheelCommand step(double v, double w, bool fresh, double dt);

  // wheel_vel_l/r: 바퀴 속도 [rad/s], pos_l/r: 바퀴 누적 각도 [rad]
  // imu_yaw: 쓸 수 있는 IMU yaw [rad] (없으면 nullopt → 엔코더로 이어감)
  BaseOdom odom(
    double wheel_vel_l, double wheel_vel_r, double pos_l, double pos_r,
    std::optional<double> imu_yaw = std::nullopt);

private:
  double slip_scale(double v, double w) const;   // 회전 미끄러짐 배율 (base_kinematics.cpp)

  BaseParams p_;
  double v_ref_ = 0.0, w_ref_ = 0.0;
  bool odom_started_ = false;
  double prev_pos_l_ = 0.0, prev_pos_r_ = 0.0;
  double yaw_total_ = 0.0;   // IMU 로 보정한 yaw (끊기면 엔코더로 누적, ±π 에서 안 끊김)
};

}  // namespace control
