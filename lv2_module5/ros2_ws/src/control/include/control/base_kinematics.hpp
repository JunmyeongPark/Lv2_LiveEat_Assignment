// Base Kinematics: (v, ω) → 좌/우 바퀴 속도
//   1) 명령 끊김(cmd_timeout) → 정지 목표
//   2) 속도 한계 + 가속도 한계 (급출발 / 급회전 방지)
//   3) (v, ω) → 좌/우 목표 바퀴 속도, 바퀴 한계를 넘으면 둘 다 같은 비율로 줄여 곡률 유지
//   속도 자체는 OpenCR 쪽 모터 PID 가 맞추고, 경로 오차는 상위(판단)가 실시간으로 고친다
#pragma once

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
};

// 한 주기 결과 (명령 + 기록용 중간값)
struct WheelCommand
{
  double left = 0.0, right = 0.0;   // OpenCR 로 보낼 바퀴 속도 [rad/s]
  double v_ref = 0.0, w_ref = 0.0;  // 가속도 제한을 거친 목표 (v, ω)
};

// 바퀴 IMU 로 계산한 몸통 상태
struct BaseOdom
{
  double v = 0.0;           // 몸 직진속도, 앞 +(m/s)
  double w = 0.0;           // 몸 회전속도, 반시계 +(rad/s)
  double yaw = 0.0;         // 몸 방향(rad), [-π, π]
  double yaw_total = 0.0;   // 몸 누적 방향(rad), ±π 에서 안 끊기고 계속 늘어남
  bool from_imu = false;    // true: IMU yaw, false: 엔코더 yaw
};


class BaseKinematics
{
public:
  explicit BaseKinematics(const BaseParams & p);

  // v, w: 받은 /cmd_vel,  fresh: cmd_timeout 안에 받은 명령인지,  dt: 실제 경과 시간 [s]
  WheelCommand step(double v, double w, bool fresh, double dt);
  // wheel_vel_l/r: 바퀴 속도 [rad/s], pos_l/r: 바퀴 적분 위치 [rad], imu_ok: IMU yaw 사용 가능, imu_yaw: IMU yaw [rad]
  BaseOdom odom(double wheel_vel_l, double wheel_vel_r, double pos_l, double pos_r, bool imu_ok, double imu_yaw);

private:
  BaseParams p_;
  double v_ref_ = 0.0, w_ref_ = 0.0;
  bool odom_started_ = false;
  double prev_pos_l_ = 0.0, prev_pos_r_ = 0.0;
  double prev_imu_yaw_ = 0.0;
  double yaw_total_ = 0.0;
};

}  // namespace control
