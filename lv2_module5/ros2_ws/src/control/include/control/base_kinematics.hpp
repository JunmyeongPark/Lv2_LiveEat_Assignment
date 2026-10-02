// Base Kinematics: (v, ω) → 좌/우 바퀴 속도
//   1) 명령 끊김(cmd_timeout) → 정지 목표
//   2) 속도 한계 + 가속도 한계 (급출발 / 급회전 방지)
//   3) (v, ω) → 좌/우 목표 바퀴 속도, 바퀴 한계를 넘으면 둘 다 같은 비율로 줄여 곡률 유지
//   4) 목표 속도를 적분한 "목표 바퀴 각도" 와 엔코더 각도를 비교해서 누적 밀림 보정
//      (속도 자체는 OpenCR 쪽 모터 PID 가 맞춰줌)
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
  double kp_ang = 1.0;              // 바퀴 각도 오차(누적 밀림) 보정 게인
  double max_ang_err = 0.5;         // 각도 오차 보정 상한 [rad]
};

// 한 주기 결과 (명령 + 기록용 중간값)
struct WheelCommand
{
  double left = 0.0, right = 0.0;          // OpenCR 로 보낼 바퀴 속도 [rad/s]
  double v_ref = 0.0, w_ref = 0.0;         // 가속도 제한을 거친 목표 (v, ω)
  double left_ref = 0.0, right_ref = 0.0;  // 보정 전 목표 바퀴 속도 [rad/s]
  double left_err = 0.0, right_err = 0.0;  // 바퀴 각도 오차 [rad]
};

class BaseKinematics
{
public:
  explicit BaseKinematics(const BaseParams & p);

  // v, w: 받은 /cmd_vel,  fresh: cmd_timeout 안에 받은 명령인지
  // th_left, th_right: 엔코더 바퀴 각도 [rad],  dt: 실제 경과 시간 [s]
  WheelCommand step(double v, double w, bool fresh, double th_left, double th_right, double dt);

private:
  BaseParams p_;
  bool started_ = false;
  double v_ref_ = 0.0, w_ref_ = 0.0;
  double th_ref_[2] = {0.0, 0.0};   // 목표 바퀴 각도 (목표 속도 적분)
};

}  // namespace control
