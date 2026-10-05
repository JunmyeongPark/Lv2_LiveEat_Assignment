// control_master: 고정 주기 ① Serial RX → ② Base Kinematics → ③ Arm Command → ④ Serial TX 호출
//
// [입력 토픽]  (planning_master 에서 옴)
//   /cmd_vel       geometry_msgs/Twist          linear.x = v [m/s], angular.z = ω [rad/s]
//   /arm/command   std_msgs/Float64MultiArray   [yaw, pitch] [rad]
// [출력 토픽]
//   (planning_master 로)
//   /control/imu      sensor_msgs/Imu          IMU 원본: yaw (±π, 쿼터니언), 각속도 z
//   /control/odom     nav_msgs/Odometry        엔코더 기반: v [m/s], ω [rad/s], yaw (±π, 쿼터니언, 시작 0)
//                                              yaw 형식은 /control/imu 와 같게 맞춤
//                     (IMU · 엔코더 중 어느 yaw 를 쓸지는 판단부에서 정함)
//   (디버깅 · bag 기록용 원본)
//   /joint_states     sensor_msgs/JointState   wheel_left_joint, wheel_right_joint,
//                                              arm_yaw_joint, arm_pitch_joint
// [실행]
//   ros2 run control control_master --ros-args --params-file lv2_module5/config/control.yaml

#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <optional>
#include <string>

#include "control/arm_command.hpp"
#include "control/base_kinematics.hpp"
#include "control/serial_bridge.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

namespace control
{

constexpr double DEG = M_PI / 180.0;

class ControlMaster : public rclcpp::Node
{
public:
  ControlMaster()
  : Node("control_master"),
    base_(load_base_params()),
    arm_(load_arm_params())
  {
    // ---------- 설정값 (config/control.yaml) ----------
    rate_hz_ = declare_parameter("rate_hz", 50.0);
    port_ = declare_parameter("port", std::string("/dev/ttyACM0"));
    baud_ = declare_parameter("baud", 1000000);
    cmd_timeout_ = declare_parameter("cmd_timeout_s", 0.3);   // 이 시간 동안 /cmd_vel 없으면 바퀴 정지
    motor_enable_ = declare_parameter("motor_enable", true);  // false: 상태는 받고, 모터에는 정지만 보냄
    const auto log_dir = declare_parameter("log_dir", std::string("results/logs"));  // "" 이면 기록 안 함

    // ---------- OpenCR 연결 ----------
    if (!serial_.open(port_, static_cast<int>(baud_))) {
      throw std::runtime_error(serial_.error());
    }

    // ---------- 토픽 ----------
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr m) {
        cmd_v_ = m->linear.x;
        cmd_w_ = m->angular.z;
        cmd_vel_t_ = now();
      });
    arm_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/arm/command", 10, [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr m) {
        if (m->data.size() >= 2) {
          arm_goal_ = ArmAngles{m->data[0], m->data[1]};
        }
      });
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/control/imu", 10);
    js_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/control/odom", 10);

    // ---------- 기록 ----------
    if (!log_dir.empty()) {
      open_log(log_dir);
    }

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_hz_), [this]() {step();});
    RCLCPP_INFO(
      get_logger(), "포트=%s, 주기=%.0fHz, 모터=%s, 기록=%s", port_.c_str(), rate_hz_,
      motor_enable_ ? "켜짐" : "꺼짐", log_path_.empty() ? "없음" : log_path_.c_str());
  }

  // 종료할 때 바퀴 정지 명령을 여러 번 보냄 (응답 확인이 없으므로, 펌웨어 watchdog 이 최종 백업)
  void stop()
  {
    wheel_ = WheelCommand{};
    for (int i = 0; i < 5; ++i) {
      serial_tx();
    }
    if (log_.is_open()) {
      log_.close();
    }
    RCLCPP_INFO(get_logger(), "정지, 기록: %s", log_path_.empty() ? "없음" : log_path_.c_str());
  }

private:
  BaseParams load_base_params()
  {
    BaseParams p;
    p.wheel_radius = declare_parameter("wheel_radius", p.wheel_radius);
    p.wheel_separation = declare_parameter("wheel_separation", p.wheel_separation);
    p.max_v = declare_parameter("max_v", p.max_v);
    p.max_w = declare_parameter("max_w", p.max_w);
    p.max_wheel = declare_parameter("max_wheel", p.max_wheel);
    p.max_acc_v = declare_parameter("max_acc_v", p.max_acc_v);
    p.max_acc_w = declare_parameter("max_acc_w", p.max_acc_w);
    return p;
  }

  ArmParams load_arm_params()
  {
    ArmParams p;   // config 에는 읽기 쉽게 도(deg) 로 적고, 안에서는 rad 로 씀
    p.yaw_limit = declare_parameter("arm_yaw_limit_deg", 120.0) * DEG;
    p.pitch_min = declare_parameter("arm_pitch_min_deg", -80.0) * DEG;
    p.pitch_max = declare_parameter("arm_pitch_max_deg", 85.0) * DEG;
    p.max_rate = declare_parameter("arm_max_dps", 120.0) * DEG;
    return p;
  }

  void open_log(const std::string & dir)
  {
    std::filesystem::create_directories(dir);
    char stamp[32];
    const std::time_t tt = std::time(nullptr);
    std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", std::localtime(&tt));
    log_path_ = dir + "/control_" + stamp + ".csv";
    log_.open(log_path_);
    log_ << "t,dt,loop_ms,v_ref,w_ref,wL_ref,wL,wR_ref,wR,"
            "yaw_cmd,yaw,pitch_cmd,pitch,imu_yaw,odom_v,odom_w,odom_yaw\n";
    log_ << std::fixed << std::setprecision(4);
  }

  // ---------- 매 주기 ----------
  void step()
  {
    const auto loop_start = std::chrono::steady_clock::now();
    const rclcpp::Time t = now();
    const double dt = last_t_ ? (t - *last_t_).seconds() : 1.0 / rate_hz_;   // 실제 dt 사용
    last_t_ = t;
    if (dt <= 0.0) {
      return;
    }

    serial_rx();                      // ①
    if (!state_) {
      serial_tx();                    // 상태가 아직 없어도 정지 명령(heartbeat)은 보냄
      return;
    }
    const bool fresh = cmd_vel_t_ && (t - *cmd_vel_t_).seconds() < cmd_timeout_;
    wheel_ = base_.step(cmd_v_, cmd_w_, fresh, dt);              // ②
    arm_cmd_ = arm_.step(                                        // ③
      arm_goal_, ArmAngles{state_->arm_pos[0], state_->arm_pos[1]}, dt);
    serial_tx();                      // ④

    if (log_.is_open()) {
      const double loop_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - loop_start).count();
      const auto & s = *state_;
      log_ << t.seconds() << ',' << dt << ',' << loop_ms << ','
           << wheel_.v_ref << ',' << wheel_.w_ref << ','
           << wheel_.left << ',' << s.wheel_vel[0] << ','
           << wheel_.right << ',' << s.wheel_vel[1] << ','
           << arm_cmd_->yaw << ',' << s.arm_pos[0] << ','
           << arm_cmd_->pitch << ',' << s.arm_pos[1] << ','
           << s.imu_yaw << ',' << odom_.v << ',' << odom_.w << ','
           << odom_.yaw << '\n';
    }
  }

  // ① Serial RX ----------------------------------------------------------
  void serial_rx()
  {
    const auto st = serial_.receive();
    if (!st) {
      if (state_t_ && (now() - *state_t_).seconds() > 0.5) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "OpenCR 상태 수신 끊김");
      }
      return;
    }
    state_ = st;
    state_t_ = now();
    const auto stamp = now();

    sensor_msgs::msg::Imu imu;
    imu.header.stamp = stamp;
    imu.header.frame_id = "imu_link";
    imu.orientation.z = std::sin(st->imu_yaw / 2.0);    // yaw 만 있는 쿼터니언
    imu.orientation.w = std::cos(st->imu_yaw / 2.0);
    imu.angular_velocity.z = st->imu_gyro_z;
    imu_pub_->publish(imu);

    sensor_msgs::msg::JointState js;
    js.header.stamp = stamp;
    js.name = {"wheel_left_joint", "wheel_right_joint", "arm_yaw_joint", "arm_pitch_joint"};
    js.position = {st->wheel_pos[0], st->wheel_pos[1], st->arm_pos[0], st->arm_pos[1]};
    js.velocity = {st->wheel_vel[0], st->wheel_vel[1], st->arm_vel[0], st->arm_vel[1]};
    js_pub_->publish(js);

    const BaseOdom o = base_.odom(st->wheel_vel[0], st->wheel_vel[1], st->wheel_pos[0], st->wheel_pos[1]);
    odom_ = o;
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = o.v;   // o 의 직진 속도
    odom.twist.twist.angular.z = o.w;  // o 의 회전속도
    odom.pose.pose.orientation.z = std::sin(o.yaw / 2.0);  // 방향 yaw (쿼터니언)
    odom.pose.pose.orientation.w = std::cos(o.yaw / 2.0); 
    odom_pub_->publish(odom);

  }

  // ④ Serial TX ----------------------------------------------------------
  void serial_tx()
  {
    double wl = wheel_.left, wr = wheel_.right;
    const ArmAngles arm = arm_cmd_.value_or(ArmAngles{});
    uint8_t seq = seq_;
    if (!arm_cmd_ || !motor_enable_) {
      // 팔 상태를 아직 모르거나 모터 꺼짐: seq=0 → 펌웨어가 팔 값을 무시
      seq = 0;
    }
    if (!motor_enable_) {
      wl = wr = 0.0;
    }
    serial_.send_command(seq, wl, wr, arm.yaw, arm.pitch);
    seq_ = static_cast<uint8_t>(seq_ % 255 + 1);   // 1 ~ 255 반복, 0 은 "팔 무시" 용도로 남겨둠
  }

  // 설정
  double rate_hz_;
  std::string port_;
  int64_t baud_;
  double cmd_timeout_;
  bool motor_enable_;

  // 계산 모듈
  BaseKinematics base_;
  ArmCommand arm_;
  SerialBridge serial_;

  // 상태
  double cmd_v_ = 0.0, cmd_w_ = 0.0;               // 마지막으로 받은 (v, ω)
  std::optional<rclcpp::Time> cmd_vel_t_;          // 받은 시각
  std::optional<ArmAngles> arm_goal_;              // 마지막으로 받은 팔 목표
  std::optional<RobotState> state_;                // OpenCR 에서 받은 최신 상태
  std::optional<rclcpp::Time> state_t_;
  std::optional<rclcpp::Time> last_t_;
  WheelCommand wheel_;
  BaseOdom odom_;                                  // 최신 odom 결과 (CSV 기록용)
  std::optional<ArmAngles> arm_cmd_;
  uint8_t seq_ = 1;

  // ROS
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr arm_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // 기록
  std::ofstream log_;
  std::string log_path_;
};

}  // namespace control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<control::ControlMaster> node;
  try {
    node = std::make_shared<control::ControlMaster>();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("control_master"), "시작 실패: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::spin(node);   // Ctrl+C 까지 타이머 · 콜백 실행
  node->stop();
  rclcpp::shutdown();
  return 0;
}