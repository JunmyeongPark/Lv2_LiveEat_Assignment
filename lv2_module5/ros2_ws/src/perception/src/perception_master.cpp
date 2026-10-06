// perception_master: 새 프레임마다 ① Detector → ② Depth Extractor → (③ IMU · Joint Reader) 호출
//
// 출력 /detection (geometry_msgs/PointStamped)
//   x = e_x, y = e_y  (정규화 중심 오차, 오른쪽·아래가 +)
//   z = depth [m]     검출 + depth 성공 (min_depth ~ max_depth, 기본 0.2 ~ 3.0 m)
//   z = 0             미검출 (매 프레임 발행 — 발제 규약: "정상 영상의 미검출은 z=0 발행")
//                     또는 검출은 됐지만 depth 실패 (범위 밖·유효 픽셀 부족)
//   header.stamp      원본 영상 시각 유지
//
// 진단 /perception/camera_health (diagnostic_msgs/DiagnosticStatus, health_rate_hz 로 계속 발행)
//   ERROR  camera USB disconnected   color·depth 끊김 + USB 에 RealSense(8086:usb_product_id) 없음 → 선 빠짐
//          camera driver not running color·depth 끊김 + 토픽 발행자 0 (realsense 노드 없음)
//          frames stopped            color·depth 끊김, USB·드라이버는 있음 (드라이버 멈춤)
//          no color / no depth       한쪽만 끊김
//          color frozen / depth frozen
//                                    메시지는 오는데 stamp 가 안 늘거나, color 내용이 frozen_frames 연속 동일
//          sync fail                 둘 다 오는데 color·depth 짝이 안 맞음 (sync_slop 초과)
//          exception: ...            영상 변환 · 검출 중 예외
//   OK     fps <동기화 프레임 수/초>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <algorithm>
#include <memory>
#include <optional>
#include <string>

#include <cv_bridge/cv_bridge.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <message_filters/synchronizer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "perception/depth_extractor.hpp"
#include "perception/detector.hpp"

using sensor_msgs::msg::Image;
using diagnostic_msgs::msg::DiagnosticStatus;
using SyncPolicy = message_filters::sync_policies::ApproximateTime<Image, Image>;

// message_filters 버전 차이 흡수: 라즈베리파이(Ubuntu 26 / 신버전)는 rclcpp::QoS 를 받고,
// Jazzy(PC)는 rmw_qos_profile_t 만 받는다. 컴파일 시 되는 쪽을 자동 선택.
template <class Sub, class N>
auto subscribe_sensor(Sub& sub, N* node, const std::string& topic, int)
    -> decltype(sub.subscribe(node, topic, rclcpp::QoS(1)), void()) {
  sub.subscribe(node, topic, rclcpp::SensorDataQoS());
}
template <class Sub, class N>
void subscribe_sensor(Sub& sub, N* node, const std::string& topic, long) {
  sub.subscribe(node, topic, rclcpp::SensorDataQoS().get_rmw_qos_profile());
}

class PerceptionMaster : public rclcpp::Node {
 public:
  PerceptionMaster() : Node("perception_master") {
    perception::DetectorConfig dc;
    dc.backend = declare_parameter("backend", std::string("ncnn"));
    dc.ncnn_param = declare_parameter("model_param", std::string(""));
    dc.ncnn_bin = declare_parameter("model_bin", std::string(""));
    dc.ncnn_input = declare_parameter("ncnn_input", std::string("in0"));
    dc.ncnn_output = declare_parameter("ncnn_output", std::string("out0"));
    dc.onnx_model = declare_parameter("model_onnx", std::string(""));
    dc.input_w = declare_parameter("input_width", 320);
    dc.input_h = declare_parameter("input_height", 320);
    dc.conf_threshold = static_cast<float>(declare_parameter("conf_threshold", 0.25));
    dc.num_threads = declare_parameter("num_threads", 4);
    dc.output_format = declare_parameter("output_format", std::string("auto"));
    dc.target_class = declare_parameter("target_class", -1);

    perception::DepthConfig pc;
    pc.roi_ratio = static_cast<float>(declare_parameter("depth_roi_ratio", 0.4));
    pc.min_depth = static_cast<float>(declare_parameter("min_depth", 0.2));
    pc.max_depth = static_cast<float>(declare_parameter("max_depth", 3.0));
    pc.min_valid_ratio = static_cast<float>(declare_parameter("min_valid_ratio", 0.5));
    pc.depth_scale = static_cast<float>(declare_parameter("depth_scale", 0.001));

    const auto color_topic = declare_parameter("color_topic", std::string("/camera/camera/color/image_raw"));
    const auto depth_topic =
        declare_parameter("depth_topic", std::string("/camera/camera/aligned_depth_to_color/image_raw"));
    const auto out_topic = declare_parameter("output_topic", std::string("/detection"));
    const auto timing_csv = declare_parameter("timing_csv", std::string(""));  // 비우면 기록 안 함
    const double slop = declare_parameter("sync_slop", 0.02);
    // 픽셀 → 각도 변환용 RGB 화각 [deg] (D435 color: 69 x 42)
    hfov_deg_ = declare_parameter("hfov_deg", 69.0);
    vfov_deg_ = declare_parameter("vfov_deg", 42.0);
    // 진단
    color_topic_ = color_topic;
    depth_topic_ = depth_topic;
    frame_timeout_ = declare_parameter("frame_timeout_s", 0.5);
    frozen_frames_ = declare_parameter("frozen_frames", 10);       // 30 fps 기준 약 0.33 s
    usb_check_ = declare_parameter("usb_check", true);             // 카메라가 다른 PC 에 꽂혀 있으면 false
    usb_vendor_ = declare_parameter("usb_vendor_id", std::string("8086"));   // Intel
    usb_product_ = declare_parameter("usb_product_id", std::string("0b07"));  // D435 (D435i=0b3a)
    const double health_rate = declare_parameter("health_rate_hz", 10.0);

    detector_ = perception::make_detector(dc);
    depth_ = std::make_unique<perception::DepthExtractor>(pc);
    RCLCPP_INFO(get_logger(), "detector backend=%s input=%dx%d threads=%d", detector_->name().c_str(),
                dc.input_w, dc.input_h, dc.num_threads);

    if (!timing_csv.empty()) {
      csv_.open(timing_csv);
      csv_ << "stamp_ns,backend,pre_ms,infer_ms,post_ms,depth_ms,callback_ms,detected,score,ex,ey,z,"
              "angle_x_deg,angle_y_deg\n";
    }

    pub_ = create_publisher<geometry_msgs::msg::PointStamped>(out_topic, rclcpp::QoS(1).best_effort());

    subscribe_sensor(color_sub_, this, color_topic, 0);
    subscribe_sensor(depth_sub_, this, depth_topic, 0);
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), color_sub_, depth_sub_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(slop));
    sync_->registerCallback(&PerceptionMaster::on_frames, this);
    // 진단용: 동기화 전 원본 수신도 따로 기록 (어느 쪽이 끊겼는지 구분)
    color_sub_.registerCallback([this](const Image::ConstSharedPtr & m) {on_raw(m, color_rx_);});
    depth_sub_.registerCallback([this](const Image::ConstSharedPtr & m) {on_raw(m, depth_rx_);});

    health_pub_ = create_publisher<DiagnosticStatus>("/perception/camera_health", 10);
    health_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / health_rate),
                                      [this]() { publish_health(); });
  }

 private:
  // ---------- 진단 ----------
  struct Stream {
    std::optional<rclcpp::Time> rx_t;  // 마지막 수신 시각 (노드 시계)
    int64_t last_stamp = -1;           // 마지막 header.stamp [ns]
    uint64_t last_hash = 0;
    int same_stamp = 0;                // stamp 가 안 늘어난 연속 횟수
    int same_image = 0;                // 내용이 같은 연속 횟수
  };

  // 영상 일부 바이트만 샘플링한 FNV-1a 해시. 전체가 한 값(가림·검은 화면)이면 0 → 멈춤 판정 제외
  static uint64_t sample_hash(const Image& m) {
    const auto& d = m.data;
    if (d.empty()) return 0;
    const size_t stride = d.size() / 4096 + 1;
    uint64_t h = 1469598103934665603ULL;
    bool uniform = true;
    for (size_t i = 0; i < d.size(); i += stride) {
      uniform = uniform && d[i] == d[0];
      h = (h ^ d[i]) * 1099511628211ULL;
    }
    return uniform ? 0 : h;
  }

  void on_raw(const Image::ConstSharedPtr& m, Stream& s) {
    s.rx_t = now();
    const int64_t stamp = rclcpp::Time(m->header.stamp).nanoseconds();
    s.same_stamp = (stamp <= s.last_stamp) ? s.same_stamp + 1 : 0;
    s.last_stamp = std::max(stamp, s.last_stamp);
    if (&s == &color_rx_) {  // depth 는 정적 장면·무효(0) 영역이 많아 내용 비교는 color 만
      const uint64_t h = sample_hash(*m);
      s.same_image = (h != 0 && h == s.last_hash) ? s.same_image + 1 : 0;
      s.last_hash = h;
    }
  }

  double age(const std::optional<rclcpp::Time>& t, const rclcpp::Time& now_t) const {
    return t ? (now_t - *t).seconds() : 1e9;
  }

  bool usb_present() const {
    std::error_code ec;
    for (const auto& e : std::filesystem::directory_iterator("/sys/bus/usb/devices", ec)) {
      std::ifstream v(e.path() / "idVendor"), p(e.path() / "idProduct");
      std::string vid, pid;
      if (v >> vid && p >> pid && vid == usb_vendor_ && pid == usb_product_) return true;
    }
    return false;
  }

  void publish_health() {
    const rclcpp::Time t = now();
    DiagnosticStatus d;
    d.name = "camera";
    d.hardware_id = "realsense";
    d.level = DiagnosticStatus::ERROR;
    const bool color_ok = age(color_rx_.rx_t, t) <= frame_timeout_;
    const bool depth_ok = age(depth_rx_.rx_t, t) <= frame_timeout_;
    char buf[96];

    if (!color_ok && !depth_ok) {
      if (usb_check_ && !usb_present()) {
        d.message = "camera USB disconnected";
      } else if (count_publishers(color_topic_) == 0 && count_publishers(depth_topic_) == 0) {
        d.message = "camera driver not running";
      } else {
        d.message = "frames stopped";
      }
    } else if (!color_ok) {
      d.message = "no color";
    } else if (!depth_ok) {
      d.message = "no depth";
    } else if (color_rx_.same_stamp >= frozen_frames_ || color_rx_.same_image >= frozen_frames_) {
      d.message = "color frozen";
    } else if (depth_rx_.same_stamp >= frozen_frames_) {
      d.message = "depth frozen";
    } else if (age(sync_t_, t) > frame_timeout_) {
      d.message = "sync fail";
    } else if (age(exc_t_, t) <= frame_timeout_) {
      d.message = "exception: " + exc_msg_;
    } else {
      d.level = DiagnosticStatus::OK;
      const double win = age(fps_t0_, t);
      std::snprintf(buf, sizeof(buf), "fps %.1f", win > 0 && win < 1e8 ? fps_n_ / win : 0.0);
      d.message = buf;
    }
    if (age(fps_t0_, t) >= 1.0) {  // fps 창 1초마다 초기화
      fps_t0_ = t;
      fps_n_ = 0;
    }
    health_pub_->publish(d);
  }

  void on_frames(const Image::ConstSharedPtr& color, const Image::ConstSharedPtr& depth) {
    sync_t_ = now();
    ++fps_n_;
    try {
      process(color, depth);
    } catch (const std::exception& e) {  // 예외로 노드가 죽지 않게 하고 진단으로 알린다
      exc_t_ = now();
      exc_msg_ = e.what();
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "frame 처리 실패: %s", e.what());
    }
  }

  void process(const Image::ConstSharedPtr& color, const Image::ConstSharedPtr& depth) {
    using Clock = std::chrono::steady_clock;
    const auto t_cb = Clock::now();

    const cv::Mat bgr = cv_bridge::toCvShare(color, "bgr8")->image;
    const cv::Mat depth_img = cv_bridge::toCvShare(depth)->image;  // 16UC1 [mm]

    perception::Timing t;
    const auto det = detector_->detect(bgr, &t);

    geometry_msgs::msg::PointStamped msg;
    msg.header = color->header;  // 촬영(수신) 시각 유지

    double depth_ms = 0.0;
    double angle_x = 0.0, angle_y = 0.0;  // [deg] 미검출이면 0
    if (det) {
      const float cx = det->box.x + det->box.width / 2.f;
      const float cy = det->box.y + det->box.height / 2.f;
      const float half_w = bgr.cols / 2.f, half_h = bgr.rows / 2.f;
      msg.point.x = (cx - half_w) / half_w;
      msg.point.y = (cy - half_h) / half_h;
      // 픽셀 → 각도: 좌우 = (HFOV/2)·(w/2 − x)/(w/2), 상하 = (VFOV/2)·(h/2 − y)/(h/2)
      // 부호가 ex·ey와 반대 (왼쪽·위가 +). /target 출력은 규약대로 ex·ey 유지, 각도는 CSV·디버그 로그에만 기록
      angle_x = (hfov_deg_ / 2.0) * (half_w - cx) / half_w;
      angle_y = (vfov_deg_ / 2.0) * (half_h - cy) / half_h;
      RCLCPP_DEBUG(get_logger(), "angle_x=%.2f deg angle_y=%.2f deg", angle_x, angle_y);
      const auto t_d = Clock::now();
      const auto z = depth_->median(depth_img, det->box);
      depth_ms = std::chrono::duration<double, std::milli>(Clock::now() - t_d).count();
      msg.point.z = z.value_or(0.0);  // depth 실패(범위 밖·유효 픽셀 부족)도 0
    } else {
      msg.point.x = msg.point.y = msg.point.z = 0.0;  // 미검출: z=0
    }
    pub_->publish(msg);

    if (csv_.is_open()) {
      const double cb_ms = std::chrono::duration<double, std::milli>(Clock::now() - t_cb).count();
      const int64_t stamp = rclcpp::Time(color->header.stamp).nanoseconds();
      csv_ << stamp << ',' << detector_->name() << ',' << t.pre_ms << ',' << t.infer_ms << ',' << t.post_ms << ','
           << depth_ms << ',' << cb_ms << ',' << (det ? 1 : 0) << ',' << (det ? det->score : 0.f) << ','
           << msg.point.x << ',' << msg.point.y << ',' << msg.point.z << ',' << angle_x << ',' << angle_y << '\n';
    }
  }

  double hfov_deg_ = 69.0, vfov_deg_ = 42.0;

  // 진단
  std::string color_topic_, depth_topic_, usb_vendor_, usb_product_;
  double frame_timeout_ = 0.5;
  int64_t frozen_frames_ = 10;
  bool usb_check_ = true;
  Stream color_rx_, depth_rx_;  // 원본 수신 기록 (depth_ 는 DepthExtractor)
  std::optional<rclcpp::Time> sync_t_, exc_t_, fps_t0_;
  std::string exc_msg_;
  int fps_n_ = 0;
  rclcpp::Publisher<DiagnosticStatus>::SharedPtr health_pub_;
  rclcpp::TimerBase::SharedPtr health_timer_;

  std::unique_ptr<perception::Detector> detector_;
  std::unique_ptr<perception::DepthExtractor> depth_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_;
  message_filters::Subscriber<Image> color_sub_, depth_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  std::ofstream csv_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PerceptionMaster>());
  rclcpp::shutdown();
  return 0;
}
