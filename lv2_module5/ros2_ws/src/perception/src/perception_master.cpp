// perception_master: 새 프레임마다 ① Detector → ② Depth Extractor → (③ IMU · Joint Reader) 호출
//
// 출력 /detection (geometry_msgs/PointStamped)
//   x = e_x, y = e_y  (정규화 중심 오차, 오른쪽·아래가 +)
//   z = depth [m]     검출 + depth 성공
//   z = 0             미검출 (매 프레임 발행 — 발제 규약: "정상 영상의 미검출은 z=0 발행")
//   z = NaN           검출은 됐지만 depth 실패
//   header.stamp      원본 영상 시각 유지
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <string>

#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <message_filters/synchronizer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "perception/depth_extractor.hpp"
#include "perception/detector.hpp"

using sensor_msgs::msg::Image;
using SyncPolicy = message_filters::sync_policies::ApproximateTime<Image, Image>;

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

    color_sub_.subscribe(this, color_topic, rclcpp::SensorDataQoS());
    depth_sub_.subscribe(this, depth_topic, rclcpp::SensorDataQoS());
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), color_sub_, depth_sub_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(slop));
    sync_->registerCallback(&PerceptionMaster::on_frames, this);
  }

 private:
  void on_frames(const Image::ConstSharedPtr& color, const Image::ConstSharedPtr& depth) {
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
      msg.point.z = z.value_or(std::nanf(""));
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
