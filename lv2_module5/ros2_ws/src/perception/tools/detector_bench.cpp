// detector_bench: 같은 입력 프레임으로 NCNN / ONNX 백엔드를 오프라인 벤치마크
//
// 입력 (--input)
//   - rosbag2 디렉터리 (--topic 의 sensor_msgs/Image 또는 CompressedImage) ※ ROS 빌드에서만
//   - 영상 파일 (.mp4, .avi ...)
//   - 이미지 폴더 (.jpg/.png, 파일명 순)
//
// 모든 프레임을 빠짐없이 처리한다 (실시간 재생과 달리 프레임 드롭이 없어 백엔드 간 공정 비교 가능).
//
// 출력
//   --out <csv>          프레임별 시간·검출 결과
//   <csv>.meta.json      실행 조건 (백엔드, 모델, 입력 크기, 스레드, 모델 로드 시간 ...)
//   --save-every K       K 프레임마다 검출 결과를 그린 이미지 저장 (사람 대조용)
//
// 예)
//   detector_bench --backend ncnn --ncnn-param m.ncnn.param --ncnn-bin m.ncnn.bin
//                  --input bag_dir --topic /camera/camera/color/image_raw --out ncnn.csv
//   detector_bench --backend onnx --onnx m.onnx --input bag_dir --out onnx.csv
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "perception/detector.hpp"

#ifdef BENCH_WITH_ROSBAG
#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#endif

namespace fs = std::filesystem;

namespace {

struct Args {
  perception::DetectorConfig dc;
  std::string input, topic = "/camera/camera/color/image_raw", out = "bench.csv", save_dir;
  int warmup = 10, max_frames = -1, save_every = 0;
};

void usage() {
  std::cerr <<
      "usage: detector_bench --backend ncnn|onnx [--ncnn-param P --ncnn-bin B | --onnx M]\n"
      "                      --input <bag_dir|video|image_dir> [--topic T] --out <csv>\n"
      "                      [--size 320|640x480] [--conf 0.25] [--threads 3] [--format auto|raw|e2e]\n"
      "                      [--class -1] [--ncnn-input in0] [--ncnn-output out0]\n"
      "                      [--warmup 10] [--max-frames -1] [--save-every 0] [--save-dir DIR]\n";
}

Args parse(int argc, char** argv) {
  Args a;
  std::map<std::string, std::string> kv;
  for (int i = 1; i < argc; ++i) {
    std::string k = argv[i];
    if (k == "-h" || k == "--help") { usage(); std::exit(0); }
    if (k.rfind("--", 0) != 0 || i + 1 >= argc) { usage(); std::exit(2); }
    kv[k.substr(2)] = argv[++i];
  }
  auto get = [&](const std::string& k, const std::string& d) { return kv.count(k) ? kv[k] : d; };
  a.dc.backend = get("backend", "ncnn");
  a.dc.ncnn_param = get("ncnn-param", "");
  a.dc.ncnn_bin = get("ncnn-bin", "");
  a.dc.ncnn_input = get("ncnn-input", "in0");
  a.dc.ncnn_output = get("ncnn-output", "out0");
  a.dc.onnx_model = get("onnx", "");
  // --size: 정사각형이면 "320", 직사각형이면 "WxH" (예: 640x480 ↔ export imgsz=[480,640])
  const std::string size = get("size", "320");
  const auto xpos = size.find('x');
  a.dc.input_w = std::stoi(size.substr(0, xpos));
  a.dc.input_h = (xpos == std::string::npos) ? a.dc.input_w : std::stoi(size.substr(xpos + 1));
  a.dc.conf_threshold = std::stof(get("conf", "0.25"));
  a.dc.num_threads = std::stoi(get("threads", "3"));
  a.dc.output_format = get("format", "auto");
  a.dc.target_class = std::stoi(get("class", "-1"));
  a.input = get("input", "");
  a.topic = get("topic", a.topic);
  a.out = get("out", a.out);
  a.warmup = std::stoi(get("warmup", "10"));
  a.max_frames = std::stoi(get("max-frames", "-1"));
  a.save_every = std::stoi(get("save-every", "0"));
  a.save_dir = get("save-dir", "");
  if (a.input.empty()) { usage(); std::exit(2); }
  return a;
}

// ---- 프레임 소스 ----
struct Frame {
  cv::Mat bgr;
  int64_t stamp_ns = 0;
};

class FrameSource {
 public:
  virtual ~FrameSource() = default;
  virtual bool next(Frame& f) = 0;
};

class ImageDirSource : public FrameSource {
 public:
  explicit ImageDirSource(const std::string& dir) {
    for (auto& e : fs::directory_iterator(dir)) {
      auto ext = e.path().extension().string();
      std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
      if (ext == ".jpg" || ext == ".jpeg" || ext == ".png" || ext == ".bmp") files_.push_back(e.path().string());
    }
    std::sort(files_.begin(), files_.end());
  }
  bool next(Frame& f) override {
    while (i_ < files_.size()) {
      f.bgr = cv::imread(files_[i_], cv::IMREAD_COLOR);
      f.stamp_ns = static_cast<int64_t>(i_);
      ++i_;
      if (!f.bgr.empty()) return true;
    }
    return false;
  }
 private:
  std::vector<std::string> files_;
  size_t i_ = 0;
};

class VideoSource : public FrameSource {
 public:
  explicit VideoSource(const std::string& path) : cap_(path) {
    if (!cap_.isOpened()) throw std::runtime_error("cannot open video " + path);
  }
  bool next(Frame& f) override {
    if (!cap_.read(f.bgr)) return false;
    f.stamp_ns = static_cast<int64_t>(cap_.get(cv::CAP_PROP_POS_MSEC) * 1e6);
    return true;
  }
 private:
  cv::VideoCapture cap_;
};

#ifdef BENCH_WITH_ROSBAG
class BagSource : public FrameSource {
 public:
  BagSource(const std::string& uri, const std::string& topic) : topic_(topic) {
    reader_.open(uri);
    for (const auto& t : reader_.get_all_topics_and_types())
      if (t.name == topic) type_ = t.type;
    if (type_.empty()) throw std::runtime_error("topic not found in bag: " + topic);
    rosbag2_storage::StorageFilter filter;
    filter.topics = {topic};
    reader_.set_filter(filter);
  }
  bool next(Frame& f) override {
    while (reader_.has_next()) {
      auto m = reader_.read_next();
      if (m->topic_name != topic_) continue;
      rclcpp::SerializedMessage sm(*m->serialized_data);
      if (type_ == "sensor_msgs/msg/Image") {
        sensor_msgs::msg::Image img;
        img_ser_.deserialize_message(&sm, &img);
        f.bgr = cv_bridge::toCvCopy(img, "bgr8")->image;
        f.stamp_ns = rclcpp::Time(img.header.stamp).nanoseconds();
      } else if (type_ == "sensor_msgs/msg/CompressedImage") {
        sensor_msgs::msg::CompressedImage img;
        cimg_ser_.deserialize_message(&sm, &img);
        f.bgr = cv::imdecode(img.data, cv::IMREAD_COLOR);
        f.stamp_ns = rclcpp::Time(img.header.stamp).nanoseconds();
      } else {
        throw std::runtime_error("unsupported topic type: " + type_);
      }
      if (!f.bgr.empty()) return true;
    }
    return false;
  }
 private:
  rosbag2_cpp::Reader reader_;
  std::string topic_, type_;
  rclcpp::Serialization<sensor_msgs::msg::Image> img_ser_;
  rclcpp::Serialization<sensor_msgs::msg::CompressedImage> cimg_ser_;
};
#endif

std::unique_ptr<FrameSource> open_source(const Args& a) {
  if (fs::is_directory(a.input)) {
    if (fs::exists(fs::path(a.input) / "metadata.yaml")) {
#ifdef BENCH_WITH_ROSBAG
      return std::make_unique<BagSource>(a.input, a.topic);
#else
      throw std::runtime_error("rosbag input needs ROS build (BENCH_WITH_ROSBAG)");
#endif
    }
    return std::make_unique<ImageDirSource>(a.input);
  }
  return std::make_unique<VideoSource>(a.input);
}

std::string json_escape(const std::string& s) {
  std::string o;
  for (char c : s) { if (c == '"' || c == '\\') o += '\\'; o += c; }
  return o;
}

std::string model_path(const perception::DetectorConfig& dc) {
  return dc.backend == "onnx" ? dc.onnx_model : dc.ncnn_param;
}

uintmax_t model_bytes(const perception::DetectorConfig& dc) {
  std::error_code ec;
  if (dc.backend == "onnx") return fs::file_size(dc.onnx_model, ec);
  return fs::file_size(dc.ncnn_param, ec) + fs::file_size(dc.ncnn_bin, ec);
}

}  // namespace

int main(int argc, char** argv) {
  const Args a = parse(argc, argv);
  using Clock = std::chrono::steady_clock;

  // 모델 로드 시간
  const auto t_load = Clock::now();
  auto det = perception::make_detector(a.dc);
  const double load_ms = std::chrono::duration<double, std::milli>(Clock::now() - t_load).count();

  auto src = open_source(a);
  if (!a.save_dir.empty()) fs::create_directories(a.save_dir);

  std::ofstream csv(a.out);
  csv << "backend,frame_idx,stamp_ns,width,height,pre_ms,infer_ms,post_ms,total_ms,"
         "detected,score,cls,x1,y1,x2,y2,ex,ey\n";
  csv << std::fixed << std::setprecision(4);

  Frame f;
  int idx = 0;
  double sum_total = 0.0;
  const auto t_wall = Clock::now();

  while ((a.max_frames < 0 || idx < a.max_frames) && src->next(f)) {
    // 첫 프레임으로 워밍업 (기록 안 함): 메모리 할당·캐시 등 초기 비용 제외
    if (idx == 0)
      for (int w = 0; w < a.warmup; ++w) det->detect(f.bgr);

    perception::Timing t;
    const auto d = det->detect(f.bgr, &t);
    sum_total += t.total_ms();

    double ex = 0, ey = 0;
    cv::Rect b;
    if (d) {
      b = d->box;
      ex = (b.x + b.width / 2.0 - f.bgr.cols / 2.0) / (f.bgr.cols / 2.0);
      ey = (b.y + b.height / 2.0 - f.bgr.rows / 2.0) / (f.bgr.rows / 2.0);
    }
    csv << det->name() << ',' << idx << ',' << f.stamp_ns << ',' << f.bgr.cols << ',' << f.bgr.rows << ','
        << t.pre_ms << ',' << t.infer_ms << ',' << t.post_ms << ',' << t.total_ms() << ',' << (d ? 1 : 0) << ','
        << (d ? d->score : 0.f) << ',' << (d ? d->cls : -1) << ',' << b.x << ',' << b.y << ',' << b.x + b.width
        << ',' << b.y + b.height << ',' << ex << ',' << ey << '\n';

    if (a.save_every > 0 && idx % a.save_every == 0 && !a.save_dir.empty()) {
      cv::Mat vis = f.bgr.clone();
      cv::drawMarker(vis, {vis.cols / 2, vis.rows / 2}, {255, 255, 255}, cv::MARKER_CROSS, 20, 1);
      if (d) {
        cv::rectangle(vis, b, {0, 255, 0}, 2);
        cv::circle(vis, {b.x + b.width / 2, b.y + b.height / 2}, 4, {0, 0, 255}, -1);
      }
      std::ostringstream label;
      label << det->name() << " #" << idx << (d ? " det " : " none ") << std::setprecision(2) << (d ? d->score : 0.f);
      cv::putText(vis, label.str(), {8, 24}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {0, 255, 255}, 2);
      std::ostringstream fn;
      fn << a.save_dir << '/' << det->name() << '_' << std::setw(6) << std::setfill('0') << idx << ".jpg";
      cv::imwrite(fn.str(), vis);
    }
    ++idx;
  }
  const double wall_s = std::chrono::duration<double>(Clock::now() - t_wall).count();

  std::ofstream meta(a.out + ".meta.json");
  meta << "{\n"
       << "  \"backend\": \"" << det->name() << "\",\n"
       << "  \"model\": \"" << json_escape(model_path(a.dc)) << "\",\n"
       << "  \"model_bytes\": " << model_bytes(a.dc) << ",\n"
       << "  \"input\": \"" << json_escape(a.input) << "\",\n"
       << "  \"topic\": \"" << json_escape(a.topic) << "\",\n"
       << "  \"input_w\": " << a.dc.input_w << ",\n"
       << "  \"input_h\": " << a.dc.input_h << ",\n"
       << "  \"threads\": " << a.dc.num_threads << ",\n"
       << "  \"conf_threshold\": " << a.dc.conf_threshold << ",\n"
       << "  \"output_format\": \"" << a.dc.output_format << "\",\n"
       << "  \"warmup\": " << a.warmup << ",\n"
       << "  \"frames\": " << idx << ",\n"
       << "  \"load_ms\": " << load_ms << ",\n"
       << "  \"wall_s\": " << wall_s << ",\n"
       << "  \"opencv\": \"" << CV_VERSION << "\"\n"
       << "}\n";

  std::cerr << det->name() << ": " << idx << " frames, mean total " << (idx ? sum_total / idx : 0) << " ms, "
            << "processing FPS " << (sum_total > 0 ? idx * 1000.0 / sum_total : 0) << ", load " << load_ms
            << " ms -> " << a.out << '\n';
  return 0;
}
