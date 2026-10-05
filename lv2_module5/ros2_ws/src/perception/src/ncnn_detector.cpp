// NCNN 백엔드
#include <cstring>
#include <stdexcept>

#include <net.h>  // ncnn CMake 타깃이 include/ncnn을 include 경로로 추가함

#include "detector_backends.hpp"

namespace perception {
namespace {

class NcnnDetector : public Detector {
 public:
  explicit NcnnDetector(const DetectorConfig& cfg) : Detector(cfg) {
    net_.opt.num_threads = cfg.num_threads;   // Pi4: 4 (perception.yaml num_threads)
    net_.opt.use_vulkan_compute = false;      // Pi4 GPU는 사용하지 않음
    if (net_.load_param(cfg.ncnn_param.c_str()) != 0)
      throw std::runtime_error("ncnn: failed to load param " + cfg.ncnn_param);
    if (net_.load_model(cfg.ncnn_bin.c_str()) != 0)
      throw std::runtime_error("ncnn: failed to load bin " + cfg.ncnn_bin);
  }

  std::string name() const override { return "ncnn"; }

 protected:
  void infer(const cv::Mat& blob, std::vector<float>& out, std::vector<int64_t>& shape) override {
    const int w = cfg_.input_w, h = cfg_.input_h;
    const size_t plane = static_cast<size_t>(w) * h;
    // ncnn::Mat은 채널마다 정렬(cstep)이 있어 채널 단위로 복사
    ncnn::Mat in(w, h, 3);
    const float* src = blob.ptr<float>();
    for (int c = 0; c < 3; ++c)
      std::memcpy(in.channel(c), src + c * plane, sizeof(float) * plane);

    ncnn::Extractor ex = net_.create_extractor();
    if (ex.input(cfg_.ncnn_input.c_str(), in) != 0)
      throw std::runtime_error("ncnn: bad input blob name " + cfg_.ncnn_input);
    ncnn::Mat o;
    if (ex.extract(cfg_.ncnn_output.c_str(), o) != 0)
      throw std::runtime_error("ncnn: bad output blob name " + cfg_.ncnn_output);

    // 2D 출력 (h=행, w=열). 3D면 c*h를 행으로 펼침
    const int rows = (o.dims == 3) ? o.c * o.h : o.h;
    const int cols = o.w;
    shape = {rows, cols};
    out.resize(static_cast<size_t>(rows) * cols);
    if (o.dims == 3) {
      for (int c = 0; c < o.c; ++c)
        for (int y = 0; y < o.h; ++y)
          std::memcpy(out.data() + (static_cast<size_t>(c) * o.h + y) * cols, o.channel(c).row(y),
                      sizeof(float) * cols);
    } else {
      for (int y = 0; y < rows; ++y)
        std::memcpy(out.data() + static_cast<size_t>(y) * cols, o.row(y), sizeof(float) * cols);
    }
  }

 private:
  ncnn::Net net_;
};

}  // namespace

std::unique_ptr<Detector> make_ncnn_detector(const DetectorConfig& cfg) {
  return std::make_unique<NcnnDetector>(cfg);
}

}  // namespace perception
