// ONNX Runtime 백엔드
#include <stdexcept>
#include <string>

#include <onnxruntime_cxx_api.h>

#include "detector_backends.hpp"

namespace perception {
namespace {

class OnnxDetector : public Detector {
 public:
  explicit OnnxDetector(const DetectorConfig& cfg)
      : Detector(cfg),
        env_(ORT_LOGGING_LEVEL_WARNING, "perception_detector"),
        mem_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)) {
    Ort::SessionOptions so;
    so.SetIntraOpNumThreads(cfg.num_threads);   // NCNN과 같은 스레드 수로 맞춰 비교
    so.SetInterOpNumThreads(1);
    so.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    session_ = std::make_unique<Ort::Session>(env_, cfg.onnx_model.c_str(), so);

    Ort::AllocatorWithDefaultOptions alloc;
    in_name_ = session_->GetInputNameAllocated(0, alloc).get();
    out_name_ = session_->GetOutputNameAllocated(0, alloc).get();

    // ONNX는 export 시 입력 크기가 고정됨 → 설정과 다르면 추론 때가 아니라 여기서 바로 알림
    const auto in_shape = session_->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();  // [1,3,H,W]
    if (in_shape.size() == 4 && in_shape[2] > 0 && in_shape[3] > 0 &&
        (in_shape[2] != cfg.input_h || in_shape[3] != cfg.input_w))
      throw std::runtime_error("onnx: model input is " + std::to_string(in_shape[3]) + "x" +
                               std::to_string(in_shape[2]) + " (WxH) but config is " +
                               std::to_string(cfg.input_w) + "x" + std::to_string(cfg.input_h));
  }

  std::string name() const override { return "onnx"; }

 protected:
  void infer(const cv::Mat& blob, std::vector<float>& out, std::vector<int64_t>& shape) override {
    const int64_t w = cfg_.input_w, h = cfg_.input_h;
    const int64_t in_shape[4] = {1, 3, h, w};
    auto* data = const_cast<float*>(blob.ptr<float>());
    Ort::Value in = Ort::Value::CreateTensor<float>(mem_, data, static_cast<size_t>(3 * h * w), in_shape, 4);

    const char* in_names[] = {in_name_.c_str()};
    const char* out_names[] = {out_name_.c_str()};
    auto outs = session_->Run(Ort::RunOptions{nullptr}, in_names, &in, 1, out_names, 1);

    auto info = outs[0].GetTensorTypeAndShapeInfo();
    shape = info.GetShape();
    const float* p = outs[0].GetTensorData<float>();
    out.assign(p, p + info.GetElementCount());
  }

 private:
  Ort::Env env_;
  Ort::MemoryInfo mem_;
  std::unique_ptr<Ort::Session> session_;
  std::string in_name_, out_name_;
};

}  // namespace

std::unique_ptr<Detector> make_onnx_detector(const DetectorConfig& cfg) {
  return std::make_unique<OnnxDetector>(cfg);
}

}  // namespace perception
