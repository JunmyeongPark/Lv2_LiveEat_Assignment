// Detector (YOLO26): bbox 중심 e_x, e_y
// 추론 백엔드(NCNN / ONNX Runtime)만 바꿔 끼우고, 전처리·후처리는 공통으로 사용한다.
// → 두 백엔드를 비교할 때 "추론 엔진" 차이만 남도록 하기 위함.
#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace perception {

struct Detection {
  cv::Rect box;      // 원본 영상 좌표 (px)
  float score = 0.f;
  int cls = 0;
};

// 단계별 처리 시간 [ms]
struct Timing {
  double pre_ms = 0.0;    // letterbox + 정규화 + NCHW 변환
  double infer_ms = 0.0;  // 엔진 추론 (입력 복사 포함)
  double post_ms = 0.0;   // 출력 해석 + 좌표 복원
  double total_ms() const { return pre_ms + infer_ms + post_ms; }
};

struct DetectorConfig {
  std::string backend = "ncnn";      // "ncnn" | "onnx"
  std::string ncnn_param;            // model.ncnn.param
  std::string ncnn_bin;              // model.ncnn.bin
  std::string ncnn_input = "in0";    // ultralytics NCNN export 기본 blob 이름
  std::string ncnn_output = "out0";
  std::string onnx_model;            // model.onnx
  int input_w = 320;                 // 모델 입력 가로·세로 [px]. export 시 imgsz=[h, w]와 같아야 함
  int input_h = 320;                 // (ultralytics imgsz는 [높이, 너비] 순서)
  float conf_threshold = 0.25f;
  int num_threads = 4;
  std::string output_format = "auto";  // "auto" | "raw" (4+nc, N) | "e2e" (N, 6)
  int target_class = -1;               // -1이면 모든 클래스 중 최고 점수
};

// letterbox 변환 정보 (출력 좌표 복원용)
struct Letterbox {
  float scale = 1.f;
  int pad_x = 0;
  int pad_y = 0;
};

class Detector {
 public:
  virtual ~Detector() = default;

  // 공통 파이프라인: 전처리 → infer() → 후처리. timing이 주어지면 단계별 시간 기록.
  std::optional<Detection> detect(const cv::Mat& bgr, Timing* timing = nullptr);

  virtual std::string name() const = 0;
  const DetectorConfig& config() const { return cfg_; }

 protected:
  explicit Detector(const DetectorConfig& cfg) : cfg_(cfg) {}

  // blob: 1x3xHxW float32 (RGB, 0~1). out: 출력 텐서(평탄화), shape: 출력 차원
  virtual void infer(const cv::Mat& blob, std::vector<float>& out,
                     std::vector<int64_t>& shape) = 0;

  DetectorConfig cfg_;

 private:
  std::vector<float> out_buf_;
  std::vector<int64_t> out_shape_;
};

// cfg.backend에 맞는 Detector 생성. 해당 백엔드 없이 빌드됐으면 std::runtime_error.
std::unique_ptr<Detector> make_detector(const DetectorConfig& cfg);

// ---- 공통 전처리·후처리 (테스트·벤치마크에서도 사용) ----
// size: 모델 입력 (W, H). 비율 유지 축소 후 남는 쪽을 회색(114)으로 채움
cv::Mat letterbox_blob(const cv::Mat& bgr, const cv::Size& size, Letterbox& lb);

std::optional<Detection> parse_output(const std::vector<float>& out,
                                      const std::vector<int64_t>& shape,
                                      const Letterbox& lb, const cv::Size& image_size,
                                      const DetectorConfig& cfg);

}  // namespace perception
