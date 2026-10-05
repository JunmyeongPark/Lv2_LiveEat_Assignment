// Depth Extractor: bbox 내 depth 중앙값
#pragma once

#include <optional>

#include <opencv2/core.hpp>

namespace perception {

struct DepthConfig {
  float roi_ratio = 0.4f;        // bbox 중앙에서 사용할 영역 비율 (가장자리 배경 제외)
  float min_depth = 0.2f;        // [m] 이보다 가까운 값 제외
  float max_depth = 3.0f;        // [m] 이보다 먼 값 제외
  float min_valid_ratio = 0.5f;  // ROI 중 유효 픽셀 비율이 이보다 낮으면 실패
  float depth_scale = 0.001f;    // 16UC1 → m (RealSense 기본 1 unit = 1 mm)
};

class DepthExtractor {
 public:
  explicit DepthExtractor(const DepthConfig& cfg) : cfg_(cfg) {}

  // depth: RGB에 정렬된 depth (16UC1 [unit] 또는 32FC1 [m]).
  // min_depth ~ max_depth 픽셀의 중앙값. 유효 픽셀이 min_valid_ratio 미만이면 nullopt (노드는 z=0 발행)
  std::optional<float> median(const cv::Mat& depth, const cv::Rect& box) const;

 private:
  DepthConfig cfg_;
};

}  // namespace perception
