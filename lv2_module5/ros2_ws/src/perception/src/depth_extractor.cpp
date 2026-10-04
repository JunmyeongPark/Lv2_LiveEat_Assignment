// Depth Extractor: bbox 내 depth 중앙값
#include "perception/depth_extractor.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace perception {

std::optional<float> DepthExtractor::median(const cv::Mat& depth, const cv::Rect& box) const {
  if (depth.empty() || box.area() <= 0) return std::nullopt;

  // bbox 중앙 roi_ratio 영역만 사용
  const int rw = std::max(1, static_cast<int>(std::lround(box.width * cfg_.roi_ratio)));
  const int rh = std::max(1, static_cast<int>(std::lround(box.height * cfg_.roi_ratio)));
  cv::Rect roi(box.x + (box.width - rw) / 2, box.y + (box.height - rh) / 2, rw, rh);
  roi &= cv::Rect(0, 0, depth.cols, depth.rows);
  if (roi.area() <= 0) return std::nullopt;

  std::vector<float> vals;
  vals.reserve(roi.area());
  const bool is_u16 = depth.type() == CV_16UC1;
  if (!is_u16 && depth.type() != CV_32FC1) return std::nullopt;

  for (int y = roi.y; y < roi.y + roi.height; ++y) {
    for (int x = roi.x; x < roi.x + roi.width; ++x) {
      const float d = is_u16 ? depth.at<uint16_t>(y, x) * cfg_.depth_scale : depth.at<float>(y, x);
      if (std::isfinite(d) && d >= cfg_.min_depth && d <= cfg_.max_depth) vals.push_back(d);
    }
  }
  if (vals.empty() || vals.size() < cfg_.min_valid_ratio * roi.area()) return std::nullopt;

  auto mid = vals.begin() + vals.size() / 2;
  std::nth_element(vals.begin(), mid, vals.end());
  return *mid;
}

}  // namespace perception
