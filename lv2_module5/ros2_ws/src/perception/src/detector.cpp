// Detector (YOLO26) 공통부: 전처리(letterbox), 후처리(출력 해석), 시간 측정
// 백엔드별 추론은 ncnn_detector.cpp / onnx_detector.cpp
#include "perception/detector.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

#include <opencv2/imgproc.hpp>

namespace perception {
namespace {

using Clock = std::chrono::steady_clock;
double ms_since(Clock::time_point t0) {
  return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

// 앞쪽의 크기 1 차원(batch 등)을 제거
std::vector<int64_t> squeeze_leading(const std::vector<int64_t>& shape) {
  std::vector<int64_t> s = shape;
  while (s.size() > 2 && s.front() == 1) s.erase(s.begin());
  return s;
}

cv::Rect to_image_rect(float x1, float y1, float x2, float y2, const Letterbox& lb,
                       const cv::Size& img) {
  auto ux = [&](float v) { return (v - lb.pad_x) / lb.scale; };
  auto uy = [&](float v) { return (v - lb.pad_y) / lb.scale; };
  cv::Rect r(cv::Point(static_cast<int>(std::lround(ux(x1))), static_cast<int>(std::lround(uy(y1)))),
             cv::Point(static_cast<int>(std::lround(ux(x2))), static_cast<int>(std::lround(uy(y2)))));
  return r & cv::Rect(0, 0, img.width, img.height);
}

}  // namespace

cv::Mat letterbox_blob(const cv::Mat& bgr, const cv::Size& size, Letterbox& lb) {
  if (bgr.empty() || bgr.type() != CV_8UC3) throw std::invalid_argument("letterbox_blob: need CV_8UC3 BGR image");
  const int W = size.width, H = size.height;

  // 예: 640x480 영상 → 640x640 입력이면 scale 1, 위아래 80px 패딩 / 640x480 입력이면 패딩 없음
  lb.scale = std::min(W / static_cast<float>(bgr.cols), H / static_cast<float>(bgr.rows));
  const int w = static_cast<int>(std::lround(bgr.cols * lb.scale));
  const int h = static_cast<int>(std::lround(bgr.rows * lb.scale));
  lb.pad_x = (W - w) / 2;
  lb.pad_y = (H - h) / 2;

  cv::Mat resized, padded, rgb, f32;
  if (w == bgr.cols && h == bgr.rows) resized = bgr;  // 크기가 같으면 resize 생략
  else cv::resize(bgr, resized, cv::Size(w, h), 0, 0, cv::INTER_LINEAR);
  cv::copyMakeBorder(resized, padded, lb.pad_y, H - h - lb.pad_y, lb.pad_x, W - w - lb.pad_x,
                     cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
  cv::cvtColor(padded, rgb, cv::COLOR_BGR2RGB);
  rgb.convertTo(f32, CV_32FC3, 1.0 / 255.0);

  // HWC → NCHW (1x3xHxW)
  const int dims[4] = {1, 3, H, W};
  cv::Mat blob(4, dims, CV_32F);
  std::vector<cv::Mat> planes;
  planes.reserve(3);
  for (int c = 0; c < 3; ++c)
    planes.emplace_back(H, W, CV_32F, blob.ptr<float>() + static_cast<size_t>(c) * H * W);
  cv::split(f32, planes);  // 크기·타입이 같으므로 blob 메모리에 바로 기록됨
  return blob;
}

std::optional<Detection> parse_output(const std::vector<float>& out,
                                      const std::vector<int64_t>& shape,
                                      const Letterbox& lb, const cv::Size& image_size,
                                      const DetectorConfig& cfg) {
  const auto s = squeeze_leading(shape);
  if (s.size() != 2) throw std::runtime_error("parse_output: unexpected output rank");
  const int64_t d0 = s[0], d1 = s[1];
  if (static_cast<int64_t>(out.size()) < d0 * d1) throw std::runtime_error("parse_output: output too small");

  // 형식 판별
  //  e2e : (N, 6)      [x1, y1, x2, y2, score, cls]   — YOLO26 end-to-end (NMS-free) export
  //  raw : (4+nc, N)   [cx, cy, w, h, cls0, cls1, ...] — one-to-many export (NCNN은 보통 이 형식)
  std::string fmt = cfg.output_format;
  if (fmt == "auto") fmt = (d1 == 6 && d0 != 6) ? "e2e" : "raw";

  int best = -1;
  float best_score = cfg.conf_threshold;
  int best_cls = 0;

  if (fmt == "e2e") {
    for (int64_t i = 0; i < d0; ++i) {
      const float* r = out.data() + i * d1;
      const int cls = static_cast<int>(std::lround(r[5]));
      if (cfg.target_class >= 0 && cls != cfg.target_class) continue;
      if (r[4] > best_score) { best_score = r[4]; best = static_cast<int>(i); best_cls = cls; }
    }
    if (best < 0) return std::nullopt;
    const float* r = out.data() + best * d1;
    const cv::Rect box = to_image_rect(r[0], r[1], r[2], r[3], lb, image_size);
    if (box.area() <= 0) return std::nullopt;
    return Detection{box, best_score, best_cls};
  }

  // raw: 행 = 채널(cx, cy, w, h, 클래스 점수...), 열 = 후보 N개
  const int64_t nc = d0 - 4, n = d1;
  if (nc < 1) throw std::runtime_error("parse_output: raw format needs 4+nc rows");
  // 표적이 하나뿐이라 NMS 대신 최고 점수 후보 1개만 사용
  for (int64_t i = 0; i < n; ++i) {
    for (int64_t c = 0; c < nc; ++c) {
      if (cfg.target_class >= 0 && c != cfg.target_class) continue;
      const float sc = out[(4 + c) * n + i];
      if (sc > best_score) { best_score = sc; best = static_cast<int>(i); best_cls = static_cast<int>(c); }
    }
  }
  if (best < 0) return std::nullopt;
  const float cx = out[0 * n + best], cy = out[1 * n + best];
  const float bw = out[2 * n + best], bh = out[3 * n + best];
  const cv::Rect box = to_image_rect(cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2, lb, image_size);
  if (box.area() <= 0) return std::nullopt;
  return Detection{box, best_score, best_cls};
}

std::optional<Detection> Detector::detect(const cv::Mat& bgr, Timing* timing) {
  Letterbox lb;

  auto t0 = Clock::now();
  const cv::Mat blob = letterbox_blob(bgr, cv::Size(cfg_.input_w, cfg_.input_h), lb);
  const double pre = ms_since(t0);

  t0 = Clock::now();
  infer(blob, out_buf_, out_shape_);
  const double inf = ms_since(t0);

  t0 = Clock::now();
  auto det = parse_output(out_buf_, out_shape_, lb, bgr.size(), cfg_);
  const double post = ms_since(t0);

  if (timing) *timing = Timing{pre, inf, post};
  return det;
}

}  // namespace perception
