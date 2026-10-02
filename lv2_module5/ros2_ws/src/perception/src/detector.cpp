// Detector (YOLO26): bbox 중심 e_x, e_y
// detector.cpp
Detector::Detector(const std::string& param, const std::string& bin,
                   int input_size, float conf, int threads)
: size_(input_size), conf_(conf) {
  net_.opt.num_threads = threads;           // Pi4: 4
  net_.opt.use_vulkan_compute = false;      // Pi4 GPU는 느려서 CPU만 사용
  net_.load_param(param.c_str());
  net_.load_model(bin.c_str());
}

std::optional<Detection> Detector::detect(const cv::Mat& bgr) {
  // 1. letterbox: 비율 유지 리사이즈 + 114(회색) 패딩
  float scale = std::min(size_ / float(bgr.cols), size_ / float(bgr.rows));
  int w = std::round(bgr.cols * scale), h = std::round(bgr.rows * scale);
  int px = (size_ - w) / 2, py = (size_ - h) / 2;

  ncnn::Mat in = ncnn::Mat::from_pixels_resize(
      bgr.data, ncnn::Mat::PIXEL_BGR2RGB, bgr.cols, bgr.rows, w, h);
  ncnn::Mat padded;
  ncnn::copy_make_border(in, padded, py, size_ - h - py, px, size_ - w - px,
                         ncnn::BORDER_CONSTANT, 114.f);
  const float norm[3] = {1 / 255.f, 1 / 255.f, 1 / 255.f};
  padded.substract_mean_normalize(nullptr, norm);

  // 2. 추론: out = 5 x 8400 (행: cx, cy, w, h, score)
  ncnn::Extractor ex = net_.create_extractor();
  ex.input("in0", padded);
  ncnn::Mat out;
  ex.extract("out0", out);

  // 3. 박스 하나만 쓰므로 NMS 대신 최고 score 1개 선택
  const float* score = out.row(4);
  int best = -1;
  float best_s = conf_;
  for (int i = 0; i < out.w; ++i)
    if (score[i] > best_s) { best_s = score[i]; best = i; }
  if (best < 0) return std::nullopt;

  // 4. letterbox 되돌려 원본 좌표로
  float cx = out.row(0)[best], cy = out.row(1)[best];
  float bw = out.row(2)[best], bh = out.row(3)[best];
  cv::Rect box(cv::Point((cx - bw / 2 - px) / scale, (cy - bh / 2 - py) / scale),
               cv::Point((cx + bw / 2 - px) / scale, (cy + bh / 2 - py) / scale));
  return Detection{box & cv::Rect(0, 0, bgr.cols, bgr.rows), best_s};
}