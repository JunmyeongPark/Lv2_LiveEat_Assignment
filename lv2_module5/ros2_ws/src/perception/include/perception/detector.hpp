// Detector (YOLO26): bbox 중심 e_x, e_y
// detector.hpp
struct Detection { cv::Rect box; float score; };

class Detector {
public:
  Detector(const std::string& param, const std::string& bin,
           int input_size, float conf, int threads);
  std::optional<Detection> detect(const cv::Mat& bgr);
private:
  ncnn::Net net_;
  int size_;
  float conf_;
};