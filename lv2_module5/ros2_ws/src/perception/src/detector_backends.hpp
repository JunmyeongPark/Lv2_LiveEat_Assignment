// 백엔드 생성 함수 (패키지 내부용)
#pragma once

#include <memory>

#include "perception/detector.hpp"

namespace perception {

#ifdef PERCEPTION_WITH_NCNN
std::unique_ptr<Detector> make_ncnn_detector(const DetectorConfig& cfg);
#endif

#ifdef PERCEPTION_WITH_ONNX
std::unique_ptr<Detector> make_onnx_detector(const DetectorConfig& cfg);
#endif

}  // namespace perception
