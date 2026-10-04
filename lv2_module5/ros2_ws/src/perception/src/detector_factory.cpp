#include <stdexcept>

#include "detector_backends.hpp"

namespace perception {

std::unique_ptr<Detector> make_detector(const DetectorConfig& cfg) {
  if (cfg.backend == "ncnn") {
#ifdef PERCEPTION_WITH_NCNN
    return make_ncnn_detector(cfg);
#else
    throw std::runtime_error("perception was built without NCNN (PERCEPTION_WITH_NCNN=OFF)");
#endif
  }
  if (cfg.backend == "onnx") {
#ifdef PERCEPTION_WITH_ONNX
    return make_onnx_detector(cfg);
#else
    throw std::runtime_error("perception was built without ONNX Runtime (PERCEPTION_WITH_ONNX=OFF)");
#endif
  }
  throw std::invalid_argument("unknown detector backend: " + cfg.backend + " (use ncnn | onnx)");
}

}  // namespace perception
