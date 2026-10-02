// perception_master: 새 프레임마다 ① Detector → ② Depth Extractor → ③ IMU · Joint Reader 호출
void on_frames(const Image::ConstSharedPtr& color, const Image::ConstSharedPtr& depth) {
  cv::Mat bgr = cv_bridge::toCvShare(color, "bgr8")->image;
  cv::Mat depth_mm = cv_bridge::toCvShare(depth)->image;    // 16UC1, 단위 mm

  auto det = detector_->detect(bgr);
  if (!det) return;   // 발행 안 함 → State Machine이 입력 신선도로 LOST 판단

  auto z = depth_extractor_->median(depth_mm, det->box);    // optional<float> [m]

  geometry_msgs::msg::PointStamped msg;
  msg.header = color->header;    // 촬영 시각 유지 → planning이 지연 시간을 알 수 있음
  float cx = det->box.x + det->box.width / 2.f;
  float cy = det->box.y + det->box.height / 2.f;
  msg.point.x = (cx - bgr.cols / 2.f) / (bgr.cols / 2.f);   // e_x
  msg.point.y = (cy - bgr.rows / 2.f) / (bgr.rows / 2.f);   // e_y
  msg.point.z = z.value_or(std::nanf(""));                  // 깊이 실패 시 NaN
  pub_->publish(msg);
}