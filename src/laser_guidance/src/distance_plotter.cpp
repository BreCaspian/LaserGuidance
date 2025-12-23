// SPDX-License-Identifier: GPL-3.0-only

// Copyright © 2025 YAO YUZHUO (yaoyuzhuo6@gmail.com).
// ROBOMASTER-2026-NCST-HORIZON
// Thu 27 Nov 2025 01∶47∶05 AM CST

#include "laser_guidance/distance_plotter.hpp"

#include <algorithm>

namespace laser_guidance {

DistancePlotter::DistancePlotter(const DistancePlotterConfig &config)
    : config_{config} {}

void DistancePlotter::update(double distance) {
  double smoothed = distance;
  if (!smoothed_.empty()) {
    smoothed =
        config_.smoothing_alpha * distance +
        (1.0 - config_.smoothing_alpha) * smoothed_.back();
  }
  values_.push_back(distance);
  smoothed_.push_back(smoothed);
  if (static_cast<int>(values_.size()) > config_.history_size) {
    values_.pop_front();
    smoothed_.pop_front();
  }
  if (config_.enabled) {
    draw();
  }
}

void DistancePlotter::draw() const {
  if (values_.empty()) {
    return;
  }
  const int width = 600;
  const int height = 250;
  cv::Mat canvas(height, width, CV_8UC3, cv::Scalar(20, 20, 20));
  cv::rectangle(canvas, cv::Rect(0, 0, width - 1, height - 1),
                cv::Scalar(80, 80, 80), 1);

  double min_val = *std::min_element(smoothed_.begin(), smoothed_.end());
  double max_val = *std::max_element(smoothed_.begin(), smoothed_.end());
  if (max_val - min_val < 0.5) {
    double mid = 0.5 * (max_val + min_val);
    min_val = mid - 0.25;
    max_val = mid + 0.25;
  }
  auto transform = [&](double value, size_t idx, size_t total) -> cv::Point {
    double x = static_cast<double>(idx) / std::max<size_t>(1, total - 1);
    double y = (value - min_val) / (max_val - min_val);
    int px = static_cast<int>(x * (width - 20)) + 10;
    int py = height - 10 - static_cast<int>(y * (height - 20));
    return cv::Point(px, py);
  };
  for (size_t i = 1; i < smoothed_.size(); ++i) {
    cv::line(canvas,
             transform(smoothed_[i - 1], i - 1, smoothed_.size()),
             transform(smoothed_[i], i, smoothed_.size()),
             cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
  }
  for (size_t i = 1; i < values_.size(); ++i) {
    cv::line(canvas, transform(values_[i - 1], i - 1, values_.size()),
             transform(values_[i], i, values_.size()),
             cv::Scalar(0, 140, 255), 1, cv::LINE_AA);
  }
  cv::putText(canvas, "Dist: " + std::to_string(values_.back()),
              cv::Point(15, 25), cv::FONT_HERSHEY_SIMPLEX, 0.6,
              cv::Scalar(200, 200, 200), 1);
  cv::imshow(config_.window_name, canvas);
  cv::waitKey(1);
}

}  // namespace laser_guidance
