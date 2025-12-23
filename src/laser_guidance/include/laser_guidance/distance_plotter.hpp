// SPDX-License-Identifier: GPL-3.0-only

// Copyright © 2025 YAO YUZHUO (yaoyuzhuo6@gmail.com).
// ROBOMASTER-2026-NCST-HORIZON
// Thu 27 Nov 2025 01∶47∶05 AM CST

#pragma once

#include <opencv2/opencv.hpp>
#include <deque>
#include <string>

namespace laser_guidance {

struct DistancePlotterConfig {
  int history_size{200};
  double smoothing_alpha{0.3};
  bool enabled{true};
  std::string window_name{"Guide Distance Plot"};
};

class DistancePlotter {
 public:
  explicit DistancePlotter(const DistancePlotterConfig &config);

  void update(double distance);

 private:
  void draw() const;

  DistancePlotterConfig config_;
  std::deque<double> values_;
  std::deque<double> smoothed_;
};

}  // namespace laser_guidance
