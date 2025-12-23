// SPDX-License-Identifier: GPL-3.0-only

// Copyright © 2025 YAO YUZHUO (yaoyuzhuo6@gmail.com).
// ROBOMASTER-2026-NCST-HORIZON
// Thu 27 Nov 2025 01∶47∶05 AM CST

#pragma once

#include <Eigen/Dense>

namespace laser_guidance {

struct Kalman3DConfig {
  double process_noise_accel{1.0};
  double initial_position_var{0.1};
  double initial_velocity_var{1.0};
  double sigma_min{0.03};
  double sigma_max{0.2};
  int inliers_lo{80};
  int inliers_hi{400};
  double cos_lo{0.25};
  double quality_alpha{0.3};
  double gate_nis{11.3};
};

struct MeasurementQuality {
  int inliers{0};
  double cos_angle{0.0};
};

class Kalman3D {
 public:
  explicit Kalman3D(const Kalman3DConfig &config);

  bool initialized() const { return initialized_; }
  void reset(const Eigen::Vector3d &position);
  Eigen::Vector3d update(const Eigen::Vector3d &position, double dt,
                         const MeasurementQuality &quality);

 private:
  double computeQuality(int inliers, double cos_angle);
  double computeSigma(double quality) const;

  Kalman3DConfig config_;
  Eigen::Matrix<double, 6, 1> x_{Eigen::Matrix<double, 6, 1>::Zero()};
  Eigen::Matrix<double, 6, 6> P_{Eigen::Matrix<double, 6, 6>::Identity()};
  bool initialized_{false};
  double quality_{1.0};
};

}  // namespace laser_guidance
