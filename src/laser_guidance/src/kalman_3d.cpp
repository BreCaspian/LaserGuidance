// SPDX-License-Identifier: GPL-3.0-only

// Copyright © 2025 YAO YUZHUO (yaoyuzhuo6@gmail.com).
// ROBOMASTER-2026-NCST-HORIZON
// Thu 27 Nov 2025 01∶47∶05 AM CST

#include "laser_guidance/kalman_3d.hpp"

#include <algorithm>

namespace laser_guidance {

Kalman3D::Kalman3D(const Kalman3DConfig &config) : config_{config} {}

void Kalman3D::reset(const Eigen::Vector3d &position) {
  x_.setZero();
  x_.segment<3>(0) = position;
  P_.setZero();
  P_.block<3, 3>(0, 0) =
      Eigen::Matrix3d::Identity() * config_.initial_position_var;
  P_.block<3, 3>(3, 3) =
      Eigen::Matrix3d::Identity() * config_.initial_velocity_var;
  quality_ = 1.0;
  initialized_ = true;
}

Eigen::Vector3d Kalman3D::update(const Eigen::Vector3d &position, double dt,
                                 const MeasurementQuality &quality) {
  if (!initialized_) {
    reset(position);
    return position;
  }
  const double clamped_dt = std::max(1e-3, dt);
  Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity();
  F(0, 3) = clamped_dt;
  F(1, 4) = clamped_dt;
  F(2, 5) = clamped_dt;

  const double dt2 = clamped_dt * clamped_dt;
  const double dt3 = dt2 * clamped_dt;
  const double dt4 = dt2 * dt2;
  const double q = config_.process_noise_accel;
  Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Zero();
  const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  Q.block<3, 3>(0, 0) = I * (dt4 * 0.25 * q);
  Q.block<3, 3>(0, 3) = I * (dt3 * 0.5 * q);
  Q.block<3, 3>(3, 0) = I * (dt3 * 0.5 * q);
  Q.block<3, 3>(3, 3) = I * (dt2 * q);

  x_ = F * x_;
  P_ = F * P_ * F.transpose() + Q;

  const double q_score = computeQuality(quality.inliers, quality.cos_angle);
  const double sigma = computeSigma(q_score);
  const Eigen::Matrix3d R =
      Eigen::Matrix3d::Identity() * sigma * sigma;

  Eigen::Matrix<double, 3, 6> H = Eigen::Matrix<double, 3, 6>::Zero();
  H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  Eigen::Vector3d y = position - H * x_;
  Eigen::Matrix3d S = H * P_ * H.transpose() + R;
  const double nis = y.dot(S.ldlt().solve(y));
  if (nis > config_.gate_nis) {
    return x_.segment<3>(0);
  }
  Eigen::Matrix<double, 6, 3> K = P_ * H.transpose() * S.inverse();
  x_ = x_ + K * y;
  Eigen::Matrix<double, 6, 6> I6 =
      Eigen::Matrix<double, 6, 6>::Identity();
  P_ = (I6 - K * H) * P_;

  return x_.segment<3>(0);
}

double Kalman3D::computeQuality(int inliers, double cos_angle) {
  const int clipped = std::max(config_.inliers_lo,
                               std::min(inliers, config_.inliers_hi));
  const double q_inliers =
      (config_.inliers_hi > config_.inliers_lo)
          ? (static_cast<double>(clipped - config_.inliers_lo) /
             static_cast<double>(config_.inliers_hi - config_.inliers_lo))
          : 1.0;
  const double cos_clamped = std::max(config_.cos_lo, cos_angle);
  const double q_cos = (cos_clamped - config_.cos_lo) /
                       std::max(1e-6, 1.0 - config_.cos_lo);
  const double q = std::max(0.0, std::min(1.0, q_inliers * q_cos));
  // quality_alpha: closer to 1.0 follows new measurements more closely.
  quality_ = config_.quality_alpha * q +
             (1.0 - config_.quality_alpha) * quality_;
  return quality_;
}

double Kalman3D::computeSigma(double quality) const {
  const double t = 1.0 - std::max(0.0, std::min(1.0, quality));
  return config_.sigma_min +
         (config_.sigma_max - config_.sigma_min) * t * t;
}

}  // namespace laser_guidance
