// SPDX-License-Identifier: GPL-3.0-only

// Copyright © 2025 YAO YUZHUO (yaoyuzhuo6@gmail.com).
// ROBOMASTER-2026-NCST-HORIZON
// Thu 27 Nov 2025 01∶47∶05 AM CST

#pragma once

#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <string>

namespace laser_guidance {

struct CameraModel {
  cv::Mat camera_matrix;
  cv::Mat dist_coeffs;
};

bool loadCameraModel(const std::string &path, CameraModel &model);
bool loadExtrinsic(const std::string &path, Eigen::Matrix4d &T_lidar_to_cam);

}  // namespace laser_guidance
