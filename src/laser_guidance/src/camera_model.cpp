// SPDX-License-Identifier: GPL-3.0-only

// Copyright © 2025 YAO YUZHUO (yaoyuzhuo6@gmail.com).
// ROBOMASTER-2026-NCST-HORIZON
// Thu 27 Nov 2025 01∶47∶05 AM CST

#include "laser_guidance/camera_model.hpp"

#include <rclcpp/rclcpp.hpp>

namespace laser_guidance {

bool loadCameraModel(const std::string &path, CameraModel &model) {
  cv::FileStorage fs(path, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    return false;
  }
  fs["K_0"] >> model.camera_matrix;
  fs["C_0"] >> model.dist_coeffs;
  return !model.camera_matrix.empty() && !model.dist_coeffs.empty();
}

bool loadExtrinsic(const std::string &path, Eigen::Matrix4d &T_lidar_to_cam) {
  cv::FileStorage fs(path, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    return false;
  }
  cv::Mat mat;
  fs["E_0"] >> mat;
  if (mat.empty() || mat.rows != 4 || mat.cols != 4) {
    return false;
  }
  T_lidar_to_cam.setZero();
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      T_lidar_to_cam(r, c) = mat.at<double>(r, c);
    }
  }
  return true;
}

}  // namespace laser_guidance
