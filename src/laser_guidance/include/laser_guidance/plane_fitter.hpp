// SPDX-License-Identifier: GPL-3.0-only

// Copyright © 2025 YAO YUZHUO (yaoyuzhuo6@gmail.com).
// ROBOMASTER-2026-NCST-HORIZON
// Thu 27 Nov 2025 01∶47∶05 AM CST

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "laser_guidance/camera_model.hpp"

namespace laser_guidance {

struct PlaneFitterConfig {
  double min_depth_m{8.0};
  double max_depth_m{40.0};
  int roi_radius_px{80};
  double min_plane_normal_z{0.4};
  double min_normal_alignment{0.5};
  double distance_threshold{0.04};
};

struct PlaneResult {
  bool success{false};
  pcl::ModelCoefficients::Ptr coefficients;
  Eigen::Vector3d center_cam{0, 0, 0};
  Eigen::Vector3d normal_cam{0, 0, 1};
  double cos_angle{0.0};
  size_t inliers{0};
};

class PlaneFitter {
 public:
  PlaneFitter(const PlaneFitterConfig &config,
              const CameraModel &camera_model,
              const Eigen::Matrix4d &T_lidar_to_cam);

  PlaneResult fit(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
                  const cv::Point2f &target_uv,
                  const Eigen::Vector3d &ray_dir_cam) const;

 private:
  void filterPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_cam,
                    const cv::Point2f &target_uv,
                    pcl::PointIndices::Ptr &indices) const;

  PlaneFitterConfig config_;
  CameraModel camera_model_;
  Eigen::Matrix4d T_lidar_to_cam_;
  mutable Eigen::Vector3d last_normal_{0.0, 0.0, 1.0};
  mutable bool has_last_normal_{false};
};

}  // namespace laser_guidance
