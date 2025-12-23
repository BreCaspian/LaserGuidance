// SPDX-License-Identifier: GPL-3.0-only

// Copyright © 2025 YAO YUZHUO (yaoyuzhuo6@gmail.com).
// ROBOMASTER-2026-NCST-HORIZON
// Thu 27 Nov 2025 01∶47∶05 AM CST

#include "laser_guidance/plane_fitter.hpp"

#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace laser_guidance {

PlaneFitter::PlaneFitter(const PlaneFitterConfig &config,
                         const CameraModel &camera_model,
                         const Eigen::Matrix4d &T_lidar_to_cam)
    : config_{config},
      camera_model_{camera_model},
      T_lidar_to_cam_{T_lidar_to_cam} {}

PlaneResult PlaneFitter::fit(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
    const cv::Point2f &target_uv, const Eigen::Vector3d &ray_dir_cam) const {
  PlaneResult result;
  if (!cloud_in || cloud_in->empty()) {
    return result;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cam(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*cloud_in, *cloud_cam,
                           T_lidar_to_cam_.cast<float>());

  std::vector<cv::Point2f> src_pts{target_uv};
  std::vector<cv::Point2f> dst_pts;
  cv::undistortPoints(src_pts, dst_pts, camera_model_.camera_matrix,
                      camera_model_.dist_coeffs, cv::noArray(),
                      camera_model_.camera_matrix);

  pcl::PointIndices::Ptr indices(new pcl::PointIndices);
  filterPoints(cloud_cam, dst_pts[0], indices);
  if (indices->indices.size() < 50) {
    return result;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr roi_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloud_cam);
  extract.setIndices(indices);
  extract.setNegative(false);
  extract.filter(*roi_cloud);

  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(config_.distance_threshold);
  seg.setInputCloud(roi_cloud);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  result.coefficients.reset(new pcl::ModelCoefficients);
  seg.segment(*inliers, *(result.coefficients));
  result.inliers = inliers->indices.size();
  if (result.inliers == 0) {
    result.coefficients.reset();
    return result;
  }

  result.center_cam.setZero();
  for (const auto idx : inliers->indices) {
    const auto &pt = (*roi_cloud)[idx];
    result.center_cam.x() += pt.x;
    result.center_cam.y() += pt.y;
    result.center_cam.z() += pt.z;
  }
  result.center_cam /= static_cast<double>(inliers->indices.size());

  result.normal_cam =
      Eigen::Vector3d(result.coefficients->values[0],
                      result.coefficients->values[1],
                      result.coefficients->values[2]);
  if (result.normal_cam.norm() < 1e-6) {
    result.coefficients.reset();
    return result;
  }
  result.normal_cam.normalize();
  if (result.normal_cam.z() < 0.0) {
    result.normal_cam = -result.normal_cam;
    for (auto &value : result.coefficients->values) {
      value = -value;
    }
  }
  if (result.normal_cam.z() < config_.min_plane_normal_z) {
    result.coefficients.reset();
    return result;
  }
  if (has_last_normal_) {
    double align = result.normal_cam.dot(last_normal_);
    if (align < config_.min_normal_alignment) {
      result.coefficients.reset();
      return result;
    }
  }
  last_normal_ = result.normal_cam;
  has_last_normal_ = true;
  result.cos_angle = std::abs(result.normal_cam.dot(ray_dir_cam));
  result.success = true;
  return result;
}

void PlaneFitter::filterPoints(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_cam,
    const cv::Point2f &target_uv, pcl::PointIndices::Ptr &indices) const {
  for (size_t i = 0; i < cloud_cam->points.size(); ++i) {
    const auto &pt = cloud_cam->points[i];
    if (pt.z <= 0.0) {
      continue;
    }
    if (pt.z < config_.min_depth_m || pt.z > config_.max_depth_m) {
      continue;
    }
    double u = camera_model_.camera_matrix.at<double>(0, 0) * pt.x / pt.z +
               camera_model_.camera_matrix.at<double>(0, 2);
    double v = camera_model_.camera_matrix.at<double>(1, 1) * pt.y / pt.z +
               camera_model_.camera_matrix.at<double>(1, 2);
    double du = u - target_uv.x;
    double dv = v - target_uv.y;
    if ((du * du + dv * dv) <= config_.roi_radius_px * config_.roi_radius_px) {
      indices->indices.push_back(static_cast<int>(i));
    }
  }
}

}  // namespace laser_guidance
