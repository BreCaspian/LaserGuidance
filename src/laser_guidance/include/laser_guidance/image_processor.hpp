// SPDX-License-Identifier: GPL-3.0-only

// Copyright © 2025 YAO YUZHUO (yaoyuzhuo6@gmail.com).
// ROBOMASTER-2026-NCST-HORIZON
// Thu 27 Nov 2025 01∶47∶05 AM CST

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "laser_guidance/camera_model.hpp"

namespace laser_guidance {

struct ImageProcessorConfig {
  cv::Scalar hsv_lower{40.0, 80.0, 120.0};
  cv::Scalar hsv_upper{90.0, 255.0, 255.0};
  int debug_roi_half_size{80};
  int debug_roi_output_size{256};
};

struct ImageResult {
  bool success{false};
  Eigen::Vector3d ray_dir{0, 0, 1};
  cv::Point2f uv{0, 0};
};

class ImageProcessor {
 public:
  ImageProcessor(const CameraModel &model, const ImageProcessorConfig &config,
                 image_transport::Publisher *debug_pub);

  ImageResult process(const sensor_msgs::msg::Image::SharedPtr &msg) const;

 private:
  ImageResult detectGreenLight(const cv::Mat &image) const;
  void publishDebug(const sensor_msgs::msg::Image::SharedPtr &msg,
                    const cv::Point2f &center) const;

  CameraModel camera_model_;
  ImageProcessorConfig config_;
  image_transport::Publisher *debug_pub_{nullptr};
};

}  // namespace laser_guidance
