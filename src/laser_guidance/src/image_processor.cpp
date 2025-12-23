// SPDX-License-Identifier: GPL-3.0-only

// Copyright © 2025 YAO YUZHUO (yaoyuzhuo6@gmail.com).
// ROBOMASTER-2026-NCST-HORIZON
// Thu 27 Nov 2025 01∶47∶05 AM CST

#include "laser_guidance/image_processor.hpp"

#include <algorithm>
#include <vector>

namespace laser_guidance {

ImageProcessor::ImageProcessor(const CameraModel &model,
                               const ImageProcessorConfig &config,
                               image_transport::Publisher *debug_pub)
    : camera_model_{model},
      config_{config},
      debug_pub_{debug_pub} {}

ImageResult ImageProcessor::process(
    const sensor_msgs::msg::Image::SharedPtr &msg) const {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (const cv_bridge::Exception &) {
    return {};
  }

  ImageResult result = detectGreenLight(cv_ptr->image);
  if (!result.success) {
    return result;
  }

  std::vector<cv::Point2f> src_pts{result.uv};
  std::vector<cv::Point2f> dst_pts;
  cv::undistortPoints(src_pts, dst_pts, camera_model_.camera_matrix,
                      camera_model_.dist_coeffs);

  result.ray_dir = Eigen::Vector3d(dst_pts[0].x, dst_pts[0].y, 1.0).normalized();

  if (debug_pub_) {
    publishDebug(msg, result.uv);
  }

  return result;
}

ImageResult ImageProcessor::detectGreenLight(const cv::Mat &image) const {
  ImageResult result;
  cv::Mat hsv, mask;
  cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
  cv::inRange(hsv, config_.hsv_lower, config_.hsv_upper, mask);
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, {3, 3});
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
  cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);

  double max_area = 0.0;
  bool found = false;
  for (const auto &cnt : contours) {
    double area = cv::contourArea(cnt);
    if (area < 10.0 || area > 2000.0) {
      continue;
    }
    double perimeter = cv::arcLength(cnt, true);
    if (perimeter <= 0.0) {
      continue;
    }
    double circularity = 4.0 * CV_PI * area / (perimeter * perimeter);
    if (circularity < 0.5) {
      continue;
    }
    cv::Moments m = cv::moments(cnt);
    if (m.m00 <= 0.0) {
      continue;
    }
    if (area > max_area) {
      max_area = area;
      result.uv = cv::Point2f(static_cast<float>(m.m10 / m.m00),
                              static_cast<float>(m.m01 / m.m00));
      found = true;
    }
  }

  if (!found) {
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    double min_val, max_val;
    cv::Point min_loc, max_loc;
    cv::minMaxLoc(gray, &min_val, &max_val, &min_loc, &max_loc);
    if (max_val > 200) {
      result.uv = cv::Point2f(static_cast<float>(max_loc.x),
                              static_cast<float>(max_loc.y));
      found = true;
    }
  }

  result.success = found;
  return result;
}

void ImageProcessor::publishDebug(
    const sensor_msgs::msg::Image::SharedPtr &msg,
    const cv::Point2f &center) const {
  cv_bridge::CvImagePtr cv_ptr =
      cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  const int half = std::max(10, config_.debug_roi_half_size);
  const int img_w = cv_ptr->image.cols;
  const int img_h = cv_ptr->image.rows;

  int x0 = std::clamp(static_cast<int>(center.x) - half, 0, std::max(0, img_w - 1));
  int y0 = std::clamp(static_cast<int>(center.y) - half, 0, std::max(0, img_h - 1));
  int x1 = std::clamp(static_cast<int>(center.x) + half, x0 + 1, img_w);
  int y1 = std::clamp(static_cast<int>(center.y) + half, y0 + 1, img_h);

  cv::Rect roi(cv::Point(x0, y0), cv::Point(x1, y1));
  cv::Mat roi_img = cv_ptr->image(roi).clone();
  cv::Mat roi_norm;
  roi_img.convertTo(roi_norm, CV_32F);
  cv::normalize(roi_norm, roi_norm, 0.0, 255.0, cv::NORM_MINMAX);
  roi_norm.convertTo(roi_norm, CV_8U);
  
  if (config_.debug_roi_output_size > 0) {
    cv::resize(roi_norm, roi_norm,
               cv::Size(config_.debug_roi_output_size,
                        config_.debug_roi_output_size),
               0, 0, cv::INTER_LINEAR);
  }
  cv::Point center_draw(roi_norm.cols / 2, roi_norm.rows / 2);
  cv::drawMarker(roi_norm, center_draw, cv::Scalar(0, 0, 255),
                 cv::MARKER_CROSS, 30, 2);
  cv::circle(roi_norm, center_draw, 8, cv::Scalar(0, 255, 0), 2);
  auto debug_msg = cv_bridge::CvImage(msg->header, "bgr8", roi_norm).toImageMsg();
  debug_pub_->publish(debug_msg);
}

}  // namespace laser_guidance
