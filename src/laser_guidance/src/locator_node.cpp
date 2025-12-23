// SPDX-License-Identifier: GPL-3.0-only

// Copyright © 2025 YAO YUZHUO (yaoyuzhuo6@gmail.com).
// ROBOMASTER-2026-NCST-HORIZON
// Thu 27 Nov 2025 01∶47∶05 AM CST

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <deque>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <image_transport/image_transport.hpp>
#include <memory>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <utility>
#include <visualization_msgs/msg/marker.hpp>

#include "laser_guidance/camera_model.hpp"
#include "laser_guidance/distance_plotter.hpp"
#include "laser_guidance/image_processor.hpp"
#include "laser_guidance/kalman_3d.hpp"
#include "laser_guidance/plane_fitter.hpp"

using namespace std::chrono_literals;
using laser_guidance::CameraModel;
using laser_guidance::DistancePlotter;
using laser_guidance::DistancePlotterConfig;
using laser_guidance::ImageProcessor;
using laser_guidance::ImageProcessorConfig;
using laser_guidance::ImageResult;
using laser_guidance::PlaneFitter;
using laser_guidance::PlaneFitterConfig;
using laser_guidance::PlaneResult;
using laser_guidance::loadExtrinsic;

class DartGuideLocator : public rclcpp::Node {
 public:
  DartGuideLocator();

 private:
  void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void imgCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  bool computeIntersection(const Eigen::Vector3d &ray_dir_cam,
                           const PlaneResult &plane, Eigen::Vector3d &out) const;
  void publishPlaneMarker(const PlaneResult &plane);

  double accumulate_time_s_{1.0};
  std::string image_topic_;
  std::string lidar_topic_;

  CameraModel camera_model_;
  Eigen::Matrix4d T_lidar_to_cam_{Eigen::Matrix4d::Identity()};
  Eigen::Matrix4d T_cam_to_lidar_{Eigen::Matrix4d::Identity()};

  ImageProcessorConfig image_config_;
  PlaneFitterConfig plane_config_;
  DistancePlotterConfig plot_config_;
  laser_guidance::Kalman3DConfig kalman_config_;
  bool kalman_enabled_{true};
  bool kalman_initialized_{false};
  rclcpp::Time last_kalman_stamp_;

  image_transport::Publisher debug_pub_;
  std::unique_ptr<ImageProcessor> image_processor_;
  std::unique_ptr<PlaneFitter> plane_fitter_;
  std::unique_ptr<DistancePlotter> plotter_;
  laser_guidance::Kalman3D kalman_{laser_guidance::Kalman3DConfig{}};

  std::deque<std::pair<rclcpp::Time, pcl::PointCloud<pcl::PointXYZ>::Ptr>>
      cloud_queue_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pos_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr dist_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr plane_cos_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr plane_inliers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr plane_marker_pub_;
};

DartGuideLocator::DartGuideLocator() : Node("laser_guidance") {
  auto pkg_share = ament_index_cpp::get_package_share_directory("laser_guidance");
  std::string calib_default = pkg_share + "/config/LidarCamera.yaml";
  std::string calib_file =
      this->declare_parameter<std::string>("calib_file", calib_default);
  accumulate_time_s_ =
      this->declare_parameter<double>("accumulate_time_s", 1.0);
  image_topic_ =
      this->declare_parameter<std::string>("image_topic", "/image_raw");
  lidar_topic_ =
      this->declare_parameter<std::string>("lidar_topic", "/livox/lidar");

  image_config_.hsv_lower = cv::Scalar(
      this->declare_parameter<double>("green_h", 40.0),
      this->declare_parameter<double>("green_s", 80.0),
      this->declare_parameter<double>("green_v", 120.0));
  image_config_.hsv_upper = cv::Scalar(
      this->declare_parameter<double>("green_h_max", 90.0),
      this->declare_parameter<double>("green_s_max", 255.0),
      this->declare_parameter<double>("green_v_max", 255.0));
  image_config_.debug_roi_half_size =
      this->declare_parameter<int>("debug_roi_half_size_px", 80);
  image_config_.debug_roi_output_size =
      this->declare_parameter<int>("debug_roi_output_size_px", 256);

  plane_config_.min_depth_m =
      this->declare_parameter<double>("min_depth_m", 8.0);
  plane_config_.max_depth_m =
      this->declare_parameter<double>("max_depth_m", 40.0);
  plane_config_.roi_radius_px =
      this->declare_parameter<int>("roi_radius_px", 80);
  plane_config_.min_plane_normal_z =
      this->declare_parameter<double>("min_plane_normal_z", 0.4);
  plane_config_.min_normal_alignment =
      this->declare_parameter<double>("min_normal_alignment", 0.5);
  plane_config_.distance_threshold =
      this->declare_parameter<double>("plane_distance_threshold", 0.04);

  plot_config_.enabled =
      this->declare_parameter<bool>("plot_window_enabled", true);
  plot_config_.history_size =
      this->declare_parameter<int>("plot_history_size", 200);
  plot_config_.smoothing_alpha =
      this->declare_parameter<double>("plot_smoothing_alpha", 0.3);

  kalman_enabled_ =
      this->declare_parameter<bool>("kalman_enabled", true);
  kalman_config_.process_noise_accel =
      this->declare_parameter<double>("kalman_process_noise", 1.0);
  kalman_config_.initial_position_var =
      this->declare_parameter<double>("kalman_initial_position_var", 0.1);
  kalman_config_.initial_velocity_var =
      this->declare_parameter<double>("kalman_initial_velocity_var", 1.0);
  kalman_config_.sigma_min =
      this->declare_parameter<double>("kalman_sigma_min", 0.03);
  kalman_config_.sigma_max =
      this->declare_parameter<double>("kalman_sigma_max", 0.2);
  kalman_config_.inliers_lo =
      this->declare_parameter<int>("kalman_quality_inliers_lo", 80);
  kalman_config_.inliers_hi =
      this->declare_parameter<int>("kalman_quality_inliers_hi", 400);
  kalman_config_.cos_lo =
      this->declare_parameter<double>("kalman_quality_cos_lo", 0.25);
  kalman_config_.quality_alpha =
      this->declare_parameter<double>("kalman_quality_alpha", 0.3);
  kalman_config_.gate_nis =
      this->declare_parameter<double>("kalman_gate_nis", 11.3);
  kalman_ = laser_guidance::Kalman3D(kalman_config_);

  if (!loadCameraModel(calib_file, camera_model_) ||
      !loadExtrinsic(calib_file, T_lidar_to_cam_)) {
    throw std::runtime_error("Failed to load calibration file " + calib_file);
  }
  T_cam_to_lidar_ = T_lidar_to_cam_.inverse();

  debug_pub_ = image_transport::create_publisher(this, "debug_image");
  image_processor_ =
      std::make_unique<ImageProcessor>(camera_model_, image_config_, &debug_pub_);
  plane_fitter_ = std::make_unique<PlaneFitter>(plane_config_, camera_model_,
                                                T_lidar_to_cam_);
  plotter_ = std::make_unique<DistancePlotter>(plot_config_);

  auto img_qos = rclcpp::SensorDataQoS();
  img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic_, img_qos,
      std::bind(&DartGuideLocator::imgCallback, this, std::placeholders::_1));

  auto lidar_qos = rclcpp::QoS(rclcpp::KeepLast(5));
  lidar_qos.reliable().durability_volatile();
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      lidar_topic_, lidar_qos,
      std::bind(&DartGuideLocator::lidarCallback, this, std::placeholders::_1));

  pos_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
      "/guide_light/position", 10);
  dist_pub_ =
      this->create_publisher<std_msgs::msg::Float32>("/guide_light/distance",
                                                     10);
  plane_cos_pub_ =
      this->create_publisher<std_msgs::msg::Float32>("/guide_light/plane_cos",
                                                     10);
  plane_inliers_pub_ =
      this->create_publisher<std_msgs::msg::UInt32>(
          "/guide_light/plane_inliers", 10);
  plane_marker_pub_ =
      this->create_publisher<visualization_msgs::msg::Marker>(
          "/plane_marker", 10);

  RCLCPP_INFO(this->get_logger(), "laser_guidance ready. Accumulate %.2fs",
              accumulate_time_s_);
}

void DartGuideLocator::lidarCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  auto cloud_in = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromROSMsg(*msg, *cloud_in);
  rclcpp::Time stamp(msg->header.stamp);
  while (!cloud_queue_.empty()) {
    if ((stamp - cloud_queue_.front().first).seconds() > accumulate_time_s_) {
      cloud_queue_.pop_front();
    } else {
      break;
    }
  }
  cloud_queue_.push_back({stamp, cloud_in});
}

void DartGuideLocator::imgCallback(
    const sensor_msgs::msg::Image::SharedPtr msg) {
  if (cloud_queue_.empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Waiting for point cloud...");
    return;
  }

  ImageResult image_result = image_processor_->process(msg);
  if (!image_result.success) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Green light NOT detected.");
    return;
  }

  auto accumulated_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  for (const auto &entry : cloud_queue_) {
    *accumulated_cloud += *entry.second;
  }

  PlaneResult plane =
      plane_fitter_->fit(accumulated_cloud, image_result.uv,
                         image_result.ray_dir);
  if (!plane.success) {
    return;
  }

  if (plane_cos_pub_) {
    std_msgs::msg::Float32 cos_msg;
    cos_msg.data = static_cast<float>(plane.cos_angle);
    plane_cos_pub_->publish(cos_msg);
  }
  if (plane_inliers_pub_) {
    std_msgs::msg::UInt32 inliers_msg;
    inliers_msg.data = static_cast<uint32_t>(plane.inliers);
    plane_inliers_pub_->publish(inliers_msg);
  }

  Eigen::Vector3d intersection_lidar;
  if (!computeIntersection(image_result.ray_dir, plane, intersection_lidar)) {
    return;
  }

  Eigen::Vector3d filtered_lidar = intersection_lidar;
  if (kalman_enabled_) {
    rclcpp::Time stamp(msg->header.stamp);
    if (!kalman_initialized_) {
      kalman_.reset(intersection_lidar);
      last_kalman_stamp_ = stamp;
      kalman_initialized_ = true;
    } else {
      double dt = (stamp - last_kalman_stamp_).seconds();
      if (dt <= 0.0 || dt > 1.0) {
        kalman_.reset(intersection_lidar);
      } else {
        filtered_lidar =
            kalman_.update(intersection_lidar, dt,
                           {static_cast<int>(plane.inliers), plane.cos_angle});
      }
      last_kalman_stamp_ = stamp;
    }
  }

  double distance = filtered_lidar.norm();
  RCLCPP_INFO(this->get_logger(),
              ">>> Target Localized! <<< Pos [%.3f, %.3f, %.3f] m | "
              "Distance: %.3f m",
              filtered_lidar.x(), filtered_lidar.y(),
              filtered_lidar.z(), distance);

  geometry_msgs::msg::PointStamped pos_msg;
  pos_msg.header = msg->header;
  pos_msg.header.frame_id = "livox_frame";
  pos_msg.point.x = filtered_lidar.x();
  pos_msg.point.y = filtered_lidar.y();
  pos_msg.point.z = filtered_lidar.z();
  pos_pub_->publish(pos_msg);

  std_msgs::msg::Float32 dist_msg;
  dist_msg.data = static_cast<float>(distance);
  dist_pub_->publish(dist_msg);

  plotter_->update(distance);
  publishPlaneMarker(plane);
}

bool DartGuideLocator::computeIntersection(const Eigen::Vector3d &ray_dir_cam,
                                           const PlaneResult &plane,
                                           Eigen::Vector3d &out) const {
  if (!plane.coefficients) {
    return false;
  }
  Eigen::Vector3d n(plane.coefficients->values[0],
                    plane.coefficients->values[1],
                    plane.coefficients->values[2]);
  double d = plane.coefficients->values[3];
  double denom = n.dot(ray_dir_cam);
  if (std::abs(denom) < 1e-6) {
    return false;
  }
  double t = -d / denom;
  if (t < 0.0) {
    return false;
  }
  Eigen::Vector3d intersection_cam = t * ray_dir_cam;
  Eigen::Vector4d homo(intersection_cam.x(), intersection_cam.y(),
                       intersection_cam.z(), 1.0);
  out = (T_cam_to_lidar_ * homo).head<3>();
  return true;
}

void DartGuideLocator::publishPlaneMarker(const PlaneResult &plane) {
  if (!plane_marker_pub_ || !plane.coefficients) {
    return;
  }
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "livox_frame";
  marker.header.stamp = this->now();
  marker.ns = "plane_visualization";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = marker.scale.y = marker.scale.z = 1.0;
  marker.color.a = 0.35;
  marker.color.r = 0.0;
  marker.color.g = 0.5;
  marker.color.b = 1.0;

  Eigen::Vector3d center_lidar =
      (T_cam_to_lidar_ *
       Eigen::Vector4d(plane.center_cam.x(), plane.center_cam.y(),
                       plane.center_cam.z(), 1.0))
          .head<3>();
  Eigen::Vector3d normal_lidar =
      (T_cam_to_lidar_.block<3, 3>(0, 0) * plane.normal_cam).normalized();
  Eigen::Vector3d basis_u =
      normal_lidar.cross(Eigen::Vector3d::UnitZ());
  if (basis_u.norm() < 1e-3) {
    basis_u = normal_lidar.cross(Eigen::Vector3d::UnitY());
  }
  basis_u.normalize();
  Eigen::Vector3d basis_v = normal_lidar.cross(basis_u);
  const double half = 2.0;
  std::vector<Eigen::Vector3d> corners = {
      center_lidar + half * (basis_u + basis_v),
      center_lidar + half * (basis_u - basis_v),
      center_lidar + half * (-basis_u - basis_v),
      center_lidar + half * (-basis_u + basis_v)};
  auto push_triangle = [&](const Eigen::Vector3d &a, const Eigen::Vector3d &b,
                           const Eigen::Vector3d &c) {
    geometry_msgs::msg::Point pa, pb, pc;
    pa.x = a.x();
    pa.y = a.y();
    pa.z = a.z();
    pb.x = b.x();
    pb.y = b.y();
    pb.z = b.z();
    pc.x = c.x();
    pc.y = c.y();
    pc.z = c.z();
    marker.points.push_back(pa);
    marker.points.push_back(pb);
    marker.points.push_back(pc);
  };
  push_triangle(corners[0], corners[1], corners[2]);
  push_triangle(corners[2], corners[3], corners[0]);
  plane_marker_pub_->publish(marker);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DartGuideLocator>());
  rclcpp::shutdown();
  return 0;
}
