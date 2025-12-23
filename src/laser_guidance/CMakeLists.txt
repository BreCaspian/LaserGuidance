cmake_minimum_required(VERSION 3.8)
project(laser_guidance)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(
  include
)

add_executable(locator_node
  src/locator_node.cpp
  src/camera_model.cpp
  src/image_processor.cpp
  src/plane_fitter.cpp
  src/kalman_3d.cpp
  src/distance_plotter.cpp
)
ament_target_dependencies(locator_node
  rclcpp
  ament_index_cpp
  sensor_msgs
  geometry_msgs
  std_msgs
  cv_bridge
  image_transport
  pcl_conversions
  visualization_msgs
)
target_link_libraries(locator_node ${OpenCV_LIBS})

install(TARGETS locator_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

ament_package()
