cmake_minimum_required(VERSION 3.16)
project(evk4_bringup)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_calibration_parsers REQUIRED)

# camera_info publisher as a composable component: loaded into the camera
# container so images arrive intra-process (pointer, no copy) and only the
# header is read.
add_library(camera_info_component SHARED src/camera_info_component.cpp)
ament_target_dependencies(camera_info_component
  rclcpp rclcpp_components sensor_msgs camera_calibration_parsers)
rclcpp_components_register_nodes(camera_info_component
  "evk4_bringup::CameraInfoPublisher")

install(TARGETS camera_info_component DESTINATION lib)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
