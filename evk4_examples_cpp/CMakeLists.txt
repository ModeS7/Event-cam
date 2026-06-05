cmake_minimum_required(VERSION 3.16)
project(evk4_examples_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(event_camera_msgs REQUIRED)
find_package(event_camera_codecs REQUIRED)

add_library(event_rate_component SHARED src/event_rate.cpp)
ament_target_dependencies(event_rate_component
  rclcpp rclcpp_components event_camera_msgs event_camera_codecs)

# Registers the component and generates the standalone executable
# (ros2 run evk4_examples_cpp event_rate).
rclcpp_components_register_node(event_rate_component
  PLUGIN "evk4_examples_cpp::EventRate"
  EXECUTABLE event_rate)

install(TARGETS event_rate_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_package()
