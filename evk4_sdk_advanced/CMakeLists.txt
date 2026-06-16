cmake_minimum_required(VERSION 3.16)
project(evk4_sdk_advanced)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(event_camera_msgs REQUIRED)
find_package(event_camera_codecs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# The Metavision SDK CV module is closed (SDK Pro) and is NOT part of
# openeb_vendor (which ships only the OPEN modules and has no `cv` config
# target). It must be built/installed separately (see docs/sdk/) and pointed at
# explicitly, because a ROS workspace that sourced openeb_vendor puts the
# open-only copy first on CMAKE_PREFIX_PATH:
#
#   colcon build --packages-select evk4_sdk_advanced \
#     --cmake-args -DMetavisionSDK_DIR=$HOME/mv_sdk/lib/cmake/MetavisionSDK
#
# This package is OPTIONAL: when the SDK (with the cv component) is not found we
# skip it cleanly so a normal `colcon build` on a machine without SDK Pro still
# succeeds. SDK 5 splits components as base/core/cv (the old `driver` became
# `stream`, which we do not need -- events arrive over the EventPacket topic).
find_package(MetavisionSDK QUIET COMPONENTS base core cv)
if(NOT MetavisionSDK_FOUND)
  message(WARNING
    "evk4_sdk_advanced: Metavision SDK (with the cv module) not found -- "
    "skipping this optional package. To build it, install SDK Pro and pass "
    "-DMetavisionSDK_DIR=<prefix>/lib/cmake/MetavisionSDK (see docs/sdk/).")
  ament_package()
  return()
endif()

add_library(optical_flow_component SHARED src/optical_flow_node.cpp)
target_compile_features(optical_flow_component PUBLIC cxx_std_17)
ament_target_dependencies(optical_flow_component
  rclcpp rclcpp_components sensor_msgs event_camera_msgs event_camera_codecs
  cv_bridge)
target_link_libraries(optical_flow_component
  MetavisionSDK::base MetavisionSDK::core MetavisionSDK::cv ${OpenCV_LIBS})
# Find the SDK .so at runtime without sourcing the SDK's setup_env.sh.
# INSTALL_RPATH_USE_LINK_PATH bakes the SDK's lib dir (wherever it was found at
# build time) into the component's RPATH; --disable-new-dtags makes that a
# DT_RPATH, which the loader searches BEFORE LD_LIBRARY_PATH -- so the SDK Pro
# 5.x libs win over openeb_vendor's open-only 5.0.0 copy that a sourced ROS
# workspace puts on LD_LIBRARY_PATH (the version-mismatch hazard, see docs/sdk/).
set_target_properties(optical_flow_component PROPERTIES
  INSTALL_RPATH_USE_LINK_PATH TRUE)
target_link_options(optical_flow_component PRIVATE "-Wl,--disable-new-dtags")

# Registers the component and a standalone executable
# (ros2 run evk4_sdk_advanced optical_flow).
rclcpp_components_register_node(optical_flow_component
  PLUGIN "evk4_sdk_advanced::OpticalFlow"
  EXECUTABLE optical_flow)

# Capture the SDK's lib dir (wherever it was found at build time) into the
# package share, so the launch can prepend it to LD_LIBRARY_PATH. The SDK's own
# libraries use RUNPATH, so their transitive deps (e.g. sdk_base) follow
# LD_LIBRARY_PATH -- putting the build tree first keeps the whole process on the
# Pro 5.x libs instead of leaking to openeb_vendor's open-only 5.0.0. This is
# what makes the launch a single command (no `source setup_env.sh`).
file(GENERATE
  OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/sdk_libdir"
  CONTENT "$<TARGET_FILE_DIR:MetavisionSDK::base>")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/sdk_libdir"
  DESTINATION share/${PROJECT_NAME})

install(TARGETS optical_flow_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
