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

# The Metavision SDK modules are closed (SDK Pro) and NOT part of openeb_vendor.
# They must be built/installed separately (see docs/sdk/) and pointed at
# explicitly, because a ROS workspace that sourced openeb_vendor puts the
# open-only copy first on CMAKE_PREFIX_PATH:
#
#   colcon build --packages-select evk4_sdk_advanced \
#     --cmake-args -DMetavisionSDK_DIR=$HOME/metavision_src/openeb-5.3.1/build/generated/share/cmake/MetavisionSDKCMakePackagesFilesDir
#
# This package is OPTIONAL: when the SDK (with the needed modules) is not found
# we skip it so a normal `colcon build` without SDK Pro still succeeds.
find_package(MetavisionSDK QUIET COMPONENTS base core cv analytics)
if(NOT MetavisionSDK_FOUND)
  message(WARNING
    "evk4_sdk_advanced: Metavision SDK (base/core/cv/analytics) not found -- "
    "skipping this optional package. To build it, install SDK Pro and pass "
    "-DMetavisionSDK_DIR=<prefix>/lib/cmake/MetavisionSDK (see docs/sdk/).")
  ament_package()
  return()
endif()

# Each pipeline is a composable component + standalone executable, sharing the
# real-time harness in include/evk4_sdk_advanced/event_vision_node.hpp. Extra
# args are the MetavisionSDK::* modules that pipeline needs.
function(add_sdk_component name src plugin exec)
  add_library(${name} SHARED ${src})
  target_compile_features(${name} PUBLIC cxx_std_17)
  target_include_directories(${name} PRIVATE include)
  ament_target_dependencies(${name}
    rclcpp rclcpp_components sensor_msgs event_camera_msgs event_camera_codecs cv_bridge)
  target_link_libraries(${name} ${ARGN} ${OpenCV_LIBS})
  # Find the SDK .so at runtime via a DT_RPATH that beats LD_LIBRARY_PATH; the
  # launch also sets LD_LIBRARY_PATH (the SDK's libs use RUNPATH). See docs/sdk/.
  set_target_properties(${name} PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
  target_link_options(${name} PRIVATE "-Wl,--disable-new-dtags")
  rclcpp_components_register_node(${name} PLUGIN "${plugin}" EXECUTABLE ${exec})
  install(TARGETS ${name}
    ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
endfunction()

add_sdk_component(optical_flow_component src/optical_flow_node.cpp
  "evk4_sdk_advanced::OpticalFlow" optical_flow
  MetavisionSDK::base MetavisionSDK::core MetavisionSDK::cv)

add_sdk_component(tracking_component src/tracking_node.cpp
  "evk4_sdk_advanced::Tracking" tracking
  MetavisionSDK::base MetavisionSDK::core MetavisionSDK::analytics)

add_sdk_component(dense_flow_component src/dense_flow_node.cpp
  "evk4_sdk_advanced::DenseFlow" dense_flow
  MetavisionSDK::base MetavisionSDK::core MetavisionSDK::cv)

add_sdk_component(spatter_component src/spatter_node.cpp
  "evk4_sdk_advanced::SpatterTracking" spatter
  MetavisionSDK::base MetavisionSDK::core MetavisionSDK::analytics)

add_sdk_component(counting_component src/counting_node.cpp
  "evk4_sdk_advanced::Counting" counting
  MetavisionSDK::base MetavisionSDK::core MetavisionSDK::analytics)

add_sdk_component(frequency_component src/frequency_node.cpp
  "evk4_sdk_advanced::Frequency" frequency
  MetavisionSDK::base MetavisionSDK::core MetavisionSDK::analytics MetavisionSDK::cv)

add_sdk_component(led_component src/led_node.cpp
  "evk4_sdk_advanced::LedTracking" led_tracking
  MetavisionSDK::base MetavisionSDK::core MetavisionSDK::cv)

add_sdk_component(psm_component src/psm_node.cpp
  "evk4_sdk_advanced::Psm" psm
  MetavisionSDK::base MetavisionSDK::core MetavisionSDK::analytics)

add_sdk_component(jet_component src/jet_node.cpp
  "evk4_sdk_advanced::JetMonitoring" jet_monitoring
  MetavisionSDK::base MetavisionSDK::core MetavisionSDK::analytics)

# Capture the SDK's lib dir (wherever it was found at build time) into the
# package share, so the launches can prepend it to LD_LIBRARY_PATH -- the SDK's
# own libraries use RUNPATH, so their transitive deps follow LD_LIBRARY_PATH.
# Removes the need to source setup_env.sh.
file(GENERATE
  OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/sdk_libdir"
  CONTENT "$<TARGET_FILE_DIR:MetavisionSDK::base>")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/sdk_libdir"
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
