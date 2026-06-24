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
find_package(camera_calibration_parsers REQUIRED)
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
#
# The MetavisionSDK `stream` module -- pulled in transitively by cv/analytics (and
# by ml and cv3d below) -- does find_package(hdf5_ecf). A no-install SDK build (the
# one we use) never exports a hdf5_ecfConfig.cmake (only an install-relative targets
# fragment), so find_package(hdf5_ecf) fails and EVERY MetavisionSDK find here would
# come up empty -> the whole package would skip. Synthesize a config pointing the
# imported target at the built codec .so (derived from MetavisionSDK_DIR =
# <build>/generated/share/cmake/MetavisionSDKCMakePackagesFilesDir). This MUST run
# before the find below and regardless of Torch (it is needed on a non-ML box too).
if(MetavisionSDK_DIR AND NOT hdf5_ecf_DIR)
  get_filename_component(_sdk_build "${MetavisionSDK_DIR}/../../../.." ABSOLUTE)
  set(_ecf_so "${_sdk_build}/lib/libhdf5_ecf_codec.so")
  if(EXISTS "${_ecf_so}")
    set(_shim_dir "${CMAKE_CURRENT_BINARY_DIR}/hdf5_ecf_shim")
    file(WRITE "${_shim_dir}/hdf5_ecf-config.cmake"
"if(NOT TARGET hdf5_ecf_codec)
  add_library(hdf5_ecf_codec SHARED IMPORTED)
  set_target_properties(hdf5_ecf_codec PROPERTIES IMPORTED_LOCATION \"${_ecf_so}\")
endif()
set(hdf5_ecf_FOUND TRUE)
")
    set(hdf5_ecf_DIR "${_shim_dir}")
  endif()
endif()

# ON makes a missing SDK a hard FATAL_ERROR instead of a quiet skip -- so a build
# that CLEARLY meant to include the SDK tier (e.g. install_sdk.sh, which passes this)
# cannot silently "succeed" having produced zero pipelines. A plain `colcon build`
# without SDK Pro leaves it OFF and skips quietly, as before.
option(EVK4_REQUIRE_SDK "Fail (do not skip) if the Metavision SDK Pro is not found" OFF)

find_package(MetavisionSDK QUIET COMPONENTS base core cv analytics)
if(NOT MetavisionSDK_FOUND)
  # Diagnose the most common cause: openeb_vendor (the OPEN edition -- base/core/stream
  # only) on CMAKE_PREFIX_PATH shadows SDK Pro, so cv/analytics come up missing.
  find_package(MetavisionSDK QUIET COMPONENTS base core)
  if(MetavisionSDK_FOUND)
    set(_why "a MetavisionSDK was found at '${MetavisionSDK_DIR}' but it lacks cv/analytics -- that is the OPEN edition (openeb_vendor) shadowing SDK Pro; pass -DMetavisionSDK_DIR=<sdk_build>/generated/share/cmake/MetavisionSDKCMakePackagesFilesDir to point at your SDK Pro build")
  else()
    set(_why "no Metavision SDK found at all; install SDK Pro and pass -DMetavisionSDK_DIR=... (see docs/sdk/)")
  endif()
  if(EVK4_REQUIRE_SDK)
    message(FATAL_ERROR
      "evk4_sdk_advanced: SDK Pro (base/core/cv/analytics) was required (EVK4_REQUIRE_SDK=ON) but not found -- ${_why}.")
  endif()
  message(WARNING
    "evk4_sdk_advanced: Metavision SDK Pro (base/core/cv/analytics) not found, skipping this optional package -- ${_why}.")
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

add_sdk_component(undistortion_component src/undistortion_node.cpp
  "evk4_sdk_advanced::Undistortion" undistortion
  MetavisionSDK::base MetavisionSDK::core MetavisionSDK::cv)
# Reads the standard camera_info YAML (same file evk4_calibration produces).
ament_target_dependencies(undistortion_component camera_calibration_parsers)

# --- ML sub-tier (optional) -------------------------------------------------
# Neural-network pipelines (gesture/detection/flow inference) need the SDK `ml`
# module AND LibTorch. These are only present in the full x86 SDK build (built
# with -DUSE_TORCH=ON); the Pi's lean build (USE_TORCH=OFF) has neither, so the
# ML pipelines are simply skipped there. Pass -DTorch_DIR=<libtorch>/share/cmake/Torch
# to enable.
find_package(Torch QUIET)
# (the hdf5_ecf shim that the ml/stream modules need is set up near the top, before
# the base/core/cv/analytics find, so it applies on non-ML boxes too)
find_package(MetavisionSDK QUIET COMPONENTS ml)
if(Torch_FOUND AND TARGET MetavisionSDK::ml)
  message(STATUS "evk4_sdk_advanced: ML tier ENABLED (Torch + SDK ml module found)")
  add_sdk_component(gesture_component src/gesture_node.cpp
    "evk4_sdk_advanced::GestureClassification" gesture
    MetavisionSDK::base MetavisionSDK::core MetavisionSDK::ml ${TORCH_LIBRARIES})
  add_sdk_component(detection_component src/detection_node.cpp
    "evk4_sdk_advanced::DetectionTracking" detection
    MetavisionSDK::base MetavisionSDK::core MetavisionSDK::ml ${TORCH_LIBRARIES})
  add_sdk_component(flow_inference_component src/flow_inference_node.cpp
    "evk4_sdk_advanced::OpticalFlowInference" flow_inference
    MetavisionSDK::base MetavisionSDK::core MetavisionSDK::ml ${TORCH_LIBRARIES})
else()
  message(STATUS "evk4_sdk_advanced: ML tier skipped (need Torch + SDK ml module; "
    "pass -DTorch_DIR=... on a full SDK build)")
endif()
# ----------------------------------------------------------------------------

# --- cv3d sub-tier (optional) -----------------------------------------------
# Geometric pipelines (edgelet tracking, and the future ArUco/model-3D/active
# marker ones) need the SDK `cv3d` module, which only builds when the SDK is
# configured with -DUSE_SOPHUS=ON (Sophus). A lean build without it skips these.
find_package(MetavisionSDK QUIET COMPONENTS cv3d)
if(TARGET MetavisionSDK::cv3d)
  message(STATUS "evk4_sdk_advanced: cv3d tier ENABLED (SDK cv3d module found)")
  add_sdk_component(edgelet_component src/edgelet_node.cpp
    "evk4_sdk_advanced::EdgeletTracking" edgelet
    MetavisionSDK::base MetavisionSDK::core MetavisionSDK::cv MetavisionSDK::cv3d)
else()
  message(STATUS "evk4_sdk_advanced: cv3d tier skipped (need the SDK built with "
    "-DUSE_SOPHUS=ON; see docs/sdk/install.md).")
endif()
# ----------------------------------------------------------------------------

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
