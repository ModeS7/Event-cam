cmake_minimum_required(VERSION 3.16)
project(evk4_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

# --- Metavision SDK / OpenEB ------------------------------------------------
# find_package(MetavisionSDK) is satisfied by EITHER openeb_vendor (apt) OR a
# full SDK install -- OpenEB is a strict subset, so this driver builds on both.
# Probe the version first: <5 ships a single 'driver' component, >=5 splits it.
find_package(MetavisionSDK COMPONENTS driver QUIET)
if(MetavisionSDK_FOUND AND ${MetavisionSDK_VERSION_MAJOR} LESS 5)
  set(MV_COMPONENTS driver)
else()
  set(MV_COMPONENTS base core stream)
endif()
find_package(MetavisionSDK COMPONENTS ${MV_COMPONENTS} REQUIRED)
add_definitions(-DMETAVISION_VERSION=${MetavisionSDK_VERSION_MAJOR})

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(event_camera_msgs REQUIRED)

set(MV_TARGETS ${MV_COMPONENTS})
list(TRANSFORM MV_TARGETS PREPEND "MetavisionSDK::")

# --- composable component ---------------------------------------------------
add_library(evk4_driver_component SHARED src/evk4_driver.cpp)
target_include_directories(evk4_driver_component PRIVATE include)
ament_target_dependencies(evk4_driver_component
  rclcpp rclcpp_components event_camera_msgs)
target_link_libraries(evk4_driver_component ${MV_TARGETS})
rclcpp_components_register_nodes(evk4_driver_component "evk4_driver::EVK4Driver")

# --- standalone node (self-contained: compiles the component in directly) ---
add_executable(driver_node src/driver_node.cpp src/evk4_driver.cpp)
target_include_directories(driver_node PRIVATE include)
ament_target_dependencies(driver_node
  rclcpp rclcpp_components event_camera_msgs)
target_link_libraries(driver_node ${MV_TARGETS})

install(TARGETS driver_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS evk4_driver_component DESTINATION lib)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.py")

ament_package()
