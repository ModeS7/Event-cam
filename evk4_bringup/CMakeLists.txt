cmake_minimum_required(VERSION 3.16)
project(evk4_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Python helper node (camera_info publisher); installed so `ros2 run` and the
# launch file can find it.
install(PROGRAMS scripts/camera_info_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
