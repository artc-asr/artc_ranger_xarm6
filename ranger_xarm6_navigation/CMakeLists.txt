cmake_minimum_required(VERSION 3.8)
project(ranger_xarm6_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

add_executable(livox_cloud_to_custom src/livox_cloud_to_custom.cpp)
ament_target_dependencies(livox_cloud_to_custom rclcpp sensor_msgs livox_ros_driver2)

install(TARGETS livox_cloud_to_custom DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/ekf_inputs.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
