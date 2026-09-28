cmake_minimum_required(VERSION 3.8)
project(ranger_xarm6_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

add_executable(livox_cloud_to_custom src/livox_cloud_to_custom.cpp)
ament_target_dependencies(livox_cloud_to_custom rclcpp sensor_msgs livox_ros_driver2)

add_executable(depth_to_cloud src/depth_to_cloud.cpp)
ament_target_dependencies(depth_to_cloud rclcpp sensor_msgs tf2_ros)

add_executable(cloud_self_filter src/cloud_self_filter.cpp)
target_include_directories(cloud_self_filter PRIVATE ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(cloud_self_filter rclcpp sensor_msgs tf2_ros)

add_executable(obstacle_memory src/obstacle_memory.cpp)
target_include_directories(obstacle_memory PRIVATE ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(obstacle_memory rclcpp sensor_msgs tf2_ros)

install(TARGETS livox_cloud_to_custom depth_to_cloud cloud_self_filter obstacle_memory DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch maps DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/depth_mapper.py
  scripts/ekf_inputs.py
  scripts/pcd_to_grid.py
  scripts/pcd_to_map_frame.py
  scripts/planar_map_to_odom.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
