cmake_minimum_required(VERSION 3.8)
project(ranger_xarm6_tasks)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(btcpp_ros2_interfaces REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ranger_xarm6_manipulation REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(task_server src/task_server.cpp src/nodes.cpp src/task_context.cpp)
ament_target_dependencies(task_server rclcpp rclcpp_action behaviortree_cpp behaviortree_ros2
  btcpp_ros2_interfaces nav2_msgs control_msgs geometry_msgs tf2_ros ranger_xarm6_manipulation)
target_link_libraries(task_server yaml-cpp)

install(TARGETS task_server DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS
  scripts/save_waypoint.py
  scripts/save_arm_pose.py
  scripts/run_task.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config trees DESTINATION share/${PROJECT_NAME})
# Where the trees and taught poses live: the source folder, so Groot2, the
# teach tools and the task server all use the files under version control,
# and edits need no rebuild. launch/tasks.launch.py reads this.
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/source_dir.txt "${CMAKE_CURRENT_SOURCE_DIR}")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/source_dir.txt DESTINATION share/${PROJECT_NAME})

ament_package()
