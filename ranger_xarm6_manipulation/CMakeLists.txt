cmake_minimum_required(VERSION 3.8)
project(ranger_xarm6_manipulation)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveToGoal.action"
  DEPENDENCIES geometry_msgs
)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/base_trajectory_server.py
  scripts/mobile_manipulation_coordinator.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
