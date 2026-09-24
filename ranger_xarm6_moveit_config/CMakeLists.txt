cmake_minimum_required(VERSION 3.8)
project(ranger_xarm6_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch srdf urdf DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/base_joint_state_publisher.py DESTINATION lib/${PROJECT_NAME})

ament_package()
