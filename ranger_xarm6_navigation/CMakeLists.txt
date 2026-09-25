cmake_minimum_required(VERSION 3.8)
project(ranger_xarm6_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/ekf_inputs.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
