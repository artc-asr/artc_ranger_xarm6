cmake_minimum_required(VERSION 3.8)
project(ranger_xarm6_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY robots launch config worlds meshes
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/base_pose_publisher.py scripts/gz_lidar_to_pointcloud.py scripts/fix_wrench_frame_id.py scripts/world_markers.py scripts/world_collision_objects.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
