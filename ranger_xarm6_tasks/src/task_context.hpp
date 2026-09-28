// What every task node shares: the taught waypoints and arm poses (YAML,
// re-read at the start of each task), TF, and the robot's frame names.
#pragma once

#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>

namespace ranger_xarm6_tasks
{

// A base goal: where the base goes and which way it faces.
struct Waypoint
{
  std::string frame;  // e.g. robot_a_map
  double x = 0, y = 0, yaw = 0;  // m, m, rad
};

// A gripper (link_tcp) pose.
struct ArmPose
{
  std::string frame;  // e.g. robot_a_map, or robot_a_base_link (relative to where the base ends up)
  double position[3] = { 0, 0, 0 };
  double orientation[4] = { 0, 0, 0, 1 };  // x y z w
};

class TaskContext
{
public:
  TaskContext(std::shared_ptr<tf2_ros::Buffer> tf, const std::string& prefix);

  // (Re)reads <dir>/config/waypoints.yaml and arm_poses.yaml; a missing
  // file is an empty list. Returns an error message, or "" if fine.
  std::string load(const std::string& tasks_dir);

  std::optional<Waypoint> waypoint(const std::string& name) const;
  std::optional<ArmPose> armPose(const std::string& name) const;

  // (x, y, yaw) in `frame` -> the same pose in `target` (TF, latest).
  // Throws tf2::TransformException.
  Waypoint transform(const Waypoint& in, const std::string& target) const;
  // The base's current pose in `frame`.
  Waypoint basePose(const std::string& frame) const;

  const std::string& mapFrame() const { return map_frame_; }
  const std::string& odomFrame() const { return odom_frame_; }
  const std::string& baseFrame() const { return base_frame_; }

private:
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string map_frame_, odom_frame_, base_frame_;
  mutable std::mutex mutex_;
  std::map<std::string, Waypoint> waypoints_;
  std::map<std::string, ArmPose> arm_poses_;
};

double yawOf(const geometry_msgs::msg::Quaternion& q);
geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw);

}  // namespace ranger_xarm6_tasks
