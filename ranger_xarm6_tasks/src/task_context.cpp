#include "task_context.hpp"

#include <cmath>
#include <filesystem>

#include <yaml-cpp/yaml.h>

namespace ranger_xarm6_tasks
{

TaskContext::TaskContext(std::shared_ptr<tf2_ros::Buffer> tf, const std::string& prefix)
  : tf_(std::move(tf))
  , map_frame_(prefix + "map")
  , odom_frame_(prefix + "odom")
  , base_frame_(prefix + "base_link")
{
}

std::string TaskContext::load(const std::string& tasks_dir)
{
  std::map<std::string, Waypoint> waypoints;
  std::map<std::string, ArmPose> arm_poses;
  const auto config = std::filesystem::path(tasks_dir) / "config";
  try
  {
    const auto wp_file = config / "waypoints.yaml";
    if(std::filesystem::exists(wp_file))
    {
      const YAML::Node root = YAML::LoadFile(wp_file.string());
      for(const auto& it : root)
      {
        const auto& v = it.second;
        Waypoint w;
        w.frame = v["frame"] ? v["frame"].as<std::string>() : map_frame_;
        w.x = v["x"].as<double>();
        w.y = v["y"].as<double>();
        w.yaw = (v["yaw_deg"] ? v["yaw_deg"].as<double>() : 0.0) * M_PI / 180.0;
        waypoints[it.first.as<std::string>()] = w;
      }
    }
    const auto pose_file = config / "arm_poses.yaml";
    if(std::filesystem::exists(pose_file))
    {
      const YAML::Node root = YAML::LoadFile(pose_file.string());
      for(const auto& it : root)
      {
        const auto& v = it.second;
        ArmPose p;
        p.frame = v["frame"] ? v["frame"].as<std::string>() : map_frame_;
        for(int i = 0; i < 3; ++i)
          p.position[i] = v["position"][i].as<double>();
        for(int i = 0; i < 4; ++i)
          p.orientation[i] = v["orientation"][i].as<double>();
        arm_poses[it.first.as<std::string>()] = p;
      }
    }
  }
  catch(const std::exception& e)
  {
    return std::string("reading ") + config.string() + ": " + e.what();
  }
  std::lock_guard<std::mutex> lock(mutex_);
  waypoints_ = std::move(waypoints);
  arm_poses_ = std::move(arm_poses);
  return "";
}

std::optional<Waypoint> TaskContext::waypoint(const std::string& name) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = waypoints_.find(name);
  if(it == waypoints_.end())
    return std::nullopt;
  return it->second;
}

std::optional<ArmPose> TaskContext::armPose(const std::string& name) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = arm_poses_.find(name);
  if(it == arm_poses_.end())
    return std::nullopt;
  return it->second;
}

Waypoint TaskContext::transform(const Waypoint& in, const std::string& target) const
{
  if(in.frame == target)
    return in;
  const auto t = tf_->lookupTransform(target, in.frame, tf2::TimePointZero);
  const double tyaw = yawOf(t.transform.rotation);
  const double c = std::cos(tyaw), s = std::sin(tyaw);
  Waypoint out;
  out.frame = target;
  out.x = t.transform.translation.x + c * in.x - s * in.y;
  out.y = t.transform.translation.y + s * in.x + c * in.y;
  out.yaw = std::atan2(std::sin(in.yaw + tyaw), std::cos(in.yaw + tyaw));
  return out;
}

Waypoint TaskContext::basePose(const std::string& frame) const
{
  const auto t = tf_->lookupTransform(frame, base_frame_, tf2::TimePointZero);
  Waypoint w;
  w.frame = frame;
  w.x = t.transform.translation.x;
  w.y = t.transform.translation.y;
  w.yaw = yawOf(t.transform.rotation);
  return w;
}

double yawOf(const geometry_msgs::msg::Quaternion& q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw / 2.0);
  q.w = std::cos(yaw / 2.0);
  return q;
}

}  // namespace ranger_xarm6_tasks
