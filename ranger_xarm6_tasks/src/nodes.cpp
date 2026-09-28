#include "nodes.hpp"

#include <rclcpp/parameter_client.hpp>

#include <cmath>
#include <sstream>

namespace ranger_xarm6_tasks
{
namespace
{
constexpr double kDeg = M_PI / 180.0;
constexpr uint8_t kSequential = ranger_xarm6_manipulation::action::MoveToGoal::Goal::SEQUENTIAL;
constexpr uint8_t kWholeBody = ranger_xarm6_manipulation::action::MoveToGoal::Goal::WHOLE_BODY;

const rclcpp::Logger kLog = rclcpp::get_logger("ranger_xarm6_tasks");

// "down" (gripper pointing down), "down:<deg>" (and turned about the
// vertical), or "qx qy qz qw".
bool parseOrientation(const std::string& text, double q[4])
{
  if(text.rfind("down", 0) == 0)
  {
    double yaw = 180.0;  // "down" = (0, 1, 0, 0), as in the README examples
    if(text.size() > 5 && text[4] == ':')
      yaw = std::stod(text.substr(5));
    const double h = yaw * kDeg / 2.0;
    // roll = pi, then yaw: (cos h, sin h, 0, 0) as x y z w = (cos, sin, 0, 0)
    q[0] = std::cos(h);
    q[1] = std::sin(h);
    q[2] = 0.0;
    q[3] = 0.0;
    return true;
  }
  std::istringstream in(text);
  return static_cast<bool>(in >> q[0] >> q[1] >> q[2] >> q[3]);
}

uint8_t modeFrom(const std::string& text, uint8_t fallback)
{
  if(text == "sequential")
    return kSequential;
  if(text == "whole_body")
    return kWholeBody;
  return fallback;
}
}  // namespace

// ---- Nav2 -----------------------------------------------------------

BT::PortsList NavigateToWaypoint::providedPorts()
{
  return BT::PortsList({ BT::InputPort<std::string>("waypoint", "A taught waypoint "
                                                                      "(config/waypoints.yaml)") });
}

std::optional<Waypoint> NavigateToWaypoint::target()
{
  const auto name = getInput<std::string>("waypoint");
  if(!name)
  {
    RCLCPP_ERROR(kLog, "[%s] no waypoint given", this->name().c_str());
    return std::nullopt;
  }
  auto w = context_->waypoint(name.value());
  if(!w)
    RCLCPP_ERROR(kLog, "[%s] unknown waypoint '%s' (config/waypoints.yaml)", this->name().c_str(),
                 name.value().c_str());
  return w;
}

bool NavigateToWaypoint::setGoal(Goal& goal)
{
  const auto w = target();
  if(!w)
    return false;
  goal.pose.header.frame_id = w->frame;
  goal.pose.pose.position.x = w->x;
  goal.pose.pose.position.y = w->y;
  goal.pose.pose.orientation = quaternionFromYaw(w->yaw);
  RCLCPP_INFO(kLog, "[%s] Nav2 to (%.2f, %.2f, %.0f deg) in %s", name().c_str(), w->x, w->y, w->yaw / kDeg,
              w->frame.c_str());
  return true;
}

BT::NodeStatus NavigateToWaypoint::onResult(const WrappedResult& result)
{
  if(result.code == rclcpp_action::ResultCode::SUCCEEDED)
    return BT::NodeStatus::SUCCESS;
  RCLCPP_WARN(kLog, "[%s] Nav2 goal failed (code %d)", name().c_str(), static_cast<int>(result.code));
  return BT::NodeStatus::FAILURE;
}

BT::PortsList NavigateToPose::providedPorts()
{
  return BT::PortsList({ BT::InputPort<double>("x", "m"), BT::InputPort<double>("y", "m"),
                              BT::InputPort<double>("yaw_deg", 0.0, "heading, deg"),
                              BT::InputPort<std::string>("frame", "", "default: the map frame") });
}

std::optional<Waypoint> NavigateToPose::target()
{
  Waypoint w;
  const auto x = getInput<double>("x"), y = getInput<double>("y");
  if(!x || !y)
  {
    RCLCPP_ERROR(kLog, "[%s] x and y are required", name().c_str());
    return std::nullopt;
  }
  w.x = x.value();
  w.y = y.value();
  w.yaw = getInput<double>("yaw_deg").value_or(0.0) * kDeg;
  const auto frame = getInput<std::string>("frame").value_or("");
  w.frame = frame.empty() ? context_->mapFrame() : frame;
  return w;
}

// ---- MoveIt (MoveToGoal) ----------------------------------------------

BT::PortsList MoveToGoalNode::commonPorts(BT::PortsList more)
{
  more.insert(BT::InputPort<double>("velocity_scaling", 0.0, "0..1; 0 = default (0.3)"));
  return more;
}

void MoveToGoalNode::fillCommon(Goal& goal, uint8_t default_mode)
{
  goal.mode = modeFrom(getInput<std::string>("mode").value_or(""), default_mode);
  goal.velocity_scaling = getInput<double>("velocity_scaling").value_or(0.0);
}

bool MoveToGoalNode::armPoseFromPorts(geometry_msgs::msg::PoseStamped& pose)
{
  const std::string name = getInput<std::string>("pose").value_or("");
  ArmPose p;
  if(!name.empty())
  {
    const auto taught = context_->armPose(name);
    if(!taught)
    {
      RCLCPP_ERROR(kLog, "[%s] unknown arm pose '%s' (config/arm_poses.yaml)", this->name().c_str(),
                   name.c_str());
      return false;
    }
    p = taught.value();
  }
  else
  {
    const auto x = getInput<double>("x"), y = getInput<double>("y"), z = getInput<double>("z");
    if(!x || !y || !z)
    {
      RCLCPP_ERROR(kLog, "[%s] give a taught 'pose', or x, y and z", this->name().c_str());
      return false;
    }
    p.position[0] = x.value();
    p.position[1] = y.value();
    p.position[2] = z.value();
    const auto frame = getInput<std::string>("frame").value_or("");
    p.frame = frame.empty() ? context_->mapFrame() : frame;
    if(!parseOrientation(getInput<std::string>("orientation").value_or("down"), p.orientation))
    {
      RCLCPP_ERROR(kLog, "[%s] orientation: 'down', 'down:<deg>' or 'qx qy qz qw'", this->name().c_str());
      return false;
    }
  }
  pose.header.frame_id = p.frame;
  pose.pose.position.x = p.position[0];
  pose.pose.position.y = p.position[1];
  pose.pose.position.z = p.position[2] + getInput<double>("offset_z").value_or(0.0);
  pose.pose.orientation.x = p.orientation[0];
  pose.pose.orientation.y = p.orientation[1];
  pose.pose.orientation.z = p.orientation[2];
  pose.pose.orientation.w = p.orientation[3];
  return true;
}

BT::NodeStatus MoveToGoalNode::onResult(const WrappedResult& result)
{
  if(result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result->success)
    return BT::NodeStatus::SUCCESS;
  RCLCPP_WARN(kLog, "[%s] MoveIt: %s", name().c_str(),
              result.result ? result.result->message.c_str() : "no result");
  return BT::NodeStatus::FAILURE;
}

bool MoveToGoalNode::setMonitor(const std::string& value)
{
  static std::mutex mutex;
  static std::shared_ptr<rclcpp::AsyncParametersClient> client;
  {
    std::lock_guard<std::mutex> lock(mutex);
    if(!client)
      client = std::make_shared<rclcpp::AsyncParametersClient>(ros_node_, "base_trajectory_server");
  }
  if(!client->service_is_ready())
    return false;
  monitor_future_ = client->set_parameters({ rclcpp::Parameter("collision_monitor", value) });
  monitor_sent_ = std::chrono::steady_clock::now();
  return true;
}

void MoveToGoalNode::onFinished()
{
  if(monitor_changed_)
  {
    setMonitor("auto");
    monitor_changed_ = false;
  }
}

void MoveToGoalNode::onHalted()
{
  waiting_ = false;
  ActionNode::onHalted();
  onFinished();
}

bool MoveToGoalNode::readyToStart()
{
  const auto now = std::chrono::steady_clock::now();
  if(!waiting_)
  {
    waiting_ = true;
    wait_started_ = still_since_ = now;
    still_pose_.reset();
    monitor_future_ = {};
    std::string monitor = getInput<std::string>("collision_monitor").value_or("");
    if(monitor.empty())
      monitor = default_monitor_;
    if(monitor != "auto" && monitor != "never" && monitor != "always")
    {
      RCLCPP_WARN(kLog, "[%s] collision_monitor '%s': auto | never | always; using auto", name().c_str(),
                  monitor.c_str());
      monitor = "auto";
    }
    if(setMonitor(monitor))
      monitor_changed_ = monitor != "auto";
    else if(monitor != "auto")
      RCLCPP_WARN(kLog, "[%s] can't reach base_trajectory_server to set collision_monitor=%s", name().c_str(),
                  monitor.c_str());
  }
  if(monitor_future_.valid() && monitor_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready &&
     now - monitor_sent_ < std::chrono::seconds(2))
    return false;  // the parameter must be in place before the goal
  if(now - wait_started_ > std::chrono::seconds(3))
  {
    waiting_ = false;
    return true;  // still moving after 3 s: go anyway, MoveIt will say if it can't
  }
  try
  {
    const auto pose = context_->basePose(context_->odomFrame());
    if(!still_pose_ || std::hypot(pose.x - still_pose_->x, pose.y - still_pose_->y) > 0.002 ||
       std::abs(std::remainder(pose.yaw - still_pose_->yaw, 2 * M_PI)) > 0.2 * kDeg)
    {
      still_pose_ = pose;
      still_since_ = now;
    }
  }
  catch(const std::exception&)
  {
    waiting_ = false;
    return true;  // no TF: nothing to wait for; the goal will fail with a reason
  }
  if(now - still_since_ >= std::chrono::milliseconds(400))
  {
    waiting_ = false;
    return true;
  }
  return false;
}

void MoveToGoalNode::onFeedback(const Feedback& feedback)
{
  RCLCPP_INFO(kLog, "[%s] %s: %s", name().c_str(), feedback.stage.c_str(), feedback.status.c_str());
}

namespace
{
std::pair<std::string, BT::PortInfo> monitorPort(const std::string& default_value)
{
  return BT::InputPort<std::string>("collision_monitor", default_value,
                                    "auto: the base's motion goes through the collision monitor while Nav2 "
                                    "runs | never: MoveIt's 3D check of the room only (docking under a "
                                    "tabletop) | always");
}
BT::PortsList armPosePorts(BT::PortsList more)
{
  more.insert(BT::InputPort<std::string>("pose", "", "A taught arm pose (config/arm_poses.yaml); "
                                                     "or leave empty and give x, y, z"));
  more.insert(BT::InputPort<double>("x", "m"));
  more.insert(BT::InputPort<double>("y", "m"));
  more.insert(BT::InputPort<double>("z", "m"));
  more.insert(BT::InputPort<std::string>("frame", "", "for x/y/z; default: the map frame. base_link = "
                                                      "relative to where the base ends up"));
  more.insert(BT::InputPort<std::string>("orientation", "down",
                                         "for x/y/z: down | down:<deg> | qx qy qz qw"));
  more.insert(BT::InputPort<double>("offset_z", 0.0, "added to z (e.g. 0.05 to hover above)"));
  return more;
}
}  // namespace

BT::PortsList BaseToPose::providedPorts()
{
  return commonPorts({ BT::InputPort<std::string>("waypoint", "", "A taught waypoint; or leave empty "
                                                                  "and give x, y, yaw_deg"),
                       BT::InputPort<double>("x", "m"), BT::InputPort<double>("y", "m"),
                       BT::InputPort<double>("yaw_deg", "deg"),
                       BT::InputPort<std::string>("frame", "", "for x/y/yaw; default: the map frame"),
                       BT::InputPort<std::string>("mode", "sequential", "sequential: the arm stows "
                                                                        "first | whole_body: it stays"),
                       monitorPort("auto") });
}

bool BaseToPose::setGoal(Goal& goal)
{
  Waypoint w;
  const std::string name = getInput<std::string>("waypoint").value_or("");
  if(!name.empty())
  {
    const auto taught = context_->waypoint(name);
    if(!taught)
    {
      RCLCPP_ERROR(kLog, "[%s] unknown waypoint '%s'", this->name().c_str(), name.c_str());
      return false;
    }
    w = taught.value();
  }
  else
  {
    const auto x = getInput<double>("x"), y = getInput<double>("y"), yaw = getInput<double>("yaw_deg");
    if(!x || !y || !yaw)
    {
      RCLCPP_ERROR(kLog, "[%s] give a waypoint, or x, y and yaw_deg", this->name().c_str());
      return false;
    }
    w.x = x.value();
    w.y = y.value();
    w.yaw = yaw.value() * kDeg;
    const auto frame = getInput<std::string>("frame").value_or("");
    w.frame = frame.empty() ? context_->mapFrame() : frame;
  }
  try
  {
    w = context_->transform(w, context_->odomFrame());  // MoveToGoal's base_goal is in odom
  }
  catch(const std::exception& e)
  {
    RCLCPP_ERROR(kLog, "[%s] %s", this->name().c_str(), e.what());
    return false;
  }
  fillCommon(goal, kSequential);
  goal.move_base = true;
  goal.base_goal.x = w.x;
  goal.base_goal.y = w.y;
  goal.base_goal.theta = w.yaw;
  RCLCPP_INFO(kLog, "[%s] base to (%.2f, %.2f, %.0f deg) odom", this->name().c_str(), w.x, w.y, w.yaw / kDeg);
  return true;
}

BT::PortsList TurnBase::providedPorts()
{
  return commonPorts(
      { BT::InputPort<std::string>("back_towards", "", "A waypoint or arm pose name: turn so the "
                                                       "arm side (the back) faces it"),
        BT::InputPort<double>("yaw_deg", "absolute heading in the map frame, deg"),
        BT::InputPort<double>("angle_deg", "relative: turn by this much, deg (+ = left)"),
        BT::InputPort<std::string>("mode", "sequential", "sequential: the arm stows first | "
                                                         "whole_body: it stays"),
        monitorPort("auto") });
}

bool TurnBase::setGoal(Goal& goal)
{
  double yaw_odom = 0.0;
  Waypoint here;
  try
  {
    here = context_->basePose(context_->odomFrame());
    const std::string towards = getInput<std::string>("back_towards").value_or("");
    if(!towards.empty())
    {
      Waypoint target;
      if(const auto w = context_->waypoint(towards))
        target = w.value();
      else if(const auto p = context_->armPose(towards))
      {
        target.frame = p->frame;
        target.x = p->position[0];
        target.y = p->position[1];
      }
      else
      {
        RCLCPP_ERROR(kLog, "[%s] unknown waypoint/arm pose '%s'", name().c_str(), towards.c_str());
        return false;
      }
      target = context_->transform(target, context_->odomFrame());
      yaw_odom = std::atan2(target.y - here.y, target.x - here.x) + M_PI;  // the back (-x) faces it
    }
    else if(const auto yaw = getInput<double>("yaw_deg"))
    {
      Waypoint w;
      w.frame = context_->mapFrame();
      w.yaw = yaw.value() * kDeg;
      yaw_odom = context_->transform(w, context_->odomFrame()).yaw;
    }
    else if(const auto angle = getInput<double>("angle_deg"))
      yaw_odom = here.yaw + angle.value() * kDeg;
    else
    {
      RCLCPP_ERROR(kLog, "[%s] give back_towards, yaw_deg or angle_deg", name().c_str());
      return false;
    }
  }
  catch(const std::exception& e)
  {
    RCLCPP_ERROR(kLog, "[%s] %s", name().c_str(), e.what());
    return false;
  }
  fillCommon(goal, kSequential);
  goal.move_base = true;
  goal.base_goal.x = here.x;  // same spot: MoveToGoal skips the crab, spins only
  goal.base_goal.y = here.y;
  goal.base_goal.theta = std::atan2(std::sin(yaw_odom), std::cos(yaw_odom));
  RCLCPP_INFO(kLog, "[%s] turn to %.0f deg (odom)", name().c_str(), goal.base_goal.theta / kDeg);
  return true;
}

BT::PortsList ArmToNamedPose::providedPorts()
{
  return commonPorts({ BT::InputPort<std::string>("pose", "stow", "SRDF named pose: home, stow, ...") });
}

bool ArmToNamedPose::setGoal(Goal& goal)
{
  fillCommon(goal, kSequential);
  goal.move_arm = true;
  goal.arm_named_goal = getInput<std::string>("pose").value_or("stow");
  return true;
}

BT::PortsList ArmToPose::providedPorts()
{
  return commonPorts(armPosePorts({}));
}

bool ArmToPose::setGoal(Goal& goal)
{
  fillCommon(goal, kSequential);  // arm only: the base stays where it is
  goal.move_arm = true;
  return armPoseFromPorts(goal.arm_pose_goal);
}

BT::PortsList WholeBodyToPose::providedPorts()
{
  return commonPorts(armPosePorts({ BT::InputPort<std::string>(
                                         "base_waypoint", "", "Where the base goes (a taught waypoint); empty = "
                                                              "the base position is chosen at the current heading"),
                                     monitorPort("never") }));
}

bool WholeBodyToPose::setGoal(Goal& goal)
{
  goal.mode = kWholeBody;
  goal.velocity_scaling = getInput<double>("velocity_scaling").value_or(0.0);
  goal.move_arm = true;
  const std::string base = getInput<std::string>("base_waypoint").value_or("");
  if(!base.empty())
  {
    const auto w = context_->waypoint(base);
    if(!w)
    {
      RCLCPP_ERROR(kLog, "[%s] unknown waypoint '%s'", name().c_str(), base.c_str());
      return false;
    }
    try
    {
      const auto o = context_->transform(w.value(), context_->odomFrame());
      goal.move_base = true;
      goal.base_goal.x = o.x;
      goal.base_goal.y = o.y;
      goal.base_goal.theta = o.yaw;
    }
    catch(const std::exception& e)
    {
      RCLCPP_ERROR(kLog, "[%s] %s", name().c_str(), e.what());
      return false;
    }
  }
  return armPoseFromPorts(goal.arm_pose_goal);
}

// ---- Gripper ------------------------------------------------------------

BT::PortsList Gripper::providedPorts()
{
  return BT::PortsList(
      { BT::InputPort<std::string>("command", "open", "open | close | drive_joint in rad (0..0.85)"),
        BT::InputPort<std::string>("expect", "any", "any | holding: fail if nothing was grasped | "
                                                    "empty: fail if something was") });
}

bool Gripper::setGoal(Goal& goal)
{
  const std::string cmd = getInput<std::string>("command").value_or("open");
  if(cmd == "open")
    goal.command.position = 0.0;
  else if(cmd == "close")
    goal.command.position = 0.85;
  else
  {
    try
    {
      goal.command.position = std::stod(cmd);
    }
    catch(const std::exception&)
    {
      RCLCPP_ERROR(kLog, "[%s] command '%s': open, close or a number", name().c_str(), cmd.c_str());
      return false;
    }
  }
  return true;
}

BT::NodeStatus Gripper::onResult(const WrappedResult& result)
{
  if(result.code != rclcpp_action::ResultCode::SUCCEEDED)
    return BT::NodeStatus::FAILURE;
  const std::string expect = getInput<std::string>("expect").value_or("any");
  const bool holding = result.result->stalled;
  RCLCPP_INFO(kLog, "[%s] at %.3f%s", name().c_str(), result.result->position, holding ? ", holding something" : "");
  if(expect == "holding" && !holding)
  {
    RCLCPP_WARN(kLog, "[%s] expected to hold something, but the fingers closed fully", name().c_str());
    return BT::NodeStatus::FAILURE;
  }
  if(expect == "empty" && holding)
    return BT::NodeStatus::FAILURE;
  return BT::NodeStatus::SUCCESS;
}

// ---- Misc -----------------------------------------------------------------

BT::PortsList Say::providedPorts()
{
  return { BT::InputPort<std::string>("message", "", "logged by the task server") };
}

BT::NodeStatus Say::tick()
{
  RCLCPP_INFO(kLog, "[Say] %s", getInput<std::string>("message").value_or("").c_str());
  return BT::NodeStatus::SUCCESS;
}

// ---- Registration -----------------------------------------------------------

void registerTaskNodes(BT::BehaviorTreeFactory& factory, const std::shared_ptr<rclcpp::Node>& node,
                       const ContextPtr& context)
{
  factory.registerNodeType<NavigateToWaypoint>("NavigateToWaypoint", node, context);
  factory.registerNodeType<NavigateToPose>("NavigateToPose", node, context);
  factory.registerNodeType<BaseToPose>("BaseToPose", node, context);
  factory.registerNodeType<TurnBase>("TurnBase", node, context);
  factory.registerNodeType<ArmToNamedPose>("ArmToNamedPose", node, context);
  factory.registerNodeType<ArmToPose>("ArmToPose", node, context);
  factory.registerNodeType<WholeBodyToPose>("WholeBodyToPose", node, context);
  factory.registerNodeType<Gripper>("Gripper", node);
  factory.registerNodeType<Say>("Say", node);
}

}  // namespace ranger_xarm6_tasks
