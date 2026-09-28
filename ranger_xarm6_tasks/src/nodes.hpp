// The task nodes Groot2 shows in its palette. Each one is a client of an
// action the robot already serves:
//   NavigateToWaypoint, NavigateToPose  -> <ns>/navigate_to_pose (Nav2)
//   BaseToPose, TurnBase, ArmToNamedPose,
//   ArmToPose, WholeBodyToPose          -> <ns>/mobile_manipulation/move_to_goal (MoveIt)
//   Gripper                             -> <ns>/gripper_command
// plus Say (a log line). Angles on ports are in degrees.
#pragma once

#include <memory>
#include <string>

#include <behaviortree_cpp/bt_factory.h>
#include <control_msgs/action/gripper_command.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <ranger_xarm6_manipulation/action/move_to_goal.hpp>

#include "action_node.hpp"
#include "task_context.hpp"

namespace ranger_xarm6_tasks
{

using ContextPtr = std::shared_ptr<TaskContext>;

void registerTaskNodes(BT::BehaviorTreeFactory& factory, const std::shared_ptr<rclcpp::Node>& node,
                       const ContextPtr& context);

// ---- Nav2 -----------------------------------------------------------

class NavigateToWaypoint : public ActionNode<nav2_msgs::action::NavigateToPose>
{
public:
  NavigateToWaypoint(const std::string& name, const BT::NodeConfig& conf, std::shared_ptr<rclcpp::Node> node,
                     ContextPtr context)
    : ActionNode(name, conf, std::move(node), "navigate_to_pose"), context_(std::move(context))
  {
  }
  static BT::PortsList providedPorts();
  bool setGoal(Goal& goal) override;
  BT::NodeStatus onResult(const WrappedResult& result) override;

protected:
  virtual std::optional<Waypoint> target();
  ContextPtr context_;
};

class NavigateToPose : public NavigateToWaypoint
{
public:
  using NavigateToWaypoint::NavigateToWaypoint;
  static BT::PortsList providedPorts();

protected:
  std::optional<Waypoint> target() override;
};

// ---- MoveIt (MoveToGoal) ----------------------------------------------

class MoveToGoalNode : public ActionNode<ranger_xarm6_manipulation::action::MoveToGoal>
{
public:
  MoveToGoalNode(const std::string& name, const BT::NodeConfig& conf, std::shared_ptr<rclcpp::Node> node,
                 ContextPtr context)
    : ActionNode(name, conf, node, "mobile_manipulation/move_to_goal"), context_(std::move(context)), ros_node_(node)
  {
    accept_timeout_ = std::chrono::seconds(30);  // the coordinator may still be starting a stage
  }
  BT::NodeStatus onResult(const WrappedResult& result) override;
  void onFeedback(const Feedback& feedback) override;
  // MoveIt plans from the current state and aborts a plan whose start is
  // more than 0.02 off when it executes: right after Nav2 reports arrival
  // the base is still settling (a spin failed that way in sim). So wait
  // until it has been still for 0.4 s (at most 3 s).
  bool readyToStart() override;
  // Before the goal: base_trajectory_server's collision_monitor parameter
  // from the 'collision_monitor' port (default: default_monitor_), back to
  // 'auto' after. Docking at a table has to bypass it: the base goes partly
  // under the tabletop, which the monitor (2D footprint) calls a collision
  // (a whole-body dock stopped 27 cm short in sim); MoveIt checks the room
  // in 3D instead.
  void onHalted() override;
  void onFinished() override;
  std::string default_monitor_ = "auto";

protected:
  static BT::PortsList commonPorts(BT::PortsList more);
  bool setMonitor(const std::string& value);  // async; true once confirmed (or given up)
  std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> monitor_future_;
  std::chrono::steady_clock::time_point monitor_sent_;
  bool monitor_changed_ = false;
  std::shared_ptr<rclcpp::Node> ros_node_;
  std::optional<Waypoint> still_pose_;
  std::chrono::steady_clock::time_point still_since_, wait_started_;
  bool waiting_ = false;
  // mode + velocity_scaling from the ports into the goal.
  void fillCommon(Goal& goal, uint8_t default_mode);
  // The ports' arm pose (a taught name, or x/y/z/frame/orientation), plus offset_z.
  bool armPoseFromPorts(geometry_msgs::msg::PoseStamped& pose);
  ContextPtr context_;
};

class BaseToPose : public MoveToGoalNode
{
public:
  using MoveToGoalNode::MoveToGoalNode;
  static BT::PortsList providedPorts();
  bool setGoal(Goal& goal) override;
};

class TurnBase : public MoveToGoalNode
{
public:
  using MoveToGoalNode::MoveToGoalNode;
  static BT::PortsList providedPorts();
  bool setGoal(Goal& goal) override;
};

class ArmToNamedPose : public MoveToGoalNode
{
public:
  using MoveToGoalNode::MoveToGoalNode;
  static BT::PortsList providedPorts();
  bool setGoal(Goal& goal) override;
};

class ArmToPose : public MoveToGoalNode
{
public:
  using MoveToGoalNode::MoveToGoalNode;
  static BT::PortsList providedPorts();
  bool setGoal(Goal& goal) override;
};

class WholeBodyToPose : public MoveToGoalNode
{
public:
  WholeBodyToPose(const std::string& name, const BT::NodeConfig& conf, std::shared_ptr<rclcpp::Node> node,
                  ContextPtr context)
    : MoveToGoalNode(name, conf, std::move(node), std::move(context))
  {
    default_monitor_ = "never";
  }
  static BT::PortsList providedPorts();
  bool setGoal(Goal& goal) override;
};

// ---- Gripper ------------------------------------------------------------

class Gripper : public ActionNode<control_msgs::action::GripperCommand>
{
public:
  Gripper(const std::string& name, const BT::NodeConfig& conf, std::shared_ptr<rclcpp::Node> node)
    : ActionNode(name, conf, std::move(node), "gripper_command")
  {
  }
  static BT::PortsList providedPorts();
  bool setGoal(Goal& goal) override;
  BT::NodeStatus onResult(const WrappedResult& result) override;
};

// ---- Misc -----------------------------------------------------------------

class Say : public BT::SyncActionNode
{
public:
  Say(const std::string& name, const BT::NodeConfig& conf, std::shared_ptr<rclcpp::Node> node)
    : SyncActionNode(name, conf), node_(std::move(node))
  {
  }
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  std::shared_ptr<rclcpp::Node> node_;
};

}  // namespace ranger_xarm6_tasks
