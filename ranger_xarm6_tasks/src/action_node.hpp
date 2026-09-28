// A behavior tree node that runs one ROS action goal.
//
// Why not BehaviorTree.ROS2's RosActionNode: on Humble it gives every
// action client its own executor, spun from the tree's thread, next to the
// server node's own executor. In sim, after a MoveToGoal goal completed, the
// Nav2 client never processed another goal response: Nav2 took the goal
// and drove, the node timed out and failed, for good (both FastDDS and
// Cyclone). Here all clients live on the server's node and are served by
// its one (multi-threaded) executor; callbacks only record into a State
// the tree thread polls, so they never touch a node object that a finished
// tree has destroyed. A goal accepted after the node gave up (timeout,
// halt) is cancelled at once, so the robot never runs a goal no tree
// follows.
#pragma once

#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace ranger_xarm6_tasks
{

template <class ActionT>
class ActionNode : public BT::StatefulActionNode
{
public:
  using Goal = typename ActionT::Goal;
  using Feedback = typename ActionT::Feedback;
  using Client = rclcpp_action::Client<ActionT>;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;
  using WrappedResult = typename GoalHandle::WrappedResult;

  ActionNode(const std::string& name, const BT::NodeConfig& conf, std::shared_ptr<rclcpp::Node> node,
             std::string action_name)
    : StatefulActionNode(name, conf), node_(std::move(node)), action_name_(std::move(action_name))
  {
  }

  ~ActionNode() override
  {
    abandon();
  }

protected:
  // Fill the goal from the ports; false = the node fails (logged by the caller).
  virtual bool setGoal(Goal& goal) = 0;
  virtual BT::NodeStatus onResult(const WrappedResult& result) = 0;
  virtual void onFeedback(const Feedback& /*feedback*/)
  {
  }
  // After the goal ends (result, rejection, timeout; halts: onHalted).
  virtual void onFinished()
  {
  }
  // Checked before the goal is sent, every tick, until true: e.g. wait for
  // the base to stop.
  virtual bool readyToStart()
  {
    return true;
  }
  const rclcpp::Logger& log() const
  {
    static const rclcpp::Logger logger = rclcpp::get_logger("ranger_xarm6_tasks");
    return logger;
  }

  std::chrono::seconds accept_timeout_{ 10 };

protected:
  // Shared with the callbacks, which may outlive this node.
  struct State
  {
    std::mutex mutex;
    bool responded = false;
    bool abandoned = false;
    std::shared_ptr<GoalHandle> handle;  // null after the response = rejected
    std::optional<WrappedResult> result;
    std::vector<std::shared_ptr<const Feedback>> feedback;
  };

  // One client per action name for the whole process, on the server's node.
  std::shared_ptr<Client> client()
  {
    static std::mutex mutex;
    static std::map<std::string, std::shared_ptr<Client>> clients;
    std::lock_guard<std::mutex> lock(mutex);
    auto& c = clients[action_name_];
    if(!c)
      c = rclcpp_action::create_client<ActionT>(node_, action_name_);
    return c;
  }

  BT::NodeStatus onStart() override
  {
    waiting_to_start_ = !readyToStart();
    if(waiting_to_start_)
      return BT::NodeStatus::RUNNING;
    return sendGoal();
  }

  BT::NodeStatus sendGoal()
  {
    Goal goal;
    if(!setGoal(goal))
      return BT::NodeStatus::FAILURE;
    auto c = client();
    if(!c->action_server_is_ready() && !c->wait_for_action_server(std::chrono::seconds(2)))
    {
      RCLCPP_ERROR(log(), "[%s] no action server %s", name().c_str(), action_name_.c_str());
      return BT::NodeStatus::FAILURE;
    }
    auto state = std::make_shared<State>();
    state_ = state;
    typename Client::SendGoalOptions options;
    options.goal_response_callback = [state, c](std::shared_ptr<GoalHandle> handle) {
      std::lock_guard<std::mutex> lock(state->mutex);
      state->responded = true;
      state->handle = handle;
      if(handle && state->abandoned)
        c->async_cancel_goal(handle);  // nobody follows this goal any more
    };
    options.feedback_callback = [state](std::shared_ptr<GoalHandle>, const std::shared_ptr<const Feedback> fb) {
      std::lock_guard<std::mutex> lock(state->mutex);
      if(state->feedback.size() < 100)
        state->feedback.push_back(fb);
    };
    options.result_callback = [state](const WrappedResult& result) {
      std::lock_guard<std::mutex> lock(state->mutex);
      state->result = result;
    };
    c->async_send_goal(goal, options);
    sent_at_ = std::chrono::steady_clock::now();
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    if(waiting_to_start_)
    {
      if(!readyToStart())
        return BT::NodeStatus::RUNNING;
      waiting_to_start_ = false;
      return sendGoal();
    }
    auto state = state_;
    std::vector<std::shared_ptr<const Feedback>> feedback;
    std::optional<WrappedResult> result;
    bool responded, rejected;
    {
      std::lock_guard<std::mutex> lock(state->mutex);
      feedback.swap(state->feedback);
      result = state->result;
      responded = state->responded;
      rejected = responded && !state->handle;
    }
    for(const auto& fb : feedback)
      onFeedback(*fb);
    if(rejected)
    {
      RCLCPP_WARN(log(), "[%s] goal rejected by %s", name().c_str(), action_name_.c_str());
      state_.reset();
      onFinished();
      return BT::NodeStatus::FAILURE;
    }
    if(!responded && std::chrono::steady_clock::now() - sent_at_ > accept_timeout_)
    {
      RCLCPP_ERROR(log(), "[%s] no answer from %s", name().c_str(), action_name_.c_str());
      abandon();
      onFinished();
      return BT::NodeStatus::FAILURE;
    }
    if(result)
    {
      state_.reset();
      const auto status = onResult(result.value());
      onFinished();
      return status;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    waiting_to_start_ = false;
    abandon();
  }

  // Stop following the goal; cancel it if it's (or once it's) accepted.
  void abandon()
  {
    auto state = state_;
    state_.reset();
    if(!state)
      return;
    std::lock_guard<std::mutex> lock(state->mutex);
    state->abandoned = true;
    if(state->handle && !state->result)
      client()->async_cancel_goal(state->handle);
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::string action_name_;
  std::shared_ptr<State> state_;
  std::chrono::steady_clock::time_point sent_at_;
  bool waiting_to_start_ = false;
};

}  // namespace ranger_xarm6_tasks
