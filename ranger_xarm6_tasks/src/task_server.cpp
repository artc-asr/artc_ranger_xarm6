// The task server: runs one behavior tree (a task) at a time, by name.
//
//   action   <ns>/execute_task (btcpp_ros2_interfaces/ExecuteTree: target_tree = task name)
//   service  <ns>/get_loaded_trees (the tasks loaded at the last goal or at startup)
//   Groot2   live view on port groot2_port (1667) while a task runs
//
// Every goal re-reads <tasks_dir>/trees/*.xml and <tasks_dir>/config/*.yaml,
// so a tree saved in Groot2 or a waypoint just taught is used by the next
// run, no restart or rebuild. At startup it writes
// <tasks_dir>/ranger_xarm6_tasks.btproj: the Groot2 project, listing every
// tree and this server's nodes (TreeNodesModel), so Groot2's palette
// always matches the code.
#include <atomic>
#include <filesystem>
#include <fstream>
#include <regex>
#include <set>
#include <sstream>

#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <behaviortree_cpp/xml_parsing.h>
#include <behaviortree_ros2/tree_execution_server.hpp>
#include <tf2_ros/transform_listener.h>

#include "nodes.hpp"

namespace fs = std::filesystem;
using ranger_xarm6_tasks::TaskContext;

class TaskServer : public BT::TreeExecutionServer
{
public:
  explicit TaskServer(const rclcpp::NodeOptions& options)
    : TreeExecutionServer(options)
  {
    tasks_dir_ = node()->declare_parameter<std::string>("tasks_dir", "");
    const auto prefix = node()->declare_parameter<std::string>("frame_prefix", "robot_a_");
    if(tasks_dir_.empty() || !fs::is_directory(tasks_dir_))
      throw std::runtime_error("tasks_dir '" + tasks_dir_ + "' is not a directory");
    tf_ = std::make_shared<tf2_ros::Buffer>(node()->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_, node(), false);
    context_ = std::make_shared<TaskContext>(tf_, prefix);
    RCLCPP_INFO(node()->get_logger(), "Tasks: %s/trees, waypoints and arm poses: %s/config",
                tasks_dir_.c_str(), tasks_dir_.c_str());
  }

protected:
  void registerNodesIntoFactory(BT::BehaviorTreeFactory& factory) override
  {
    ranger_xarm6_tasks::registerTaskNodes(factory, node(), context_);
    loadTrees(factory);
    writeGrootProject(factory);
  }

  bool onGoalReceived(const std::string& tree_name, const std::string& /*payload*/) override
  {
    // A tree that threw is aborted without onTreeExecutionCompleted: a
    // "running" task that hasn't ticked for 2 s is over.
    if(running_ && std::chrono::steady_clock::now() - last_tick_.load() > std::chrono::seconds(2))
      running_ = false;
    if(running_)
    {
      RCLCPP_WARN(node()->get_logger(), "Refusing task '%s': a task is running (cancel it first)",
                  tree_name.c_str());
      return false;
    }
    if(const auto err = context_->load(tasks_dir_); !err.empty())
    {
      RCLCPP_ERROR(node()->get_logger(), "%s", err.c_str());
      return false;
    }
    loadTrees(factory());
    const auto trees = factory().registeredBehaviorTrees();
    if(std::find(trees.begin(), trees.end(), tree_name) == trees.end())
    {
      std::string known;
      for(const auto& t : trees)
        known += " " + t;
      RCLCPP_ERROR(node()->get_logger(), "No task '%s'. Tasks:%s", tree_name.c_str(), known.c_str());
      return false;
    }
    return true;
  }

  void onTreeCreated(BT::Tree& tree) override
  {
    logger_ = std::make_shared<BT::StdCoutLogger>(tree);
    last_tick_ = std::chrono::steady_clock::now();
    running_ = true;
    RCLCPP_INFO(node()->get_logger(), "Task '%s' started", treeName().c_str());
  }

  std::optional<BT::NodeStatus> onLoopAfterTick(BT::NodeStatus /*status*/) override
  {
    last_tick_ = std::chrono::steady_clock::now();
    return std::nullopt;
  }

  std::optional<std::string> onTreeExecutionCompleted(BT::NodeStatus status, bool was_cancelled) override
  {
    logger_.reset();
    running_ = false;
    const std::string msg = treeName() + ": " + (was_cancelled ? "cancelled" : BT::toStr(status));
    RCLCPP_INFO(node()->get_logger(), "Task %s", msg.c_str());
    return msg;
  }

private:
  // Every *.xml in <tasks_dir>/trees, freshly.
  void loadTrees(BT::BehaviorTreeFactory& factory)
  {
    factory.clearRegisteredBehaviorTrees();
    for(const auto& entry : fs::directory_iterator(fs::path(tasks_dir_) / "trees"))
    {
      if(entry.path().extension() != ".xml")
        continue;
      try
      {
        factory.registerBehaviorTreeFromFile(entry.path().string());
      }
      catch(const std::exception& e)
      {
        RCLCPP_ERROR(node()->get_logger(), "%s: %s", entry.path().filename().c_str(), e.what());
      }
    }
  }

  // <tasks_dir>/ranger_xarm6_tasks.btproj: the trees + this server's node models.
  void writeGrootProject(BT::BehaviorTreeFactory& factory)
  {
    std::set<std::string> files;
    for(const auto& entry : fs::directory_iterator(fs::path(tasks_dir_) / "trees"))
      if(entry.path().extension() == ".xml")
        files.insert(entry.path().filename().string());
    // writeTreeNodesModelXML gives a whole <root> document; keep its <TreeNodesModel>.
    const std::string models = BT::writeTreeNodesModelXML(factory, false);
    const auto begin = models.find("<TreeNodesModel>");
    const auto end = models.find("</TreeNodesModel>");
    std::ostringstream out;
    out << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
        << "<!-- Written by ranger_xarm6_tasks' task_server at startup: the trees in trees/ and the\n"
        << "     task nodes' ports. Open this in Groot2. -->\n"
        << "<root BTCPP_format=\"4\" project_name=\"ranger_xarm6_tasks\">\n";
    for(const auto& f : files)
      out << "    <include path=\"trees/" << f << "\"/>\n";
    if(begin != std::string::npos && end != std::string::npos)
      out << "    " << models.substr(begin, end + std::string("</TreeNodesModel>").size() - begin) << "\n";
    out << "</root>\n";
    const auto path = fs::path(tasks_dir_) / "ranger_xarm6_tasks.btproj";
    std::ofstream(path) << out.str();
    RCLCPP_INFO(node()->get_logger(), "Groot2 project: %s", path.c_str());
  }

  std::string tasks_dir_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<TaskContext> context_;
  std::shared_ptr<BT::StdCoutLogger> logger_;
  std::atomic<bool> running_{ false };
  std::atomic<std::chrono::steady_clock::time_point> last_tick_{};
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto server = std::make_shared<TaskServer>(options);
  // As BehaviorTree.ROS2's sample: a timeout on spin avoids a MultiThreadedExecutor deadlock
  // when the tree's action clients come and go.
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 0, false,
                                                std::chrono::milliseconds(250));
  exec.add_node(server->node());
  exec.spin();
  exec.remove_node(server->node());
  rclcpp::shutdown();
}
