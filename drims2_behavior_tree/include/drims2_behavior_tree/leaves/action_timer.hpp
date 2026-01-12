#include <fstream>
#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include <behaviortree_ros2/plugins.hpp>
#include "behaviortree_ros2/ros_node_params.hpp"

class ActionTimer : public BT::ConditionNode
{
public:
  ActionTimer(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ConditionNode(name, config)
  {
    // Get the shared ROS2 node from the blackboard
    node_ = config.blackboard->get<std::shared_ptr<rclcpp::Node>>("node");
  }

  // Optional constructor with RosNodeParams if needed elsewhere
  ActionTimer(const std::string& name,
              const BT::NodeConfiguration& config,
              const BT::RosNodeParams& params)
  : BT::ConditionNode(name, config), node_(params.nh)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<bool>("start", true, "true = store start time, false = compute elapsed time"),
      BT::InputPort<std::string>("action_name", "Name of the timed action"),
      BT::InputPort<std::string>("log_path", "", "Log file path (\"\" = no file logging)")
    };
  }

  BT::NodeStatus tick() override
  {
    bool start_flag = true;
    if (!getInput("start", start_flag))
    {
      throw BT::RuntimeError("ActionTimer: missing required input [start]");
    }

    std::string action_name;
    if (!getInput("action_name", action_name))
    {
      throw BT::RuntimeError("ActionTimer: missing required input [action_name]");
    }

    std::string log_path;
    // Optional; if empty, no logging to file is performed
    getInput("log_path", log_path);

    const auto now = node_->get_clock()->now();
    const std::string bb_key = "start_time_" + action_name;
    auto bb = config().blackboard;

    if (start_flag)
    {
      // START: store current time in the blackboard
      bb->set<rclcpp::Time>(bb_key, now);

      RCLCPP_INFO(node_->get_logger(),
                  "ActionTimer '%s': START at time %.9f [s]",
                  action_name.c_str(), now.seconds());

      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      // STOP: read stored time, compute elapsed, print and optionally log to file
      rclcpp::Time start_time;
      if (!bb->get<rclcpp::Time>(bb_key, start_time))
      {
        RCLCPP_WARN(node_->get_logger(),
                    "ActionTimer '%s': no start time found in blackboard (key: %s)",
                    action_name.c_str(), bb_key.c_str());
        return BT::NodeStatus::FAILURE;
      }

      rclcpp::Duration diff = now - start_time;
      const double elapsed_sec = diff.seconds();

      RCLCPP_INFO(node_->get_logger(),
                  "ActionTimer '%s': elapsed %.6f [s]",
                  action_name.c_str(), elapsed_sec);

      // If log_path is not empty, append to file (create if it does not exist)
      if (!log_path.empty())
      {
        std::ofstream ofs(log_path, std::ios::app);
        if (!ofs.is_open())
        {
          RCLCPP_ERROR(node_->get_logger(),
                       "ActionTimer '%s': cannot open log file '%s'",
                       action_name.c_str(), log_path.c_str());
        }
        else
        {
          // Simple CSV-like format: action_name;elapsed_seconds
          ofs << action_name << ";" << elapsed_sec << std::endl;
        }
      }

      return BT::NodeStatus::SUCCESS;
    }
  }

private:
  std::shared_ptr<rclcpp::Node> node_;
};

