#include <rclcpp/rclcpp.hpp>

#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include <behaviortree_ros2/plugins.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

class SetPose : public BT::ConditionNode
{
public:
  SetPose(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {
    node_ = config.blackboard->get<std::shared_ptr<rclcpp::Node>>("node");
  }

  SetPose(const std::string& name,
               const BT::NodeConfiguration& config,
               const BT::RosNodeParams& params)
  : BT::ConditionNode(name, config), node_(params.nh) {}

  
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("frame_id"),
      BT::InputPort<std::vector<double>>("position"),
      BT::InputPort<std::vector<double>>("orientation"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose")
    };
  }

  BT::NodeStatus tick() override
  {
    geometry_msgs::msg::PoseStamped pose;
    
    // Required fields: frame_id, position (3), orientation (4)
    if (!getInput("frame_id", pose.header.frame_id)) {
      throw BT::RuntimeError("SetPose: Missing parameter [frame_id]");
    }
    
    std::vector<double> position;
    if (!getInput("position", position) || position.size() != 3) {
      throw BT::RuntimeError("SetPose: Invalid or missing parameter [position]. Expected 3 elements");
    }
    
    std::vector<double> orientation;
    if (!getInput("orientation", orientation) || orientation.size() != 4) {
      throw BT::RuntimeError("SetPose: Invalid or missing parameter [orientation]. Expected 4 elements");
    }

    pose.header.stamp = node_->now();         // timestamp current time
    pose.pose.position.x = position[0];
    pose.pose.position.y = position[1];
    pose.pose.position.z = position[2];
    pose.pose.orientation.x = orientation[0];
    pose.pose.orientation.y = orientation[1];
    pose.pose.orientation.z = orientation[2];
    pose.pose.orientation.w = orientation[3];

    // Set the pose in the blackboard
    this->setOutput("pose", pose);
    return BT::NodeStatus::SUCCESS;
  }

  private:
    std::shared_ptr<rclcpp::Node> node_;

};