#include <drims2_behavior_tree/leaves/navigate.hpp>

Navigate::Navigate(
  const std::string & name,
  const BT::NodeConfig & conf,
  const BT::RosNodeParams & params)
: RosActionNode<nav2_msgs::action::NavigateToPose>(name, conf, params)
{
}

bool Navigate::setGoal(RosActionNode::Goal & goal)
{
  RCLCPP_INFO(node_.lock()->get_logger(), "Navigate ticked.");

  std::string behavior_tree_xml;
  if (!getInput("behavior_tree", behavior_tree_xml)) {
    throw BT::RuntimeError("Missing parameter [behavior_tree_xml]");
  }
  goal.behavior_tree = behavior_tree_xml;

  auto pose_target = getInput<geometry_msgs::msg::PoseStamped>("pose_target");
  if (pose_target) {
    goal.pose = pose_target.value();
    return true;
  }

  // If pose_target is not set, build it from components
  geometry_msgs::msg::PoseStamped pose;

  // Required fields: frame_id, position (3), orientation (4)
  if (!getInput("frame_id", pose.header.frame_id)) {
    throw BT::RuntimeError("Missing parameter [frame_id]");
  }

  std::vector<double> position;
  if (!getInput("position", position) || position.size() != 3) {
    throw BT::RuntimeError("Invalid or missing parameter [position]. Expected 3 elements");
  }

  std::vector<double> orientation;
  if (!getInput("orientation", orientation) || orientation.size() != 4) {
    throw BT::RuntimeError("Invalid or missing parameter [orientation]. Expected 4 elements");
  }

  
  pose.header.stamp = node_.lock()->now();  // timestamp current time
  pose.pose.position.x = position[0];
  pose.pose.position.y = position[1];
  pose.pose.position.z = position[2];
  pose.pose.orientation.x = orientation[0];
  pose.pose.orientation.y = orientation[1];
  pose.pose.orientation.z = orientation[2];
  pose.pose.orientation.w = orientation[3];
  
  goal.pose = pose;
  return true;
}

BT::NodeStatus Navigate::onResultReceived(const RosActionNode::WrappedResult & wr)
{
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus Navigate::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_ERROR( node_.lock()->get_logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error) );
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus Navigate::onFeedback(
  const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
{
  (void) feedback;
  return BT::NodeStatus::RUNNING;
}

// Plugin registration.
// The class Navigate will self register with name  "Navigate".
CreateRosNodePlugin(Navigate, "Navigate");
