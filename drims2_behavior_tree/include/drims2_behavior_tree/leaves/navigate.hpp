#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/plugins.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

namespace BT
{

template<>
inline std::vector<double> convertFromString(StringView str)
{
  std::vector<double> vec;
  auto parts = splitString(str, ';');
  for (auto & element: parts) {
    vec.push_back(convertFromString<double>(element));
  }
  return vec;
}

}  // namespace BT

class Navigate : public BT::RosActionNode<nav2_msgs::action::NavigateToPose>
{
public:
  Navigate(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
    {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("pose_target"),
      BT::InputPort<std::string>("frame_id"),
      BT::InputPort<std::vector<double>>("position"),
      BT::InputPort<std::vector<double>>("orientation"),
      BT::InputPort<std::string>("behavior_tree"),
    }
    );
  }

  bool setGoal(Goal & goal) override;
  BT::NodeStatus onResultReceived(const WrappedResult & wr) override;
  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;
  BT::NodeStatus onFeedback(
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback) override;

};
