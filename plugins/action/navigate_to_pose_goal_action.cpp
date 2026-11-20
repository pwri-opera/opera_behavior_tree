#include <cmath>
#include <memory>
#include <string>
#include "behaviortree_cpp/bt_factory.h"
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

namespace opera_behavior_tree
{
// Send a map goal to the already running Nav2 navigator. Its configured BT
// performs navigation; this outer tree only sequences completed actions.
class NavigateToPoseGoal : public nav2_behavior_tree::BtActionNode<nav2_msgs::action::NavigateToPose>
{
public:
  using Base = nav2_behavior_tree::BtActionNode<nav2_msgs::action::NavigateToPose>;
  using Base::Base;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<double>("x", "Goal x [m]"),
      BT::InputPort<double>("y", "Goal y [m]"),
      BT::InputPort<double>("yaw", "Goal yaw [rad]"),
      BT::InputPort<std::string>("frame_id", "map", "Goal frame")});
  }

  void on_tick() override
  {
    double x = 0.0, y = 0.0, yaw = 0.0;
    std::string frame;
    if (!getInput("x", x) || !getInput("y", y) || !getInput("yaw", yaw) ||
      !getInput("frame_id", frame) || frame.empty() ||
      !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw))
    {
      RCLCPP_ERROR(node_->get_logger(), "Navigation goal requires finite x, y, yaw and a frame");
      should_send_goal_ = false;
      return;
    }
    goal_.pose.header.frame_id = frame;
    goal_.pose.header.stamp = node_->now();
    goal_.pose.pose.position.x = x;
    goal_.pose.pose.position.y = y;
    goal_.pose.pose.orientation.z = std::sin(yaw / 2);
    goal_.pose.pose.orientation.w = std::cos(yaw / 2);
    goal_.behavior_tree.clear();
  }

  BT::NodeStatus on_cancelled() override {return BT::NodeStatus::FAILURE;}

  void halt() override
  {
    // Resolve an in-flight acceptance so halt can cancel that goal too.
    if (future_goal_handle_) {
      if (callback_group_executor_.spin_until_future_complete(
          *future_goal_handle_, server_timeout_) == rclcpp::FutureReturnCode::SUCCESS)
      {
        goal_handle_ = future_goal_handle_->get();
        future_goal_handle_.reset();
      } else {
        RCLCPP_ERROR(node_->get_logger(), "Goal acceptance timed out during navigation halt");
      }
    }
    Base::halt();
  }
};
}  // namespace opera_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerBuilder<opera_behavior_tree::NavigateToPoseGoal>("NavigateToPoseGoal",
    [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<opera_behavior_tree::NavigateToPoseGoal>(
        name, "navigate_to_pose", config);
    });
}
