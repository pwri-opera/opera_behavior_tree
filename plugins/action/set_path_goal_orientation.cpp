#include "opera_behavior_tree/plugins/action/set_path_goal_orientation.hpp"

#include <cmath>
#include "behaviortree_cpp/bt_factory.h"
#include "nav2_behavior_tree/bt_utils.hpp"

namespace opera_behavior_tree
{
SetPathGoalOrientation::SetPathGoalOrientation(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config) {}

BT::PortsList SetPathGoalOrientation::providedPorts()
{
  return {
    BT::InputPort<nav_msgs::msg::Path>("input_path"),
    BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
    BT::InputPort<double>("position_tolerance", 0.1, "Maximum XY endpoint-to-goal distance [m]"),
    BT::OutputPort<nav_msgs::msg::Path>("output_path")};
}

BT::NodeStatus SetPathGoalOrientation::tick()
{
  nav_msgs::msg::Path path;
  geometry_msgs::msg::PoseStamped goal;
  double tolerance;
  if (!getInput("input_path", path) || path.poses.empty() ||
    !getInput("goal", goal) || !getInput("position_tolerance", tolerance) ||
    !std::isfinite(tolerance) || tolerance < 0.0 || path.header.frame_id.empty() ||
    path.header.frame_id != goal.header.frame_id)
  {
    return BT::NodeStatus::FAILURE;
  }
  const auto finite_position = [](const auto & p) {
      return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
    };
  if (!finite_position(goal.pose.position)) {
    return BT::NodeStatus::FAILURE;
  }
  for (const auto & pose : path.poses) {
    if (!finite_position(pose.pose.position) ||
      (!pose.header.frame_id.empty() && pose.header.frame_id != path.header.frame_id))
    {
      return BT::NodeStatus::FAILURE;
    }
  }
  auto orientation = goal.pose.orientation;
  const double norm = std::hypot(std::hypot(orientation.x, orientation.y),
      std::hypot(orientation.z, orientation.w));
  if (!std::isfinite(norm) || norm < 1e-12) {
    return BT::NodeStatus::FAILURE;
  }
  orientation.x /= norm;
  orientation.y /= norm;
  orientation.z /= norm;
  orientation.w /= norm;
  const auto end = path.poses.back().pose.position;
  if (std::hypot(end.x - goal.pose.position.x, end.y - goal.pose.position.y) > tolerance) {
    return BT::NodeStatus::FAILURE;
  }
  // Keep the goal proximity tolerance separate from the duplicate-point epsilon.
  for (auto it = path.poses.rbegin(); it != path.poses.rend(); ++it) {
    const auto & p = it->pose.position;
    if (std::hypot(std::hypot(p.x - end.x, p.y - end.y), p.z - end.z) > 1e-6) {
      break;
    }
    it->pose.orientation = orientation;
  }
  setOutput("output_path", path);
  return BT::NodeStatus::SUCCESS;
}
}  // namespace opera_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<opera_behavior_tree::SetPathGoalOrientation>("SetPathGoalOrientation");
}
