#include "opera_behavior_tree/plugins/action/align_path_orientations.hpp"

#include <cmath>
#include "behaviortree_cpp/bt_factory.h"
#include "nav2_behavior_tree/bt_utils.hpp"

namespace opera_behavior_tree
{
AlignPathOrientations::AlignPathOrientations(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config) {}

BT::PortsList AlignPathOrientations::providedPorts()
{
  return {
    BT::InputPort<nav_msgs::msg::Path>("input_path"),
    BT::OutputPort<nav_msgs::msg::Path>("output_path")};
}

BT::NodeStatus AlignPathOrientations::tick()
{
  nav_msgs::msg::Path path;
  if (!getInput("input_path", path) || path.poses.empty()) {
    return BT::NodeStatus::FAILURE;
  }
  for (const auto & pose : path.poses) {
    const auto & p = pose.pose.position;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      return BT::NodeStatus::FAILURE;
    }
  }

  // Traverse backwards so duplicate positions can share the next distinct point.
  // A trailing run of duplicates retains its original orientations.
  size_t next = path.poses.size() - 1;
  for (size_t i = next; i > 0; ) {
    --i;
    const auto & p = path.poses[i].pose.position;
    const auto & q = path.poses[next].pose.position;
    const double dx = q.x - p.x;
    const double dy = q.y - p.y;
    if (std::hypot(dx, dy) <= 1e-6) {
      if (next + 1 < path.poses.size()) {
        path.poses[i].pose.orientation = path.poses[next].pose.orientation;
      }
      continue;
    }
    const double yaw = std::atan2(dy, dx);
    auto & orientation = path.poses[i].pose.orientation;
    orientation.x = 0.0;
    orientation.y = 0.0;
    orientation.z = std::sin(yaw * 0.5);
    orientation.w = std::cos(yaw * 0.5);
    next = i;
  }
  setOutput("output_path", path);
  return BT::NodeStatus::SUCCESS;
}
}  // namespace opera_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<opera_behavior_tree::AlignPathOrientations>("AlignPathOrientations");
}
