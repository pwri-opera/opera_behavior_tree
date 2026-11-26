#ifndef OPERA_BEHAVIOR_TREE__PLUGINS__ACTION__ALIGN_PATH_ORIENTATIONS_HPP_
#define OPERA_BEHAVIOR_TREE__PLUGINS__ACTION__ALIGN_PATH_ORIENTATIONS_HPP_

#include <optional>
#include "behaviortree_cpp/action_node.h"
#include "nav_msgs/msg/path.hpp"

namespace opera_behavior_tree
{
// Default: forward-only alignment before smoothing.
// route_entry: latch direction from actual entry pose and orient only the route body.
class AlignPathOrientations : public BT::SyncActionNode
{
public:
  AlignPathOrientations(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  BT::NodeStatus tickRouteEntry();
  std::optional<geometry_msgs::msg::PoseStamped> goal_;
};
}  // namespace opera_behavior_tree
#endif
