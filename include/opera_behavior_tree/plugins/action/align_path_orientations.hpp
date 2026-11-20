#ifndef OPERA_BEHAVIOR_TREE__PLUGINS__ACTION__ALIGN_PATH_ORIENTATIONS_HPP_
#define OPERA_BEHAVIOR_TREE__PLUGINS__ACTION__ALIGN_PATH_ORIENTATIONS_HPP_

#include "behaviortree_cpp/action_node.h"
#include "nav_msgs/msg/path.hpp"

namespace opera_behavior_tree
{
// For forward-only paths: preserve positions, headers and the final goal orientation.
class AlignPathOrientations : public BT::SyncActionNode
{
public:
  AlignPathOrientations(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};
}  // namespace opera_behavior_tree
#endif
