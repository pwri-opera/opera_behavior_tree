#ifndef OPERA_BEHAVIOR_TREE__PLUGINS__ACTION__SET_PATH_GOAL_ORIENTATION_HPP_
#define OPERA_BEHAVIOR_TREE__PLUGINS__ACTION__SET_PATH_GOAL_ORIENTATION_HPP_

#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace opera_behavior_tree
{
// Updates only the terminal pose and its contiguous positional duplicates.
class SetPathGoalOrientation : public BT::SyncActionNode
{
public:
  SetPathGoalOrientation(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};
}  // namespace opera_behavior_tree
#endif
