#include "opera_behavior_tree/plugins/action/set_angle_action.hpp"
#include "com3_msgs/action/set_dump_angle.hpp"
#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  using Node = opera_behavior_tree::SetAngleAction<com3_msgs::action::SetDumpAngle>;
  factory.registerBuilder<Node>("SetDumpAngle",
    [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<Node>(name, "set_dump_angle", config);
    });
}
