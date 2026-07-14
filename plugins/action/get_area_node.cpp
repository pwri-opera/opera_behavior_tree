#include "opera_behavior_tree/plugins/action/get_area_node.hpp"

namespace opera_behavior_tree
{

GetAreaService::GetAreaService(
  const std::string & service_node_name,
  const BT::NodeConfiguration & conf)
: nav2_behavior_tree::BtServiceNode<opera_bt_msgs::srv::GetArea>(service_node_name, conf,
  "get_area")
{
}

void opera_behavior_tree::GetAreaService::on_tick()
{
  request_ = std::make_shared<opera_bt_msgs::srv::GetArea::Request>();
  getInput("pose", request_->pose);
}

BT::NodeStatus GetAreaService::on_completion(
  std::shared_ptr<opera_bt_msgs::srv::GetArea::Response> response)
{
  // if (!response->success) {
  //   RCLCPP_ERROR(
  //     node_->get_logger(),
  //     "GetArea service call failed");
  //   setOutput("area_id", 0);
  //   return BT::NodeStatus::FAILURE;
  // }

  setOutput("area_id", response->area_id);
  setOutput("entry_node_id", response->entry_node_id);
  setOutput("exit_node_id", response->exit_node_id);
  setOutput("entry_node_pose", response->entry_node_pose);
  setOutput("exit_node_pose", response->exit_node_pose);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace opera_behavior_tree


#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<opera_behavior_tree::GetAreaService>("GetArea");
}
