// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Pablo Iñigo Blasco
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include <memory>

// #include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

#include "opera_behavior_tree/plugins/action/enable_route_switcher_node.hpp"

#include "rclcpp/rclcpp.hpp"

namespace route_behavior_tree
{

using std::placeholders::_1;

EnableRouteSwitcher::EnableRouteSwitcher(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
  initialize();

  // Spin multiple times due to rclcpp regression in Jazzy requiring a 'warm up' spin
  callback_group_executor_.spin_some(std::chrono::nanoseconds(1));
}

void EnableRouteSwitcher::initialize()
{
  createROSInterfaces();
}

void EnableRouteSwitcher::createROSInterfaces()
{
  std::string topic_new;
  getInput("topic_name", topic_new);
  if (topic_new != topic_name_ || !controller_selector_sub_) {
    topic_name_ = topic_new;
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive,
      false);
    callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());


    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.transient_local().reliable();

    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = callback_group_;
    controller_selector_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      topic_name_,
      qos,
      std::bind(&EnableRouteSwitcher::callbackControllerSelect, this, _1),
      sub_option);
  }
}

BT::NodeStatus EnableRouteSwitcher::tick()
{
  if (!BT::isStatusActive(status())) {
    initialize();
  }

  callback_group_executor_.spin_some();

  // This behavior always use the last selected controller received from the topic input.
  // When no input is specified it uses the default controller.
  // If the default controller is not specified then we work in "required controller mode":
  // In this mode, the behavior returns failure if the controller selection is not received from
  // the topic input.
  if (!last_selected_value_) {
    bool default_value;
    getInput("default_value", default_value);
    last_selected_value_ = default_value;
  }

  if (last_selected_value_.value() == true){
    setOutput("enable_route", "enable");
  } else {
    setOutput("enable_route", "disable");
  }
  

  return BT::NodeStatus::SUCCESS;
}

void
EnableRouteSwitcher::callbackControllerSelect(const std_msgs::msg::Bool::SharedPtr msg)
{
  last_selected_value_ = msg->data;
}

}  // namespace route_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<route_behavior_tree::EnableRouteSwitcher>("EnableRouteSwitcher");
}
