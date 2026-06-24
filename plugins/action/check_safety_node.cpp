// Copyright (c) 2018 Intel Corporation
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

// ref: navigation2/nav2_behavior_tree/back_up_action.cpp

#include <string>
#include <memory>
#include "opera_behavior_tree/plugins/action/check_safety_node.hpp"


namespace check_safety
{

CheckSafetyAction::CheckSafetyAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: nav2_behavior_tree::BtActionNode<opera_bt_msgs::action::CheckSafety>(xml_tag_name, action_name, conf)
{
}

void check_safety::CheckSafetyAction::initialize()
{

}


void check_safety::CheckSafetyAction::on_tick()
{
  if (!BT::isStatusActive(status())) {
    initialize();
  }
}


BT::NodeStatus check_safety::CheckSafetyAction::on_success()
{
  return BT::NodeStatus::SUCCESS;
}


BT::NodeStatus check_safety::CheckSafetyAction::on_aborted()
{
  return BT::NodeStatus::FAILURE;
}


BT::NodeStatus check_safety::CheckSafetyAction::on_cancelled()
{
  return BT::NodeStatus::SUCCESS;
}

void check_safety::CheckSafetyAction::on_timeout()
{
}

}  // namespace check_safety


#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<check_safety::CheckSafetyAction>(
        name, "check_safety", config);
    };

  factory.registerBuilder<check_safety::CheckSafetyAction>("CheckSafety", builder);
}
