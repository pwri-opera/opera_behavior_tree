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

// ref: navigation2/nav2_behavior_tree/back_up_action.hpp

#pragma once

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "rclcpp/rclcpp.hpp"

#include "opera_bt_msgs/action/check_safety.hpp"
#include "behaviortree_cpp/action_node.h"

namespace check_safety
{

class CheckSafetyAction : public nav2_behavior_tree::BtActionNode<opera_bt_msgs::action::CheckSafety>
{
    using Action = opera_bt_msgs::action::CheckSafety;
    using ActionResult = Action::Result;

public:
  /**
   * @brief A constructor for nav2_behavior_tree::BackUpAction
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
    CheckSafetyAction(
        const std::string & xml_tag_name,
        const std::string & action_name,
        const BT::NodeConfiguration & conf);

    /**
     * @brief Function to perform some user-defined operation on tick
     */
    void on_tick() override;

    /**
     * @brief Function to perform some user-defined operation upon successful completion of the action
     */
    BT::NodeStatus on_success() override;

    /**
     * @brief Function to perform some user-defined operation upon abortion of the action
     */
    BT::NodeStatus on_aborted() override;

    /**
     * @brief Function to perform some user-defined operation upon cancellation of the action
     */
    BT::NodeStatus on_cancelled() override;


    void initialize();

    /**
     * @brief Function to perform work in a BT Node when the action server times out
     * Such as setting the error code ID status to timed out for action clients.
     */
    void on_timeout() override;
};

}  // namepspace check_safety