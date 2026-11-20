#pragma once

#include <cmath>
#include <memory>
#include <string>
#include "nav2_behavior_tree/bt_action_node.hpp"

namespace opera_behavior_tree
{

// Angle limits, dump clipping and arrival detection belong to the action server.
template<class ActionT>
class SetAngleAction : public nav2_behavior_tree::BtActionNode<ActionT>
{
public:
  using Base = nav2_behavior_tree::BtActionNode<ActionT>;
  using Base::Base;

  static BT::PortsList providedPorts()
  {
    return Base::providedBasicPorts({
      BT::InputPort<double>("target_angle", "Target angle [rad]"),
      BT::OutputPort<double>("current_error", "Server feedback error [rad]")});
  }

  void on_tick() override
  {
    double angle = 0.0;
    if (!this->getInput("target_angle", angle) || !std::isfinite(angle)) {
      RCLCPP_ERROR(this->node_->get_logger(), "target_angle must be a finite number in radians");
      this->should_send_goal_ = false;
      return;
    }
    this->goal_.target_angle = angle;
  }

  void on_wait_for_result(std::shared_ptr<const typename ActionT::Feedback> feedback) override
  {
    if (feedback) {
      this->setOutput("current_error", feedback->current_error);
    }
  }

  BT::NodeStatus on_success() override
  {
    return this->result_.result && this->result_.result->success ?
           BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus on_cancelled() override
  {
    return BT::NodeStatus::FAILURE;
  }

  void halt() override
  {
    // Also resolve a pending acceptance before asking the base class to cancel.
    // Otherwise a halt just after send_goal could leave the accepted goal running.
    if (this->future_goal_handle_) {
      auto & executor = this->callback_group_executor_;
      if (executor.spin_until_future_complete(
          *this->future_goal_handle_, this->server_timeout_) == rclcpp::FutureReturnCode::SUCCESS)
      {
        this->goal_handle_ = this->future_goal_handle_->get();
        this->future_goal_handle_.reset();
      } else {
        RCLCPP_ERROR(this->node_->get_logger(),
          "Goal acceptance timed out during halt; cancellation could not be confirmed");
      }
    }
    Base::halt();
  }
};

}  // namespace opera_behavior_tree
