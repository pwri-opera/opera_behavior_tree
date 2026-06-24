#pragma once

#include "nav2_behavior_tree/bt_service_node.hpp"
#include "rclcpp/rclcpp.hpp"

#include "opera_bt_msgs/srv/get_area.hpp"


namespace opera_behavior_tree
{

class GetAreaService : public nav2_behavior_tree::BtServiceNode<opera_bt_msgs::srv::GetArea>
{

public:
    GetAreaService(
        const std::string & service_node_name,
        const BT::NodeConfiguration & conf);

    void on_tick() override;

    BT::NodeStatus on_completion(std::shared_ptr<opera_bt_msgs::srv::GetArea::Response> response) override;

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts(
        {
            BT::InputPort<geometry_msgs::msg::PoseStamped>(
            "pose",
            "Current robot pose"),
            BT::OutputPort<unsigned int>(
            "area_id",
            "The area ID")
        });
    }
};

}  // namespace opera_behavior_tree