#include <chrono>
#include <cmath>
#include <csignal>
#include <string>
#include <thread>
#include "ament_index_cpp/get_package_prefix.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "rclcpp/rclcpp.hpp"

namespace
{
volatile std::sig_atomic_t stop_requested = 0;
void request_stop(int) {stop_requested = 1;}
}

int main(int argc, char ** argv)
{
  // Keep ROS alive while haltTree() cancels the current action on SIGINT/SIGTERM.
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, request_stop);
  std::signal(SIGTERM, request_stop);
  auto node = std::make_shared<rclcpp::Node>("angle_tree_runner");
  int exit_code = 1;
  try {
    const auto xml = node->declare_parameter<std::string>("tree_xml", "");
    const auto timeout = node->declare_parameter<double>("execution_timeout", 120.0);
    const auto server_ms = node->declare_parameter<int>("server_timeout_ms", 2000);
    if (xml.empty() ||
      !std::isfinite(timeout) || timeout <= 0 || server_ms <= 0)
    {
      throw std::runtime_error("Invalid tree_xml or timeout parameters");
    }
    BT::BehaviorTreeFactory factory;
    const auto lib_dir = ament_index_cpp::get_package_prefix("opera_behavior_tree") + "/lib/";
    for (const auto & lib : {
        "set_dump_angle_bt_node", "set_swing_angle_bt_node", "navigate_to_pose_goal_bt_node"})
    {
      factory.registerFromPlugin(lib_dir + BT::SharedLibrary::getOSName(lib));
    }
    auto bb = BT::Blackboard::create();
    bb->set("node", node);
    bb->set("bt_loop_duration", std::chrono::milliseconds(10));
    bb->set("server_timeout", std::chrono::milliseconds(server_ms));
    bb->set("wait_for_service_timeout", std::chrono::milliseconds(server_ms));
    auto tree = factory.createTreeFromFile(xml, bb);
    const auto start = std::chrono::steady_clock::now();
    try {
      BT::NodeStatus status = BT::NodeStatus::RUNNING;
      while (!stop_requested && rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
        if (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() >=
          timeout)
        {
          RCLCPP_ERROR(node->get_logger(), "Behavior tree execution timed out");
          break;
        }
        status = tree.tickOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      exit_code = stop_requested ? 130 : (status == BT::NodeStatus::SUCCESS ? 0 : 1);
      tree.haltTree();
      RCLCPP_INFO(node->get_logger(), "Behavior tree finished: exit code %d", exit_code);
    } catch (...) {
      tree.haltTree();
      throw;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "%s", e.what());
  }
  rclcpp::shutdown();
  return exit_code;
}
