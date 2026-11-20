#include <cmath>
#include <limits>
#include "gtest/gtest.h"
#include "behaviortree_cpp/bt_factory.h"
#include "nav_msgs/msg/path.hpp"
#include "nav2_behavior_tree/bt_utils.hpp"

TEST(AlignPathOrientations, PluginAndInPlacePath)
{
  BT::BehaviorTreeFactory factory;
  factory.registerFromPlugin(ALIGN_PLUGIN_PATH);
  auto bb = BT::Blackboard::create();
  auto tree = factory.createTreeFromText(
    R"(<root BTCPP_format="4"><BehaviorTree ID="Main">
      <AlignPathOrientations input_path="{path}" output_path="{path}"/>
    </BehaviorTree></root>)", bb);
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.header.stamp.sec = 42;
  for (const auto & xy : {std::pair<double, double>{2, 0}, {2, 0}, {1, 0}, {1, 1}, {1, 1}}) {
    geometry_msgs::msg::PoseStamped p;
    p.header = path.header;
    p.pose.position.x = xy.first;
    p.pose.position.y = xy.second;
    p.pose.position.z = 3;
    p.pose.orientation.w = 1;
    path.poses.push_back(p);
  }
  path.poses.back().pose.orientation.z = 0.6;
  path.poses.back().pose.orientation.w = 0.8;
  bb->set("path", path);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  auto out = bb->get<nav_msgs::msg::Path>("path");
  EXPECT_EQ(out.header, path.header);
  ASSERT_EQ(out.poses.size(), path.poses.size());
  for (size_t i = 0; i < out.poses.size(); ++i) {
    EXPECT_EQ(out.poses[i].header, path.poses[i].header);
    EXPECT_EQ(out.poses[i].pose.position, path.poses[i].pose.position);
  }
  for (size_t i : {0u, 1u}) {
    EXPECT_NEAR(out.poses[i].pose.orientation.z, 1.0, 1e-12);
    EXPECT_NEAR(out.poses[i].pose.orientation.w, 0.0, 1e-12);
  }
  EXPECT_NEAR(out.poses[2].pose.orientation.z, std::sqrt(0.5), 1e-12);
  EXPECT_EQ(out.poses[3].pose.orientation, path.poses[3].pose.orientation);
  EXPECT_EQ(out.poses.back().pose.orientation, path.poses.back().pose.orientation);

  // One-point paths and all-duplicate paths preserve the goal orientation.
  path.poses = {path.poses.back()};
  bb->set("path", path);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bb->get<nav_msgs::msg::Path>("path"), path);
  path.poses.push_back(path.poses.back());
  bb->set("path", path);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bb->get<nav_msgs::msg::Path>("path"), path);

  path.poses[0].pose.position.x = std::numeric_limits<double>::quiet_NaN();
  bb->set("path", path);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  bb->set("path", nav_msgs::msg::Path{});
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST(AlignPathOrientations, MissingInput)
{
  BT::BehaviorTreeFactory factory;
  factory.registerFromPlugin(ALIGN_PLUGIN_PATH);
  auto tree = factory.createTreeFromText(
    R"(<root BTCPP_format="4"><BehaviorTree ID="Main">
      <AlignPathOrientations input_path="{missing}" output_path="{out}"/>
    </BehaviorTree></root>)");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}
