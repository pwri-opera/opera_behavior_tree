#include <cmath>
#include <limits>
#include "gtest/gtest.h"
#include "behaviortree_cpp/bt_factory.h"
#include "nav_msgs/msg/path.hpp"
#include "nav2_behavior_tree/bt_utils.hpp"

class GoalOrientationTest : public ::testing::Test
{
protected:
  BT::BehaviorTreeFactory factory;
  BT::Blackboard::Ptr bb = BT::Blackboard::create();
  nav_msgs::msg::Path path;
  geometry_msgs::msg::PoseStamped goal;
  void SetUp() override
  {
    factory.registerFromPlugin(GOAL_PLUGIN_PATH);
    path.header.frame_id = "map";
    path.header.stamp.sec = 42;
    // Reproduce tuning_05: a short southbound path with duplicate zero-yaw endpoints.
    for (double y : {-20.0, -20.5, -21.0, -21.734, -21.734}) {
      geometry_msgs::msg::PoseStamped p;
      p.header = path.header;
      p.pose.position.x = 89.803;
      p.pose.position.y = y;
      p.pose.orientation.w = 1;
      path.poses.push_back(p);
    }
    goal = path.poses.back();
    goal.pose.orientation.z = std::sin(-80.917 * std::acos(-1.0) / 360.0);
    goal.pose.orientation.w = std::cos(-80.917 * std::acos(-1.0) / 360.0);
  }
  BT::NodeStatus run()
  {
    bb->set("path", path);
    bb->set("goal", goal);
    auto tree = factory.createTreeFromText(
      R"(<root BTCPP_format="4"><BehaviorTree ID="Main">
        <SetPathGoalOrientation input_path="{path}" goal="{goal}" output_path="{path}"/>
      </BehaviorTree></root>)", bb);
    return tree.tickOnce();
  }
};

TEST_F(GoalOrientationTest, CorrectsDuplicateEndpointsAndPreservesOtherData)
{
  ASSERT_EQ(run(), BT::NodeStatus::SUCCESS);
  const auto out = bb->get<nav_msgs::msg::Path>("path");
  auto expected = path;
  expected.poses[3].pose.orientation = goal.pose.orientation;
  expected.poses[4].pose.orientation = goal.pose.orientation;
  EXPECT_EQ(out, expected);
  // A single-point route must also receive the actual arrival orientation.
  path.poses.resize(1);
  goal.pose.position = path.poses[0].pose.position;
  ASSERT_EQ(run(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bb->get<nav_msgs::msg::Path>("path").poses[0].pose.orientation,
    goal.pose.orientation);
}

TEST_F(GoalOrientationTest, NearEndpointDoesNotOverwriteDistinctPrecedingPoint)
{
  path.poses[3].pose.position.y -= 0.01;
  goal.pose.position.x += 0.05;
  ASSERT_EQ(run(), BT::NodeStatus::SUCCESS);
  const auto out = bb->get<nav_msgs::msg::Path>("path");
  EXPECT_EQ(out.poses[3], path.poses[3]);
  EXPECT_EQ(out.poses[4].pose.orientation, goal.pose.orientation);
}

TEST_F(GoalOrientationTest, RejectsMismatchedGoalWithoutChangingInput)
{
  goal.header.frame_id = "odom";
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(bb->get<nav_msgs::msg::Path>("path"), path);
  goal.header.frame_id = "map";
  goal.pose.position.x += 1.0;
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(bb->get<nav_msgs::msg::Path>("path"), path);
}

TEST_F(GoalOrientationTest, ValidatesQuaternionAndInputs)
{
  goal.pose.orientation = geometry_msgs::msg::Quaternion{};
  goal.pose.orientation.w = 0;
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
  goal.pose.orientation.w = 2;
  ASSERT_EQ(run(), BT::NodeStatus::SUCCESS);
  EXPECT_DOUBLE_EQ(bb->get<nav_msgs::msg::Path>("path").poses.back().pose.orientation.w, 1);
  goal.pose.orientation.w = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
  goal.pose.orientation.w = 1;
  path.poses[0].pose.position.x = std::numeric_limits<double>::infinity();
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
  path.poses.clear();
  EXPECT_EQ(run(), BT::NodeStatus::FAILURE);
  auto tree = factory.createTreeFromText(
    R"(<root BTCPP_format="4"><BehaviorTree ID="Main">
      <SetPathGoalOrientation input_path="{missing}" goal="{goal}" output_path="{out}"/>
    </BehaviorTree></root>)", bb);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}
