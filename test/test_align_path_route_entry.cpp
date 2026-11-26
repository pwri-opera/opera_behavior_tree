#include <cmath>
#include <limits>
#include "gtest/gtest.h"
#include "behaviortree_cpp/bt_factory.h"
#include "nav_msgs/msg/path.hpp"

namespace
{
constexpr double pi = 3.14159265358979323846;
geometry_msgs::msg::PoseStamped pose(double x, double y, double yaw)
{
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "map";
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation.z = std::sin(yaw / 2);
  p.pose.orientation.w = std::cos(yaw / 2);
  return p;
}
double yaw(const geometry_msgs::msg::PoseStamped & p)
{
  return 2 * std::atan2(p.pose.orientation.z, p.pose.orientation.w);
}
nav_msgs::msg::Path path(std::initializer_list<geometry_msgs::msg::PoseStamped> points)
{
  nav_msgs::msg::Path p;
  p.header.frame_id = "map";
  p.poses = points;
  return p;
}
const char * node_xml = R"(
  <AlignPathOrientations mode="route_entry" input_path="{path}" unsmoothed_path="{original}"
    route_path="{route}" current_pose="{pose}" goal="{goal}"
    direction="{direction}" output_path="{output}"/>)";

class TrackMock : public BT::StatefulActionNode
{
public:
  TrackMock(const std::string & name, const BT::NodeConfiguration & config)
  : BT::StatefulActionNode(name, config) {}
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus onStart() override {return onRunning();}
  BT::NodeStatus onRunning() override
  {
    return config().blackboard->get<bool>("done") ?
           BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
  }
  void onHalted() override {}
};

class RouteEntryTest : public ::testing::Test
{
protected:
  BT::BehaviorTreeFactory factory;
  BT::Blackboard::Ptr bb = BT::Blackboard::create();
  nav_msgs::msg::Path route = path({pose(0, 0, 0), pose(.5, 0, 0),
      pose(1, 0, pi / 2), pose(1, 1, pi / 2), pose(1, 2, pi / 2)});
  nav_msgs::msg::Path full = path({pose(-2, 0, .3), pose(-1, 0, .3),
      pose(0, 0, 0), pose(.5, 0, 0), pose(1, 0, pi / 2),
      pose(1, 1, pi / 2), pose(1, 2, pi / 2), pose(2, 2, .4), pose(3, 2, .7)});

  void SetUp() override
  {
    factory.registerFromPlugin(ENTRY_PLUGIN_PATH);
    bb->set("route", route);
    bb->set("path", full);
    bb->set("original", full);
    bb->set("goal", full.poses.back());
    bb->set("direction", std::string("undecided"));
    bb->set("pose", pose(-2, 0, 0));
  }
  BT::Tree tree()
  {
    return factory.createTreeFromText(
      std::string("<root BTCPP_format=\"4\"><BehaviorTree ID=\"Main\">") +
      node_xml + "</BehaviorTree></root>", bb);
  }
};

TEST_F(RouteEntryTest, DecidesAtEntryNotAtPlanningAndPreservesConnectors)
{
  auto t = tree();
  EXPECT_EQ(t.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bb->get<std::string>("direction"), "undecided");
  EXPECT_EQ(bb->get<nav_msgs::msg::Path>("output"), full);
  // It was facing forward during planning, but arrives facing backwards.
  bb->set("pose", pose(-.2, 0, pi));
  EXPECT_EQ(t.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bb->get<std::string>("direction"), "reverse");
  auto out = bb->get<nav_msgs::msg::Path>("output");
  EXPECT_NEAR(yaw(out.poses[2]), pi, 1e-12);
  EXPECT_NEAR(yaw(out.poses[4]), 1.5 * pi, 1e-12);
  EXPECT_EQ(out.poses[0], full.poses[0]);
  EXPECT_EQ(out.poses[1], full.poses[1]);
  EXPECT_EQ(out.poses[7], full.poses[7]);
  EXPECT_EQ(out.poses.back(), full.poses.back());
  for (size_t i = 0; i < full.poses.size(); ++i) {
    EXPECT_EQ(out.poses[i].pose.position, full.poses[i].pose.position);
    EXPECT_EQ(out.poses[i].header, full.poses[i].header);
  }
  EXPECT_EQ(bb->get<nav_msgs::msg::Path>("path"), full);
}

TEST_F(RouteEntryTest, DirectionSurvivesCurveHaltAndReplanButNotNewGoal)
{
  auto t = tree();
  bb->set("pose", pose(0, 0, pi));
  ASSERT_EQ(t.tickOnce(), BT::NodeStatus::SUCCESS);
  t.haltTree();
  bb->set("pose", pose(1, 1, pi / 2));
  // Replanning updates stamps and path geometry; same goal keeps reverse.
  full.header.stamp.sec = 123;
  full.poses[5].pose.position.x += .1;
  bb->set("path", full);
  ASSERT_EQ(t.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bb->get<std::string>("direction"), "reverse");
  auto goal = full.poses.back();
  goal.header.stamp.sec = 456;
  bb->set("goal", goal);
  ASSERT_EQ(t.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bb->get<std::string>("direction"), "undecided");
  bb->set("pose", pose(0, 0, 0));
  ASSERT_EQ(t.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bb->get<std::string>("direction"), "forward");
}

TEST_F(RouteEntryTest, RepeatedTicksDoNotFlipYawAgain)
{
  auto t = tree();
  bb->set("pose", pose(0, 0, pi));
  ASSERT_EQ(t.tickOnce(), BT::NodeStatus::SUCCESS);
  const auto once = bb->get<nav_msgs::msg::Path>("output");
  ASSERT_EQ(t.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bb->get<nav_msgs::msg::Path>("output"), once);
}

TEST_F(RouteEntryTest, SmoothedEntryAndTangentAreUsed)
{
  // Smoothing moves the entry without changing indices or point count.
  for (auto & p : full.poses) {p.pose.position.y += .8;}
  bb->set("path", full);
  bb->set("pose", pose(0, 0, pi));
  auto t = tree();
  ASSERT_EQ(t.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bb->get<std::string>("direction"), "undecided");
  bb->set("pose", pose(0, .8, pi));
  ASSERT_EQ(t.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bb->get<std::string>("direction"), "reverse");
}

TEST_F(RouteEntryTest, UsesRouteTangentInsteadOfFirstMileDirection)
{
  // Connector points north, route starts east; robot faces west => reverse on route.
  full.poses[0] = pose(0, -2, pi / 2);
  full.poses[1] = pose(0, -1, pi / 2);
  bb->set("path", full);
  bb->set("original", full);
  bb->set("pose", pose(0, 0, pi));
  auto t = tree();
  ASSERT_EQ(t.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bb->get<std::string>("direction"), "reverse");
}

TEST_F(RouteEntryTest, DuplicatesAndTerminalGoalOrientation)
{
  route.poses.insert(route.poses.begin(), route.poses.front());
  route.poses.push_back(route.poses.back());
  route.poses.back().pose.orientation = pose(0, 0, .4).pose.orientation;
  route.poses[route.poses.size() - 2].pose.orientation = route.poses.back().pose.orientation;
  bb->set("route", route);bb->set("original", route);bb->set("path", route);
  bb->set("goal", route.poses.back());bb->set("pose", pose(0, 0, pi));
  auto t = tree();
  ASSERT_EQ(t.tickOnce(), BT::NodeStatus::SUCCESS);
  auto out = bb->get<nav_msgs::msg::Path>("output");
  EXPECT_NEAR(yaw(out.poses[0]), pi, 1e-12);
  EXPECT_NEAR(yaw(out.poses[1]), pi, 1e-12);
  EXPECT_EQ(out.poses.back(), route.poses.back());
  EXPECT_EQ(out.poses[out.poses.size() - 2], route.poses[route.poses.size() - 2]);
}

TEST_F(RouteEntryTest, ZeroLengthRouteDoesNotChooseDirection)
{
  route = path({pose(0, 0, .8)});
  bb->set("route", route);bb->set("original", route);bb->set("path", route);
  bb->set("pose", pose(0, 0, pi));
  auto t = tree();
  EXPECT_EQ(t.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bb->get<std::string>("direction"), "undecided");
  EXPECT_EQ(bb->get<nav_msgs::msg::Path>("output"), route);
}

TEST_F(RouteEntryTest, AngleBoundaryAndWrapAround)
{
  auto t = tree();
  for (auto input : {std::pair<double, const char *>{pi / 2, "forward"},
    {pi / 2 + .01, "reverse"}, {-pi / 2 - .01, "reverse"}, {2 * pi, "forward"}})
  {
    bb->set("direction", std::string("undecided"));
    bb->set("pose", pose(0, 0, input.first));
    EXPECT_EQ(t.tickOnce(), BT::NodeStatus::SUCCESS);
    EXPECT_EQ(bb->get<std::string>("direction"), input.second);
  }
}

TEST_F(RouteEntryTest, RejectsInvalidMappingFrameAndPose)
{
  auto t = tree();
  auto bad = full;bad.poses.pop_back();bb->set("path", bad);
  EXPECT_EQ(t.tickOnce(), BT::NodeStatus::FAILURE);
  bb->set("path", full);
  auto p = pose(0, 0, 0);p.header.frame_id = "odom";bb->set("pose", p);
  EXPECT_EQ(t.tickOnce(), BT::NodeStatus::FAILURE);
  p = pose(0, 0, 0);p.pose.orientation.w = std::numeric_limits<double>::quiet_NaN();bb->set("pose", p);
  EXPECT_EQ(t.tickOnce(), BT::NodeStatus::FAILURE);
  bb->set("pose", pose(0, 0, 0));
  route.poses[1].pose.position.x += 10;bb->set("route", route);
  EXPECT_EQ(t.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(RouteEntryTest, PlanningSnapshotCopiesRosMessages)
{
  bb->set("new_path", full);
  bb->set("route_pending", route);
  auto t = factory.createTreeFromText(
    std::string(R"(<root BTCPP_format="4"><BehaviorTree ID="Main"><Sequence>
      <Script code="route := route_pending; original := new_path; path := new_path"/>)") +
    node_xml + "</Sequence></BehaviorTree></root>", bb);
  bb->set("pose", pose(0, 0, pi));
  ASSERT_EQ(t.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bb->get<std::string>("direction"), "reverse");
  EXPECT_EQ(bb->get<nav_msgs::msg::Path>("path"), full);
}

TEST_F(RouteEntryTest, ReactiveTickWhileFollowingAndSessionReset)
{
  bb->set("done", false);
  factory.registerNodeType<TrackMock>("TrackMock");
  auto t = factory.createTreeFromText(
    std::string(R"(<root BTCPP_format="4"><BehaviorTree ID="Main"><Sequence>
      <Script code="direction := 'undecided'"/><ReactiveSequence>)") + node_xml +
    "<TrackMock/></ReactiveSequence></Sequence></BehaviorTree></root>", bb);
  ASSERT_EQ(t.tickOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(bb->get<std::string>("direction"), "undecided");
  bb->set("pose", pose(0, 0, pi));
  ASSERT_EQ(t.tickOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(bb->get<std::string>("direction"), "reverse");
  bb->set("done", true);
  ASSERT_EQ(t.tickOnce(), BT::NodeStatus::SUCCESS);
  // Identical goal, reused tree: the root script still resets the new session.
  t.haltTree();bb->set("done", false);bb->set("pose", pose(-2, 0, 0));
  ASSERT_EQ(t.tickOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(bb->get<std::string>("direction"), "undecided");
}
}  // namespace
