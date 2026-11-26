#include "opera_behavior_tree/plugins/action/align_path_orientations.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include "behaviortree_cpp/bt_factory.h"
#include "nav2_behavior_tree/bt_utils.hpp"

namespace opera_behavior_tree
{
namespace
{
constexpr double pi = 3.14159265358979323846;
constexpr double epsilon = 1e-6;
using Point = geometry_msgs::msg::Point;

double distance(const Point & a, const Point & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

bool validPath(const nav_msgs::msg::Path & path, const std::string & frame)
{
  if (path.header.frame_id != frame || path.poses.empty()) {
    return false;
  }
  for (const auto & pose : path.poses) {
    const auto & p = pose.pose.position;
    if ((!pose.header.frame_id.empty() && pose.header.frame_id != frame) ||
      !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
    {
      return false;
    }
  }
  return true;
}

bool yawOf(const geometry_msgs::msg::Quaternion & q, double & yaw)
{
  const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
  if (!std::isfinite(norm) || norm < epsilon) {
    return false;
  }
  const double x = q.x / norm, y = q.y / norm, z = q.z / norm, w = q.w / norm;
  yaw = std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  return true;
}

void setYaw(geometry_msgs::msg::Quaternion & q, double yaw)
{
  q.x = q.y = 0.0;
  q.z = std::sin(yaw / 2);
  q.w = std::cos(yaw / 2);
}
}  // namespace

AlignPathOrientations::AlignPathOrientations(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config) {}

BT::PortsList AlignPathOrientations::providedPorts()
{
  return {
    BT::InputPort<std::string>("mode", "forward", "forward or route_entry"),
    BT::InputPort<nav_msgs::msg::Path>("input_path"),
    BT::InputPort<nav_msgs::msg::Path>("unsmoothed_path"),
    BT::InputPort<nav_msgs::msg::Path>("route_path"),
    BT::InputPort<geometry_msgs::msg::PoseStamped>("current_pose"),
    BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
    BT::InputPort<double>("entry_tolerance", 0.5, "XY radius at smoothed route entry [m]"),
    BT::InputPort<double>("tangent_distance", 1.0, "Arc length used for entry direction [m]"),
    BT::BidirectionalPort<std::string>("direction", "undecided, forward, reverse"),
    BT::OutputPort<nav_msgs::msg::Path>("output_path")};
}

BT::NodeStatus AlignPathOrientations::tick()
{
  std::string mode;
  if (!getInput("mode", mode)) {
    return BT::NodeStatus::FAILURE;
  }
  if (mode == "route_entry") {
    return tickRouteEntry();
  }
  if (mode != "forward") {
    return BT::NodeStatus::FAILURE;
  }
  nav_msgs::msg::Path path;
  if (!getInput("input_path", path) || path.poses.empty()) {
    return BT::NodeStatus::FAILURE;
  }
  for (const auto & pose : path.poses) {
    const auto & p = pose.pose.position;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      return BT::NodeStatus::FAILURE;
    }
  }

  // Traverse backwards so duplicate positions can share the next distinct point.
  // A trailing run of duplicates retains its original orientations.
  size_t next = path.poses.size() - 1;
  for (size_t i = next; i > 0; ) {
    --i;
    const auto & p = path.poses[i].pose.position;
    const auto & q = path.poses[next].pose.position;
    const double dx = q.x - p.x;
    const double dy = q.y - p.y;
    if (std::hypot(dx, dy) <= 1e-6) {
      if (next + 1 < path.poses.size()) {
        path.poses[i].pose.orientation = path.poses[next].pose.orientation;
      }
      continue;
    }
    const double yaw = std::atan2(dy, dx);
    auto & orientation = path.poses[i].pose.orientation;
    orientation.x = 0.0;
    orientation.y = 0.0;
    orientation.z = std::sin(yaw * 0.5);
    orientation.w = std::cos(yaw * 0.5);
    next = i;
  }
  setOutput("output_path", path);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus AlignPathOrientations::tickRouteEntry()
{
  nav_msgs::msg::Path path, original, route;
  geometry_msgs::msg::PoseStamped pose, goal;
  std::string direction;
  double tolerance, lookahead;
  if (!getInput("input_path", path) || !getInput("unsmoothed_path", original) ||
    !getInput("route_path", route) || !getInput("current_pose", pose) ||
    !getInput("goal", goal) || !getInput("direction", direction) ||
    !getInput("entry_tolerance", tolerance) || !getInput("tangent_distance", lookahead) ||
    !std::isfinite(tolerance) || tolerance <= 0 ||
    !std::isfinite(lookahead) || lookahead <= 0)
  {
    return BT::NodeStatus::FAILURE;
  }
  const auto & frame = path.header.frame_id;
  if (frame.empty() || pose.header.frame_id != frame || goal.header.frame_id != frame ||
    !validPath(path, frame) || !validPath(original, frame) || !validPath(route, frame) ||
    path.poses.size() != original.poses.size() || route.poses.size() > original.poses.size() ||
    !std::isfinite(pose.pose.position.x) || !std::isfinite(pose.pose.position.y) ||
    (direction != "undecided" && direction != "forward" && direction != "reverse"))
  {
    return BT::NodeStatus::FAILURE;
  }
  double robot_yaw;
  if (!yawOf(pose.pose.orientation, robot_yaw)) {
    return BT::NodeStatus::FAILURE;
  }

  // ConcatenatePaths preserves all points, and SimpleSmoother preserves their count/order.
  // Locate the entire original route, rather than a nearest point on a crossing/loop.
  size_t begin = original.poses.size();
  for (size_t i = 0; i + route.poses.size() <= original.poses.size(); ++i) {
    bool matches = true;
    for (size_t j = 0; j < route.poses.size(); ++j) {
      if (distance(original.poses[i + j].pose.position, route.poses[j].pose.position) > epsilon) {
        matches = false;
        break;
      }
    }
    if (matches) {
      // Concatenation can duplicate the entry pose at the end of first mile.
      // For an all-duplicate route no tangent exists; handle it below.
      const bool has_length = std::any_of(route.poses.begin(), route.poses.end(),
        [&route](const auto & p) {
          return distance(p.pose.position, route.poses.front().pose.position) > epsilon;
        });
      if (begin != original.poses.size() && has_length)
      {
        return BT::NodeStatus::FAILURE;
      }
      begin = i;
    }
  }
  if (begin == original.poses.size()) {
    return BT::NodeStatus::FAILURE;
  }
  const size_t end = begin + route.poses.size() - 1;

  // A new navigation goal may preempt the existing one without halting the BT.
  // The root BT also resets direction at the start of each navigation invocation.
  if (!goal_ || *goal_ != goal) {
    direction = "undecided";
    goal_ = goal;
  }

  size_t tangent_end = begin;
  double length = 0;
  while (tangent_end < end && length < lookahead) {
    length += distance(path.poses[tangent_end].pose.position,
      path.poses[tangent_end + 1].pose.position);
    ++tangent_end;
  }
  const auto & entry = path.poses[begin].pose.position;
  const auto & target = path.poses[tangent_end].pose.position;
  if (distance(entry, target) <= epsilon) {
    // A single-point/zero-length route has no direction to choose.
    setOutput("direction", direction);
    setOutput("output_path", path);
    return BT::NodeStatus::SUCCESS;
  }
  if (direction == "undecided" && distance(pose.pose.position, entry) <= tolerance) {
    const double tangent = std::atan2(target.y - entry.y, target.x - entry.x);
    const double delta = std::atan2(std::sin(tangent - robot_yaw), std::cos(tangent - robot_yaw));
    // Exactly sideways has no preference; use forward at the tie.
    direction = std::abs(delta) > pi / 2 + 1e-12 ? "reverse" : "forward";
  }
  if (direction != "undecided") {
    const double offset = direction == "reverse" ? pi : 0;
    double last_yaw = std::atan2(target.y - entry.y, target.x - entry.x) + offset;
    // Preserve final requested goal pose and all its terminal duplicates.
    size_t goal_begin = path.poses.size() - 1;
    while (goal_begin > 0 && distance(path.poses[goal_begin - 1].pose.position,
      path.poses.back().pose.position) <= epsilon)
    {
      --goal_begin;
    }
    for (size_t i = begin; i <= end && i < goal_begin; ++i) {
      size_t next = i + 1;
      while (next <= end && distance(path.poses[i].pose.position,
        path.poses[next].pose.position) <= epsilon)
      {
        ++next;
      }
      if (next <= end) {
        const auto & p = path.poses[i].pose.position;
        const auto & q = path.poses[next].pose.position;
        last_yaw = std::atan2(q.y - p.y, q.x - p.x) + offset;
      }
      setYaw(path.poses[i].pose.orientation, last_yaw);
    }
  }
  setOutput("direction", direction);
  setOutput("output_path", path);
  return BT::NodeStatus::SUCCESS;
}
}  // namespace opera_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<opera_behavior_tree::AlignPathOrientations>("AlignPathOrientations");
}
