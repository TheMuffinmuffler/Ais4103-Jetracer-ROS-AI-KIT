#include "ris/navigation/astar_global_planner.h"
#include "ris/navigation/grid_utils.h"
#include "ris/navigation/planner_utils.h"
#include "ris/navigation/pose_utils.h"
#include <pluginlib/class_list_macros.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>
#include <vector>
#include <tf2/utils.h>

namespace ris_navigation
{
namespace
{
struct NodeRecord
{
  int index;
  double f_score;

  bool operator>(const NodeRecord& other) const
  {
    return f_score > other.f_score;
  }
};

inline int toIndex(unsigned int x, unsigned int y, unsigned int size_x)
{
  return static_cast<int>(y * size_x + x);
}

inline void fromIndex(int index, unsigned int size_x, unsigned int& x, unsigned int& y)
{
  x = static_cast<unsigned int>(index % static_cast<int>(size_x));
  y = static_cast<unsigned int>(index / static_cast<int>(size_x));
}

inline double heuristic(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2)
{
  const double dx = static_cast<double>(static_cast<int>(x1) - static_cast<int>(x2));
  const double dy = static_cast<double>(static_cast<int>(y1) - static_cast<int>(y2));
  return std::sqrt(dx * dx + dy * dy);
}
}

AStarGlobalPlanner::AStarGlobalPlanner()
  : initialized_(false),
    costmap_ros_(nullptr),
    costmap_(nullptr),
    allow_unknown_(false),
    use_eight_connected_(true),
    simplify_path_(true),
    goal_tolerance_(0.20),
    max_goal_search_radius_(0.80),
    lethal_cost_threshold_(252),
    neutral_cost_threshold_(100)
{
}

AStarGlobalPlanner::AStarGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  : AStarGlobalPlanner()
{
  initialize(name, costmap_ros);
}

void AStarGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
  if (initialized_)
    return;

  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();

  ros::NodeHandle private_nh("~/" + name);
  private_nh.param("allow_unknown", allow_unknown_, false);
  private_nh.param("use_eight_connected", use_eight_connected_, true);
  private_nh.param("simplify_path", simplify_path_, true);
  private_nh.param("goal_tolerance", goal_tolerance_, 0.20);
  private_nh.param("max_goal_search_radius", max_goal_search_radius_, 0.80);
  private_nh.param("lethal_cost_threshold", lethal_cost_threshold_, 252);
  private_nh.param("neutral_cost_threshold", neutral_cost_threshold_, 100);

  initialized_ = true;
  ROS_INFO("AStarGlobalPlanner initialized.");
}

bool AStarGlobalPlanner::makePlan(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal,
    std::vector<geometry_msgs::PoseStamped>& plan)
{
  plan.clear();

  if (!initialized_ || costmap_ == nullptr)
  {
    ROS_ERROR("AStarGlobalPlanner not initialized.");
    return false;
  }

  if (start.header.frame_id != costmap_ros_->getGlobalFrameID() ||
      goal.header.frame_id != costmap_ros_->getGlobalFrameID())
  {
    ROS_WARN("AStarGlobalPlanner requires start and goal in frame: %s",
             costmap_ros_->getGlobalFrameID().c_str());
    return false;
  }

  unsigned int start_mx = 0, start_my = 0, goal_mx = 0, goal_my = 0;

  if (!worldToMap(*costmap_, start.pose.position.x, start.pose.position.y, start_mx, start_my))
  {
    ROS_WARN("A* planner: start outside costmap.");
    return false;
  }

  if (!worldToMap(*costmap_, goal.pose.position.x, goal.pose.position.y, goal_mx, goal_my))
  {
    ROS_WARN("A* planner: raw goal outside costmap.");
    return false;
  }

  unsigned int valid_goal_mx = goal_mx;
  unsigned int valid_goal_my = goal_my;

  if (!isCellTraversable(*costmap_,
                         goal_mx,
                         goal_my,
                         static_cast<unsigned char>(lethal_cost_threshold_),
                         allow_unknown_))
  {
    if (!findNearestValidCell(*costmap_,
                              goal_mx,
                              goal_my,
                              max_goal_search_radius_,
                              static_cast<unsigned char>(lethal_cost_threshold_),
                              allow_unknown_,
                              valid_goal_mx,
                              valid_goal_my))
    {
      ROS_WARN("A* planner failed to find a valid goal cell near requested goal.");
      return false;
    }
  }

  const unsigned int size_x = costmap_->getSizeInCellsX();
  const unsigned int size_y = costmap_->getSizeInCellsY();
  const int total = static_cast<int>(size_x * size_y);

  const int start_index = toIndex(start_mx, start_my, size_x);
  const int goal_index = toIndex(valid_goal_mx, valid_goal_my, size_x);

  std::vector<double> g_score(total, std::numeric_limits<double>::infinity());
  std::vector<int> parent(total, -1);
  std::vector<bool> closed(total, false);

  std::priority_queue<NodeRecord, std::vector<NodeRecord>, std::greater<NodeRecord>> open;
  g_score[start_index] = 0.0;
  parent[start_index] = start_index;
  open.push({start_index, heuristic(start_mx, start_my, valid_goal_mx, valid_goal_my)});

  const int dx4[4] = {1, -1, 0, 0};
  const int dy4[4] = {0, 0, 1, -1};

  const int dx8[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  const int dy8[8] = {0, 0, 1, -1, 1, -1, 1, -1};

  while (!open.empty())
  {
    const NodeRecord current_record = open.top();
    open.pop();

    const int current = current_record.index;
    if (closed[current])
      continue;

    closed[current] = true;
    if (current == goal_index)
      break;

    unsigned int cx = 0, cy = 0;
    fromIndex(current, size_x, cx, cy);

    const int* dx = use_eight_connected_ ? dx8 : dx4;
    const int* dy = use_eight_connected_ ? dy8 : dy4;
    const int move_count = use_eight_connected_ ? 8 : 4;

    for (int i = 0; i < move_count; ++i)
    {
      const int nx_i = static_cast<int>(cx) + dx[i];
      const int ny_i = static_cast<int>(cy) + dy[i];

      if (nx_i < 0 || ny_i < 0 || nx_i >= static_cast<int>(size_x) || ny_i >= static_cast<int>(size_y))
        continue;

      const unsigned int nx = static_cast<unsigned int>(nx_i);
      const unsigned int ny = static_cast<unsigned int>(ny_i);

      if (!isCellTraversable(*costmap_,
                             nx,
                             ny,
                             static_cast<unsigned char>(lethal_cost_threshold_),
                             allow_unknown_))
      {
        continue;
      }

      if (use_eight_connected_ && dx[i] != 0 && dy[i] != 0)
      {
        const unsigned int adj1_x = static_cast<unsigned int>(static_cast<int>(cx) + dx[i]);
        const unsigned int adj1_y = cy;
        const unsigned int adj2_x = cx;
        const unsigned int adj2_y = static_cast<unsigned int>(static_cast<int>(cy) + dy[i]);

        if (!isCellTraversable(*costmap_, adj1_x, adj1_y,
                               static_cast<unsigned char>(lethal_cost_threshold_), allow_unknown_) ||
            !isCellTraversable(*costmap_, adj2_x, adj2_y,
                               static_cast<unsigned char>(lethal_cost_threshold_), allow_unknown_))
        {
          continue;
        }
      }

      const int neighbor = toIndex(nx, ny, size_x);
      if (closed[neighbor])
        continue;

      const bool diagonal = (dx[i] != 0 && dy[i] != 0);
      const double step_cost = diagonal ? 1.41421356237 : 1.0;
      const double cell_cost = static_cast<double>(costmap_->getCost(nx, ny));

      const double penalty = (cell_cost >= 1.0) ? (cell_cost / 255.0) : 0.0;
      const double tentative_g = g_score[current] + step_cost + penalty;

      if (tentative_g < g_score[neighbor])
      {
        g_score[neighbor] = tentative_g;
        parent[neighbor] = current;
        const double f = tentative_g + heuristic(nx, ny, valid_goal_mx, valid_goal_my);
        open.push({neighbor, f});
      }
    }
  }

  if (parent[goal_index] == -1)
  {
    ROS_WARN("A* planner failed to find a path.");
    return false;
  }

  std::vector<int> path_indices;
  int cur = goal_index;
  while (cur != start_index)
  {
    path_indices.push_back(cur);
    cur = parent[cur];
    if (cur < 0)
    {
      ROS_WARN("A* planner path reconstruction failed.");
      return false;
    }
  }
  path_indices.push_back(start_index);
  std::reverse(path_indices.begin(), path_indices.end());

  std::vector<geometry_msgs::PoseStamped> raw_plan;
  raw_plan.reserve(path_indices.size());

  for (size_t i = 0; i < path_indices.size(); ++i)
  {
    unsigned int mx = 0, my = 0;
    fromIndex(path_indices[i], size_x, mx, my);

    double wx = 0.0, wy = 0.0;
    mapToWorld(*costmap_, mx, my, wx, wy);

    double yaw = 0.0;
    if (i + 1 < path_indices.size())
    {
      unsigned int next_mx = 0, next_my = 0;
      fromIndex(path_indices[i + 1], size_x, next_mx, next_my);

      double next_wx = 0.0, next_wy = 0.0;
      mapToWorld(*costmap_, next_mx, next_my, next_wx, next_wy);
      yaw = std::atan2(next_wy - wy, next_wx - wx);
    }
    else
    {
      yaw = tf2::getYaw(goal.pose.orientation);
    }

    raw_plan.push_back(makePoseStamped(costmap_ros_->getGlobalFrameID(), wx, wy, yaw));
  }

  if (simplify_path_)
    plan = simplifyPathByDirection(raw_plan);
  else
    plan = raw_plan;

  if (!plan.empty())
  {
    plan.front() = start;
    plan.back() = goal;
  }

  ROS_INFO("AStarGlobalPlanner produced a plan with %zu poses.", plan.size());
  return true;
}
}

PLUGINLIB_EXPORT_CLASS(ris_navigation::AStarGlobalPlanner, nav_core::BaseGlobalPlanner)