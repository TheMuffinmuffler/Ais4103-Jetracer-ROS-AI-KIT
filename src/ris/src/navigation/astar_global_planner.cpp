#include <ris/navigation/astar_global_planner.h>
#include <pluginlib/class_list_macros.h>
#include <costmap_2d/cost_values.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <queue>
#include <vector>
#include <limits>
#include <cmath>
#include <algorithm>

namespace ris_navigation
{

struct OpenNode
{
  unsigned int idx;
  double f;

  bool operator<(const OpenNode& other) const
  {
    return f > other.f;
  }
};

AStarGlobalPlanner::AStarGlobalPlanner()
  : initialized_(false),
    costmap_ros_(nullptr),
    costmap_(nullptr)
{
}

AStarGlobalPlanner::AStarGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  : initialized_(false),
    costmap_ros_(nullptr),
    costmap_(nullptr)
{
  initialize(name, costmap_ros);
}

void AStarGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
  if (initialized_)
  {
    ROS_WARN("AStarGlobalPlanner already initialized.");
    return;
  }

  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  global_frame_ = costmap_ros_->getGlobalFrameID();
  ros::NodeHandle private_nh("~/" + name);
  private_nh.param("allow_unknown", allow_unknown_, false);
  private_nh.param("simplify_path", simplify_path_, true);
  private_nh.param("obstacle_cost_threshold", obstacle_cost_threshold_,
                   static_cast<int>(costmap_2d::INSCRIBED_INFLATED_OBSTACLE));
  private_nh.param("default_tolerance", default_tolerance_, 0.20);
  private_nh.param("neutral_cost_scale", neutral_cost_scale_, 2.0);
  private_nh.param("goal_search_radius_cells", goal_search_radius_cells_, 8);
  ros::NodeHandle nh;
  plan_pub_ = nh.advertise<nav_msgs::Path>("ris_global_plan", 1, true);
  initialized_ = true;
  ROS_INFO("ris_navigation::AStarGlobalPlanner initialized.");
}

bool AStarGlobalPlanner::worldToMapSafe(double wx, double wy, unsigned int& mx, unsigned int& my) const
{
  if (!costmap_)
    return false;
  return costmap_->worldToMap(wx, wy, mx, my);
}

void AStarGlobalPlanner::mapToWorldCenter(unsigned int mx, unsigned int my, double& wx, double& wy) const
{
  costmap_->mapToWorld(mx, my, wx, wy);
}

bool AStarGlobalPlanner::isCellTraversable(unsigned int mx, unsigned int my) const
{
  if (mx >= costmap_->getSizeInCellsX() || my >= costmap_->getSizeInCellsY())
    return false;

  unsigned char cost = costmap_->getCost(mx, my);

  if (cost == costmap_2d::NO_INFORMATION && allow_unknown_)
    return true;

  if (cost == costmap_2d::NO_INFORMATION && !allow_unknown_)
    return false;

  return static_cast<int>(cost) < obstacle_cost_threshold_;
}

double AStarGlobalPlanner::heuristic(unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1) const
{
  int dx = std::abs(static_cast<int>(x1) - static_cast<int>(x0));
  int dy = std::abs(static_cast<int>(y1) - static_cast<int>(y0));
  int diag = std::min(dx, dy);
  int straight = dx + dy - 2 * diag;
  return 14.0 * diag + 10.0 * straight;
}

bool AStarGlobalPlanner::lineOfSight(unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1) const
{
  int dx = std::abs(static_cast<int>(x1) - static_cast<int>(x0));
  int dy = std::abs(static_cast<int>(y1) - static_cast<int>(y0));
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;
  int x = static_cast<int>(x0);
  int y = static_cast<int>(y0);

  while (true)
  {
    if (!isCellTraversable(static_cast<unsigned int>(x), static_cast<unsigned int>(y)))
      return false;

    if (x == static_cast<int>(x1) && y == static_cast<int>(y1))
      break;

    int e2 = 2 * err;
    if (e2 > -dy)
    {
      err -= dy;
      x += sx;
    }
    if (e2 < dx)
    {
      err += dx;
      y += sy;
    }
  }

  return true;
}

void AStarGlobalPlanner::simplifyPlan(std::vector<geometry_msgs::PoseStamped>& plan) const
{
  if (!simplify_path_ || plan.size() < 3)
    return;

  std::vector<geometry_msgs::PoseStamped> simplified;
  simplified.push_back(plan.front());
  size_t anchor = 0;
  size_t lookahead = 2;

  while (lookahead < plan.size())
  {
    unsigned int ax, ay, lx, ly;
    if (!worldToMapSafe(plan[anchor].pose.position.x, plan[anchor].pose.position.y, ax, ay) ||
        !worldToMapSafe(plan[lookahead].pose.position.x, plan[lookahead].pose.position.y, lx, ly))
    {
      simplified.push_back(plan[lookahead - 1]);
      anchor = lookahead - 1;
      ++lookahead;
      continue;
    }

    if (!lineOfSight(ax, ay, lx, ly))
    {
      simplified.push_back(plan[lookahead - 1]);
      anchor = lookahead - 1;
    }
    ++lookahead;
  }

  simplified.push_back(plan.back());
  plan.swap(simplified);
}

bool AStarGlobalPlanner::findNearestTraversableCell(unsigned int start_mx, unsigned int start_my,
                                                    unsigned int& out_mx, unsigned int& out_my) const
{
  if (isCellTraversable(start_mx, start_my))
  {
    out_mx = start_mx;
    out_my = start_my;
    return true;
  }

  int max_r = std::max(1, goal_search_radius_cells_);
  for (int r = 1; r <= max_r; ++r)
  {
    for (int dy = -r; dy <= r; ++dy)
    {
      for (int dx = -r; dx <= r; ++dx)
      {
        int nx = static_cast<int>(start_mx) + dx;
        int ny = static_cast<int>(start_my) + dy;

        if (nx < 0 || ny < 0)
          continue;

        unsigned int umx = static_cast<unsigned int>(nx);
        unsigned int umy = static_cast<unsigned int>(ny);

        if (umx >= costmap_->getSizeInCellsX() || umy >= costmap_->getSizeInCellsY())
          continue;

        if (isCellTraversable(umx, umy))
        {
          out_mx = umx;
          out_my = umy;
          return true;
        }
      }
    }
  }

  return false;
}

void AStarGlobalPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& plan) const
{
  nav_msgs::Path path;
  path.header.stamp = ros::Time::now();
  path.header.frame_id = global_frame_;
  path.poses = plan;
  plan_pub_.publish(path);
}

bool AStarGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                  const geometry_msgs::PoseStamped& goal,
                                  std::vector<geometry_msgs::PoseStamped>& plan)
{
  plan.clear();

  if (!initialized_)
  {
    ROS_ERROR("AStarGlobalPlanner is not initialized.");
    return false;
  }

  if (start.header.frame_id != global_frame_ || goal.header.frame_id != global_frame_)
  {
    ROS_ERROR("AStarGlobalPlanner: start/goal must be in frame %s", global_frame_.c_str());
    return false;
  }

  unsigned int start_mx, start_my, goal_mx, goal_my;
  if (!worldToMapSafe(start.pose.position.x, start.pose.position.y, start_mx, start_my))
  {
    ROS_WARN("Start pose is outside costmap.");
    return false;
  }

  if (!worldToMapSafe(goal.pose.position.x, goal.pose.position.y, goal_mx, goal_my))
  {
    ROS_WARN("Goal pose is outside costmap.");
    return false;
  }

  unsigned int valid_goal_mx, valid_goal_my;
  if (!findNearestTraversableCell(goal_mx, goal_my, valid_goal_mx, valid_goal_my))
  {
    ROS_WARN("No traversable goal cell found near requested goal.");
    return false;
  }

  if (!isCellTraversable(start_mx, start_my))
  {
    ROS_WARN("Start cell is not traversable.");
    return false;
  }

  const unsigned int size_x = costmap_->getSizeInCellsX();
  const unsigned int size_y = costmap_->getSizeInCellsY();
  const unsigned int total = size_x * size_y;
  const unsigned int start_idx = toIndex(start_mx, start_my);
  const unsigned int goal_idx = toIndex(valid_goal_mx, valid_goal_my);

  std::vector<double> g_score(total, std::numeric_limits<double>::infinity());
  std::vector<int> parent(total, -1);
  std::vector<bool> closed(total, false);
  std::priority_queue<OpenNode> open;

  g_score[start_idx] = 0.0;
  open.push({start_idx, heuristic(start_mx, start_my, valid_goal_mx, valid_goal_my)});

  const int dx[8] = { 1, -1,  0,  0,  1,  1, -1, -1 };
  const int dy[8] = { 0,  0,  1, -1,  1, -1,  1, -1 };
  const double move_cost[8] = { 10, 10, 10, 10, 14, 14, 14, 14 };

  bool found = false;

  while (!open.empty())
  {
    OpenNode current = open.top();
    open.pop();

    unsigned int cx = current.idx % size_x;
    unsigned int cy = current.idx / size_x;

    if (closed[current.idx])
      continue;
    closed[current.idx] = true;

    if (current.idx == goal_idx)
    {
      found = true;
      break;
    }

    for (int k = 0; k < 8; ++k)
    {
      int nx_i = static_cast<int>(cx) + dx[k];
      int ny_i = static_cast<int>(cy) + dy[k];

      if (nx_i < 0 || ny_i < 0)
        continue;

      unsigned int nx = static_cast<unsigned int>(nx_i);
      unsigned int ny = static_cast<unsigned int>(ny_i);

      if (nx >= size_x || ny >= size_y)
        continue;

      if (!isCellTraversable(nx, ny))
        continue;

      // prevent corner cutting
      if (dx[k] != 0 && dy[k] != 0)
      {
        unsigned int sx = static_cast<unsigned int>(static_cast<int>(cx) + dx[k]);
        unsigned int sy = cy;
        unsigned int tx = cx;
        unsigned int ty = static_cast<unsigned int>(static_cast<int>(cy) + dy[k]);

        if (!isCellTraversable(sx, sy) || !isCellTraversable(tx, ty))
          continue;
      }

      unsigned int nidx = toIndex(nx, ny);
      if (closed[nidx])
        continue;

      unsigned char raw_cost = costmap_->getCost(nx, ny);
      double penalty = neutral_cost_scale_ * static_cast<double>(raw_cost);
      double tentative_g = g_score[current.idx] + move_cost[k] + penalty;

      if (tentative_g < g_score[nidx])
      {
        g_score[nidx] = tentative_g;
        parent[nidx] = static_cast<int>(current.idx);
        double f = tentative_g + heuristic(nx, ny, valid_goal_mx, valid_goal_my);
        open.push({nidx, f});
      }
    }
  }

  if (!found)
  {
    ROS_WARN("AStarGlobalPlanner: no path found.");
    return false;
  }

  std::vector<unsigned int> path_indices;
  int cur = static_cast<int>(goal_idx);
  while (cur >= 0)
  {
    path_indices.push_back(static_cast<unsigned int>(cur));
    if (static_cast<unsigned int>(cur) == start_idx)
      break;
    cur = parent[static_cast<unsigned int>(cur)];
  }

  if (path_indices.empty() || path_indices.back() != start_idx)
  {
    ROS_WARN("AStarGlobalPlanner: failed to reconstruct path.");
    return false;
  }

  std::reverse(path_indices.begin(), path_indices.end());

  for (size_t i = 0; i < path_indices.size(); ++i)
  {
    unsigned int mx = path_indices[i] % size_x;
    unsigned int my = path_indices[i] / size_x;

    double wx, wy;
    mapToWorldCenter(mx, my, wx, wy);
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = global_frame_;
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;
    plan.push_back(pose);
  }

  if (plan.size() >= 2)
  {
    simplifyPlan(plan);

    for (size_t i = 0; i + 1 < plan.size(); ++i)
    {
      double dxw = plan[i + 1].pose.position.x - plan[i].pose.position.x;
      double dyw = plan[i + 1].pose.position.y - plan[i].pose.position.y;
      double yaw = std::atan2(dyw, dxw);

      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, yaw);
      plan[i].pose.orientation = tf2::toMsg(q);
    }
    plan.back().pose.orientation = goal.pose.orientation;
  }

  publishPlan(plan);
  return true;
}
}
PLUGINLIB_EXPORT_CLASS(ris_navigation::AStarGlobalPlanner, nav_core::BaseGlobalPlanner)