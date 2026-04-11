#include "astar_global_planner.h"

#include <pluginlib/class_list_macros.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <costmap_2d/cost_values.h>

#include <cmath>
#include <limits>
#include <algorithm>

namespace ris_navigation
{

AStarGlobalPlanner::AStarGlobalPlanner()
    : initialized_(false),
      costmap_ros_(nullptr),
      costmap_(nullptr),
      allow_unknown_(false),
      use_diagonal_(true),
      simplify_plan_(true),
      publish_debug_plan_(true),
      lethal_cost_threshold_(costmap_2d::INSCRIBED_INFLATED_OBSTACLE),
      cost_penalty_scale_(2.0),
      width_(0),
      height_(0),
      resolution_(0.0),
      origin_x_(0.0),
      origin_y_(0.0)
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
    global_frame_ = costmap_ros_->getGlobalFrameID();

    width_ = costmap_->getSizeInCellsX();
    height_ = costmap_->getSizeInCellsY();
    resolution_ = costmap_->getResolution();
    origin_x_ = costmap_->getOriginX();
    origin_y_ = costmap_->getOriginY();

    ros::NodeHandle private_nh("~/" + name);

    private_nh.param("allow_unknown", allow_unknown_, true);
    private_nh.param("use_diagonal", use_diagonal_, true);
    private_nh.param("simplify_plan", simplify_plan_, true);
    private_nh.param("publish_debug_plan", publish_debug_plan_, true);

    int lethal_cost_int = static_cast<int>(costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
    private_nh.param("lethal_cost_threshold", lethal_cost_int, lethal_cost_int);
    lethal_cost_threshold_ = static_cast<unsigned char>(lethal_cost_int);

    private_nh.param("cost_penalty_scale", cost_penalty_scale_, 0.3);

    debug_plan_pub_ = private_nh.advertise<nav_msgs::Path>("debug_plan", 1, true);

    initialized_ = true;

    ROS_INFO("AStarGlobalPlanner initialized.");
}

inline int AStarGlobalPlanner::toIndex(int mx, int my) const
{
    return my * static_cast<int>(width_) + mx;
}

inline void AStarGlobalPlanner::toGrid(int index, int& mx, int& my) const
{
    mx = index % static_cast<int>(width_);
    my = index / static_cast<int>(width_);
}

bool AStarGlobalPlanner::isInside(int mx, int my) const
{
    return mx >= 0 && my >= 0 &&
           mx < static_cast<int>(width_) &&
           my < static_cast<int>(height_);
}

bool AStarGlobalPlanner::isCellTraversable(int mx, int my, bool is_goal) const
{
    if (!isInside(mx, my))
        return false;

    const unsigned char cost = costmap_->getCost(mx, my);

    if (cost == costmap_2d::NO_INFORMATION)
        return allow_unknown_ || is_goal;

    if (cost >= lethal_cost_threshold_)
        return false;

    return true;
}

bool AStarGlobalPlanner::worldToGrid(double wx, double wy, int& mx, int& my) const
{
    unsigned int umx, umy;
    if (!costmap_->worldToMap(wx, wy, umx, umy))
        return false;

    mx = static_cast<int>(umx);
    my = static_cast<int>(umy);
    return true;
}

void AStarGlobalPlanner::gridToWorld(int mx, int my, double& wx, double& wy) const
{
    costmap_->mapToWorld(static_cast<unsigned int>(mx), static_cast<unsigned int>(my), wx, wy);
}

double AStarGlobalPlanner::heuristic(int x1, int y1, int x2, int y2) const
{
    const int dx = std::abs(x1 - x2);
    const int dy = std::abs(y1 - y2);

    if (use_diagonal_)
    {
        const int min_d = std::min(dx, dy);
        const int max_d = std::max(dx, dy);
        return 14.0 * min_d + 10.0 * (max_d - min_d);
    }

    return 10.0 * (dx + dy);
}

double AStarGlobalPlanner::moveCost(int from_x, int from_y, int to_x, int to_y) const
{
    const bool diagonal = (from_x != to_x && from_y != to_y);
    const double base = diagonal ? 14.0 : 10.0;

    const unsigned char cost = costmap_->getCost(to_x, to_y);
    const double normalized = static_cast<double>(cost) / 252.0;

    return base * (1.0 + cost_penalty_scale_ * normalized);
}

bool AStarGlobalPlanner::runAStar(int start_x, int start_y,
                                  int goal_x, int goal_y,
                                  std::vector<std::pair<int, int>>& path)
{
    path.clear();

    if (!isInside(start_x, start_y) || !isInside(goal_x, goal_y))
        return false;

    if (!isCellTraversable(goal_x, goal_y, true))
    {
        ROS_WARN("Goal cell is blocked in costmap.");
        return false;
    }

    const int total = static_cast<int>(width_ * height_);
    const int start_idx = toIndex(start_x, start_y);
    const int goal_idx = toIndex(goal_x, goal_y);

    std::vector<double> g_score(total, std::numeric_limits<double>::infinity());
    std::vector<int> parent(total, -1);
    std::vector<bool> closed(total, false);

    std::priority_queue<OpenNode> open;
    g_score[start_idx] = 0.0;
    parent[start_idx] = start_idx;
    open.push({start_idx, heuristic(start_x, start_y, goal_x, goal_y), 0.0});

    const int dx8[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    const int dy8[8] = {0, 0, 1, -1, 1, -1, 1, -1};

    const int dx4[4] = {1, -1, 0, 0};
    const int dy4[4] = {0, 0, 1, -1};

    while (!open.empty())
    {
        const OpenNode cur = open.top();
        open.pop();

        if (closed[cur.index])
            continue;

        closed[cur.index] = true;

        if (cur.index == goal_idx)
        {
            int idx = goal_idx;
            while (idx != start_idx)
            {
                int mx, my;
                toGrid(idx, mx, my);
                path.push_back({mx, my});
                idx = parent[idx];
                if (idx < 0)
                    return false;
            }

            path.push_back({start_x, start_y});
            std::reverse(path.begin(), path.end());
            return true;
        }

        int cx, cy;
        toGrid(cur.index, cx, cy);

        const int n_count = use_diagonal_ ? 8 : 4;
        const int* use_dx = use_diagonal_ ? dx8 : dx4;
        const int* use_dy = use_diagonal_ ? dy8 : dy4;

        for (int i = 0; i < n_count; ++i)
        {
            const int nx = cx + use_dx[i];
            const int ny = cy + use_dy[i];

            if (!isInside(nx, ny))
                continue;

            const bool diagonal = (use_dx[i] != 0 && use_dy[i] != 0);

            if (diagonal)
            {
                if (!isCellTraversable(cx + use_dx[i], cy) ||
                    !isCellTraversable(cx, cy + use_dy[i]))
                {
                    continue;
                }
            }

            const bool is_goal = (nx == goal_x && ny == goal_y);
            if (!isCellTraversable(nx, ny, is_goal))
                continue;

            const int nidx = toIndex(nx, ny);
            if (closed[nidx])
                continue;

            const double tentative_g = g_score[cur.index] + moveCost(cx, cy, nx, ny);

            if (tentative_g < g_score[nidx])
            {
                g_score[nidx] = tentative_g;
                parent[nidx] = cur.index;
                const double f = tentative_g + heuristic(nx, ny, goal_x, goal_y);
                open.push({nidx, f, tentative_g});
            }
        }
    }

    return false;
}

std::vector<std::pair<int, int>> AStarGlobalPlanner::simplifyPath(
    const std::vector<std::pair<int, int>>& path) const
{
    if (path.size() < 3)
        return path;

    std::vector<std::pair<int, int>> simplified;
    simplified.push_back(path.front());

    for (size_t i = 1; i + 1 < path.size(); ++i)
    {
        const int dx1 = path[i].first - path[i - 1].first;
        const int dy1 = path[i].second - path[i - 1].second;
        const int dx2 = path[i + 1].first - path[i].first;
        const int dy2 = path[i + 1].second - path[i].second;

        if (dx1 != dx2 || dy1 != dy2)
            simplified.push_back(path[i]);
    }

    simplified.push_back(path.back());
    return simplified;
}

void AStarGlobalPlanner::buildPosePlan(const std::vector<std::pair<int, int>>& grid_path,
                                       const geometry_msgs::PoseStamped& goal,
                                       std::vector<geometry_msgs::PoseStamped>& plan) const
{
    plan.clear();
    if (grid_path.empty())
        return;

    plan.reserve(grid_path.size());

    for (size_t i = 0; i < grid_path.size(); ++i)
    {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = global_frame_;
        pose.header.stamp = ros::Time::now();

        double wx, wy;
        gridToWorld(grid_path[i].first, grid_path[i].second, wx, wy);

        pose.pose.position.x = wx;
        pose.pose.position.y = wy;
        pose.pose.position.z = 0.0;

        double yaw = 0.0;
        if (i + 1 < grid_path.size())
        {
            double wx2, wy2;
            gridToWorld(grid_path[i + 1].first, grid_path[i + 1].second, wx2, wy2);
            yaw = std::atan2(wy2 - wy, wx2 - wx);
            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, yaw);
            pose.pose.orientation = tf2::toMsg(q);
        }
        else
        {
            pose.pose.orientation = goal.pose.orientation;
        }

        plan.push_back(pose);
    }

    if (!plan.empty())
    {
        plan.back().pose.position.x = goal.pose.position.x;
        plan.back().pose.position.y = goal.pose.position.y;
        plan.back().pose.orientation = goal.pose.orientation;
    }
}

void AStarGlobalPlanner::publishDebugPath(const std::vector<geometry_msgs::PoseStamped>& plan) const
{
    if (!publish_debug_plan_)
        return;

    nav_msgs::Path path_msg;
    path_msg.header.frame_id = global_frame_;
    path_msg.header.stamp = ros::Time::now();
    path_msg.poses = plan;
    debug_plan_pub_.publish(path_msg);
}

bool AStarGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                  const geometry_msgs::PoseStamped& goal,
                                  std::vector<geometry_msgs::PoseStamped>& plan)
{
    plan.clear();

    if (!initialized_)
    {
        ROS_ERROR("AStarGlobalPlanner has not been initialized.");
        return false;
    }

    if (start.header.frame_id != global_frame_)
    {
        ROS_ERROR_STREAM("Start frame must be " << global_frame_ << ", but got " << start.header.frame_id);
        return false;
    }

    if (goal.header.frame_id != global_frame_)
    {
        ROS_ERROR_STREAM("Goal frame must be " << global_frame_ << ", but got " << goal.header.frame_id);
        return false;
    }

    width_ = costmap_->getSizeInCellsX();
    height_ = costmap_->getSizeInCellsY();
    resolution_ = costmap_->getResolution();
    origin_x_ = costmap_->getOriginX();
    origin_y_ = costmap_->getOriginY();

    int start_x, start_y, goal_x, goal_y;
    if (!worldToGrid(start.pose.position.x, start.pose.position.y, start_x, start_y))
    {
        ROS_ERROR("Start is outside map bounds.");
        return false;
    }

    if (!worldToGrid(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y))
    {
        ROS_ERROR("Goal is outside map bounds.");
        return false;
    }

    std::vector<std::pair<int, int>> grid_path;
    if (!runAStar(start_x, start_y, goal_x, goal_y, grid_path))
    {
        ROS_WARN("A* planner failed to find a path.");
        return false;
    }

    if (simplify_plan_)
        grid_path = simplifyPath(grid_path);

    buildPosePlan(grid_path, goal, plan);
    publishDebugPath(plan);

    ROS_INFO("AStarGlobalPlanner produced a plan with %zu poses.", plan.size());
    return !plan.empty();
}

}

PLUGINLIB_EXPORT_CLASS(ris_navigation::AStarGlobalPlanner, nav_core::BaseGlobalPlanner)