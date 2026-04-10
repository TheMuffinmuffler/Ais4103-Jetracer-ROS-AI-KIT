#pragma once

#include <ros/ros.h>
#include <nav_core/base_global_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <vector>
#include <string>
#include <queue>

namespace ris_navigation
{

class AStarGlobalPlanner : public nav_core::BaseGlobalPlanner
{
public:
    AStarGlobalPlanner();
    AStarGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;

    bool makePlan(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan) override;

private:
    struct OpenNode
    {
        int index;
        double f;
        double g;

        bool operator<(const OpenNode& other) const
        {
            return f > other.f;
        }
    };

    bool initialized_;
    costmap_2d::Costmap2DROS* costmap_ros_;
    costmap_2d::Costmap2D* costmap_;
    ros::Publisher debug_plan_pub_;
    std::string global_frame_;

    bool allow_unknown_;
    bool use_diagonal_;
    bool simplify_plan_;
    bool publish_debug_plan_;
    unsigned char lethal_cost_threshold_;
    double cost_penalty_scale_;

    unsigned int width_;
    unsigned int height_;
    double resolution_;
    double origin_x_;
    double origin_y_;

    inline int toIndex(int mx, int my) const;
    inline void toGrid(int index, int& mx, int& my) const;

    bool isInside(int mx, int my) const;
    bool isCellTraversable(int mx, int my, bool is_goal = false) const;
    bool worldToGrid(double wx, double wy, int& mx, int& my) const;
    void gridToWorld(int mx, int my, double& wx, double& wy) const;

    double heuristic(int x1, int y1, int x2, int y2) const;
    double moveCost(int from_x, int from_y, int to_x, int to_y) const;

    bool runAStar(int start_x, int start_y,
                  int goal_x, int goal_y,
                  std::vector<std::pair<int, int>>& path);

    std::vector<std::pair<int, int>> simplifyPath(
        const std::vector<std::pair<int, int>>& path) const;

    void buildPosePlan(const std::vector<std::pair<int, int>>& grid_path,
                       const geometry_msgs::PoseStamped& goal,
                       std::vector<geometry_msgs::PoseStamped>& plan) const;

    void publishDebugPath(const std::vector<geometry_msgs::PoseStamped>& plan) const;
};

}