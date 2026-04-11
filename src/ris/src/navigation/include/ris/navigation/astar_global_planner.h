#pragma once

#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_core/base_global_planner.h>
#include <ros/ros.h>

namespace ris_navigation
{
    class AStarGlobalPlanner : public nav_core::BaseGlobalPlanner
    {
    public:
        AStarGlobalPlanner();
        AStarGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;

        bool makePlan(
            const geometry_msgs::PoseStamped& start,
            const geometry_msgs::PoseStamped& goal,
            std::vector<geometry_msgs::PoseStamped>& plan) override;

    private:
        bool initialized_;
        costmap_2d::Costmap2DROS* costmap_ros_;
        costmap_2d::Costmap2D* costmap_;

        bool allow_unknown_;
        bool use_eight_connected_;
        bool simplify_path_;
        double goal_tolerance_;
        double max_goal_search_radius_;
        int lethal_cost_threshold_;
        int neutral_cost_threshold_;
    };
}