#pragma once

#include <ros/ros.h>
#include <nav_core/base_global_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

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
        bool initialized_;
        costmap_2d::Costmap2DROS* costmap_ros_;
        costmap_2d::Costmap2D* costmap_;
        ros::Publisher plan_pub_;

        std::string global_frame_;

        bool allow_unknown_;
        bool simplify_path_;
        int obstacle_cost_threshold_;
        double default_tolerance_;
        double neutral_cost_scale_;
        int goal_search_radius_cells_;

        inline unsigned int toIndex(unsigned int mx, unsigned int my) const
        {
            return my * costmap_->getSizeInCellsX() + mx;
        }

        bool worldToMapSafe(double wx, double wy, unsigned int& mx, unsigned int& my) const;
        void mapToWorldCenter(unsigned int mx, unsigned int my, double& wx, double& wy) const;
        bool isCellTraversable(unsigned int mx, unsigned int my) const;
        bool lineOfSight(unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1) const;
        void simplifyPlan(std::vector<geometry_msgs::PoseStamped>& plan) const;
        bool findNearestTraversableCell(unsigned int start_mx, unsigned int start_my,
                                        unsigned int& out_mx, unsigned int& out_my) const;

        double heuristic(unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1) const;
        void publishPlan(const std::vector<geometry_msgs::PoseStamped>& plan) const;
    };
}
