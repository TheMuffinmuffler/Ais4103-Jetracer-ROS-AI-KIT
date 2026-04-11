#pragma once

#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>

namespace ris_navigation
{
    bool worldToMap(const costmap_2d::Costmap2D& costmap, double wx, double wy, unsigned int& mx, unsigned int& my);
    void mapToWorld(const costmap_2d::Costmap2D& costmap, unsigned int mx, unsigned int my, double& wx, double& wy);

    bool isCellTraversable(
        const costmap_2d::Costmap2D& costmap,
        unsigned int mx,
        unsigned int my,
        unsigned char lethal_threshold,
        bool allow_unknown);

    bool findNearestValidCell(
        const costmap_2d::Costmap2D& costmap,
        unsigned int start_mx,
        unsigned int start_my,
        double max_radius_m,
        unsigned char lethal_threshold,
        bool allow_unknown,
        unsigned int& result_mx,
        unsigned int& result_my);
}