#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <vector>

namespace ris_navigation
{
    std::vector<geometry_msgs::PoseStamped> simplifyPathByDirection(
        const std::vector<geometry_msgs::PoseStamped>& path);
}