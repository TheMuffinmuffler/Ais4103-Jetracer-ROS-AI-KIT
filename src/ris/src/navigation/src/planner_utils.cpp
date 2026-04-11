#include "ris/navigation/planner_utils.h"

#include <cmath>

namespace ris_navigation
{
    std::vector<geometry_msgs::PoseStamped> simplifyPathByDirection(
        const std::vector<geometry_msgs::PoseStamped>& path)
    {
        if (path.size() < 3)
            return path;

        std::vector<geometry_msgs::PoseStamped> simplified;
        simplified.push_back(path.front());

        auto sign = [](double v) -> int {
            if (v > 1e-6)
                return 1;
            if (v < -1e-6)
                return -1;
            return 0;
        };

        for (size_t i = 1; i + 1 < path.size(); ++i)
        {
            const double dx1 = path[i].pose.position.x - path[i - 1].pose.position.x;
            const double dy1 = path[i].pose.position.y - path[i - 1].pose.position.y;
            const double dx2 = path[i + 1].pose.position.x - path[i].pose.position.x;
            const double dy2 = path[i + 1].pose.position.y - path[i].pose.position.y;

            if (sign(dx1) != sign(dx2) || sign(dy1) != sign(dy2))
                simplified.push_back(path[i]);
        }

        simplified.push_back(path.back());
        return simplified;
    }
}