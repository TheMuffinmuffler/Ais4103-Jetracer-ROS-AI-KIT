#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>

namespace ris_navigation
{
    geometry_msgs::PoseStamped makePoseStamped(
        const std::string& frame_id,
        double x,
        double y,
        double yaw);

    double yawFromPose(const geometry_msgs::PoseStamped& pose);
    tf2::Quaternion quaternionFromYaw(double yaw);
}