#include "ris/navigation/pose_utils.h"
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace ris_navigation
{
    geometry_msgs::PoseStamped makePoseStamped(
        const std::string& frame_id,
        double x,
        double y,
        double yaw)
    {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = frame_id;
        pose.header.stamp = ros::Time::now();
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation = tf2::toMsg(quaternionFromYaw(yaw));
        return pose;
    }

    double yawFromPose(const geometry_msgs::PoseStamped& pose)
    {
        tf2::Quaternion q;
        tf2::fromMsg(pose.pose.orientation, q);
        return tf2::getYaw(q);
    }

    tf2::Quaternion quaternionFromYaw(double yaw)
    {
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        q.normalize();
        return q;
    }
}