#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "initial_pose_sender");
    ros::NodeHandle nh("~");

    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double delay_sec = 15.0;
    int repeat_count = 30;
    std::string frame_id = "map";

    nh.param("x", x, 0.0);
    nh.param("y", y, 0.0);
    nh.param("yaw", yaw, 0.0);
    nh.param("delay_sec", delay_sec, 15.0);
    nh.param("repeat_count", repeat_count, 30);
    nh.param("frame_id", frame_id, std::string("map"));

    ros::Publisher pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, true);

    ros::Duration(delay_sec).sleep();

    geometry_msgs::PoseWithCovarianceStamped msg;
    msg.header.frame_id = frame_id;

    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    q.normalize();
    msg.pose.pose.orientation = tf2::toMsg(q);

    for (double& c : msg.pose.covariance)
        c = 0.0;

    msg.pose.covariance[0] = 0.25;
    msg.pose.covariance[7] = 0.25;
    msg.pose.covariance[35] = 0.06853891945200942;

    ros::Rate rate(5.0);
    for (int i = 0; ros::ok() && i < repeat_count; ++i)
    {
        msg.header.stamp = ros::Time::now();
        pub.publish(msg);
        ROS_INFO("Setting pose (%.6f): %.3f %.3f %.3f",
                 msg.header.stamp.toSec(),
                 x, y, yaw);
        rate.sleep();
    }

    ROS_INFO("Initial pose sent.");
    return 0;
}