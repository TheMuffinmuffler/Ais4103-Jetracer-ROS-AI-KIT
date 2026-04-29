#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "initial_pose_sender");
    ros::NodeHandle nh("~");

    double x, y, yaw, delay_sec;
    nh.param("x", x, 0.0);
    nh.param("y", y, 0.0);
    nh.param("yaw", yaw, 0.0);
    nh.param("delay_sec", delay_sec, 2.0);

    ros::Publisher pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, true);
    ros::Duration(delay_sec).sleep();

    geometry_msgs::PoseWithCovarianceStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    msg.pose.pose.position.z = 0.0;
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.pose.pose.orientation.w = q.w();

    for (double& c : msg.pose.covariance)
        c = 0.0;
    msg.pose.covariance[0] = 0.25;
    msg.pose.covariance[7] = 0.25;
    msg.pose.covariance[35] = 0.10;
    pub.publish(msg);
    ROS_INFO("Initial pose sent: x=%.3f y=%.3f yaw=%.3f", x, y, yaw);
    ros::Duration(0.5).sleep();

    return 0;
}