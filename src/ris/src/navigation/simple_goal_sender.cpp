#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_goal_sender");
    ros::NodeHandle nh("~");

    double x, y, yaw, delay_sec;
    int repeat_count;
    std::string frame_id;

    nh.param("x", x, 0.0);
    nh.param("y", y, 0.0);
    nh.param("yaw", yaw, 0.0);
    nh.param("delay_sec", delay_sec, 5.0);
    nh.param("repeat_count", repeat_count, 10);
    nh.param("frame_id", frame_id, std::string("map"));

    ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, true);

    ros::Duration(delay_sec).sleep();

    geometry_msgs::PoseStamped msg;
    msg.header.frame_id = frame_id;
    msg.pose.position.x = x;
    msg.pose.position.y = y;
    msg.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    msg.pose.orientation = tf2::toMsg(q);

    ros::Rate rate(2.0);
    for (int i = 0; i < repeat_count && ros::ok(); ++i)
    {
        msg.header.stamp = ros::Time::now();
        pub.publish(msg);
        rate.sleep();
    }

    ROS_INFO("Navigation goal sent.");
    return 0;
}