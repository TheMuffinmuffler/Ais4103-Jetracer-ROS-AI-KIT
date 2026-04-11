#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_goal_sender");
    ros::NodeHandle nh("~");

    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double delay_sec = 5.0;
    int repeat_count = 5;
    std::string frame_id = "map";

    nh.param("x", x, 0.0);
    nh.param("y", y, 0.0);
    nh.param("yaw", yaw, 0.0);
    nh.param("delay_sec", delay_sec, 5.0);
    nh.param("repeat_count", repeat_count, 5);
    nh.param("frame_id", frame_id, std::string("map"));

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

    if (!ac.waitForServer(ros::Duration(30.0)))
    {
        ROS_ERROR("move_base action server not available.");
        return 1;
    }

    ros::Duration(delay_sec).sleep();

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = frame_id;
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    q.normalize();
    goal.target_pose.pose.orientation = tf2::toMsg(q);

    for (int i = 0; ros::ok() && i < repeat_count; ++i)
    {
        goal.target_pose.header.stamp = ros::Time::now();
        ac.sendGoal(goal);
        ROS_INFO("Navigation goal sent.");
        ros::Duration(0.2).sleep();
    }

    return 0;
}