#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/tf.h>

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
    nh.param("delay_sec", delay_sec, 2.0);
    nh.param("repeat_count", repeat_count, 1);
    nh.param("frame_id", frame_id, std::string("map"));

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("/move_base", true);

    ROS_INFO("Waiting for move_base action server...");
    ac.waitForServer();

    ros::Duration(delay_sec).sleep();

    for (int i = 0; i < repeat_count; ++i)
    {
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.header.frame_id = frame_id;
        goal.target_pose.pose.position.x = x;
        goal.target_pose.pose.position.y = y;
        goal.target_pose.pose.position.z = 0.0;
        tf::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        goal.target_pose.pose.orientation.x = q.x();
        goal.target_pose.pose.orientation.y = q.y();
        goal.target_pose.pose.orientation.z = q.z();
        goal.target_pose.pose.orientation.w = q.w();
        ac.sendGoal(goal);
        ROS_INFO("Navigation goal sent.");
    }

    return 0;
}