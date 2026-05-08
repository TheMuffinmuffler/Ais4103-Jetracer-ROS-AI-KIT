
/*

* I wrote this node as a small ROS test utility for sending one navigation goal to move_base. The goal position (x, y), yaw angle, frame_id, delay, and repeat count are loaded from ROS parameters.
 *
 * My contribution:
 * I used this node to test whether move_base, AMCL/localization, the global
 * planner, and the local planner could accept and execute a navigation goal.
 *
 * External libraries:
 * This file uses standard ROS actionlib and move_base_msgs interfaces.
 *
 * AI assistance declaration:
 * ChatGPT helped with this file, it was used only as programming assistance
 * for the ROS action-client structure and quaternion conversion. The node was
 * adapted and tested by me for the JetRacer navigation setup.
 *
 * Logic:
 *   1. Read goal parameters.
 *   2. Wait for the /move_base action server.
 *   3. Convert yaw to quaternion.
 *   4. Send the goal to move_base.
 */
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
        goal.target_pose.pose.position.x = x; // Modern Robotics, Ch. 3, Eq. (3.1), p. 61
        goal.target_pose.pose.position.y = y; // Modern Robotics, Ch. 3, Eq. (3.1), p. 61
        goal.target_pose.pose.position.z = 0.0; // Modern Robotics, Ch. 3, Eq. (3.1), p. 61
        tf::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        goal.target_pose.pose.orientation.x = q.x(); // Modern Robotics, Ch. 3, Eq. (3.1), p. 61
        goal.target_pose.pose.orientation.y = q.y();// Modern Robotics, Ch. 3, Eq. (3.1), p. 61
        goal.target_pose.pose.orientation.z = q.z(); // Modern Robotics, Ch. 3, Eq. (3.1), p. 61
        goal.target_pose.pose.orientation.w = q.w(); // Modern Robotics, Ch. 3, Eq. (3.1), p. 61
        ac.sendGoal(goal);
        ROS_INFO("Navigation goal sent.");
    }

    return 0;
}
