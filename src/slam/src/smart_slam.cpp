#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <cstdlib>

double last_ratio = 0.0;
ros::Time last_change_time;
bool map_saved = false;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    int total = msg->data.size();
    int known = 0;

    for (int i = 0; i < total; i++)
    {
        if (msg->data[i] != -1)
            known++;
    }

    double ratio = (double)known / (double)total;

    ROS_INFO("Map known ratio: %.3f", ratio);

    // If ratio changed significantly
    if (fabs(ratio - last_ratio) > 0.01)
    {
        last_change_time = ros::Time::now();
        last_ratio = ratio;
    }

    // If >90% known and stable for 5 seconds
    if (ratio > 0.90 &&
        (ros::Time::now() - last_change_time).toSec() > 5.0 &&
        !map_saved)
    {
        ROS_INFO("Map stabilized. Saving map...");

        system("mkdir -p ~/maps");
        system("rosrun map_server map_saver -f ~/maps/final_map");

        map_saved = true;
        ros::shutdown();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "smart_slam_node");
    ros::NodeHandle nh;

    last_change_time = ros::Time::now();

    ros::Subscriber sub = nh.subscribe("/map", 1, mapCallback);

    ROS_INFO("Smart SLAM monitor started...");
    ros::spin();

    return 0;
}