#include <ros/ros.h>
#include <ros/console.h>

boost::shared_ptr<ros::NodeHandle> node = nullptr;

int main(int argc, char **args)
{
    ros::init(argc, args, "hello_world");
    ros::start();
    node = boost::make_shared<ros::NodeHandle>("hello_world");
    ROS_INFO("/hello_world node started");

    // Do what you need to do to set up the application here.

    ros::spin();
    return 0;
}
