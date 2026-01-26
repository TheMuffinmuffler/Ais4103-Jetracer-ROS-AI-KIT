#include <ros/ros.h>
#include <ros/console.h>

#include <ris_msgs/Hello.h>

std::atomic<bool> run = true;
boost::shared_ptr<ros::NodeHandle> node = nullptr;

void hello_message_received(boost::shared_ptr<ris_msgs::Hello> msg)
{
    ROS_INFO("/hello_subscriber received msg: %s", msg->text.c_str());
}

int main(int argc, char **args)
{
    ros::init(argc, args, "hello_subscriber");
    ros::start();
    node = boost::make_shared<ros::NodeHandle>("hello_subscriber");
    ROS_INFO("/hello_subscriber node started");

    ros::Subscriber subscriber = node->subscribe("/hello_publisher/message", 1u, &hello_message_received);
    ros::spin();
    ros::shutdown();
    return 0;
}
