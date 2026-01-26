#include <ros/ros.h>
#include <ros/console.h>

#include <ris_msgs/PingPong.h>

boost::shared_ptr<ros::NodeHandle> node = nullptr;

bool serve_pingpong(ris_msgs::PingPong::Request &request, ris_msgs::PingPong::Response &response)
{
    response.output = "Pong: " + request.input;
    ROS_INFO("/hello_server received: \"%s\"", request.input.c_str());
    return true;
    // Can also return false if something went wrong.
    // The value of the returned boolean is visible for the caller (in this case, in the hello_client-example)
}

int main(int argc, char **args)
{
    ros::init(argc, args, "hello_server");
    ros::start();
    node = boost::make_shared<ros::NodeHandle>("hello_server");
    ROS_INFO("/hello_server node started");

    ros::ServiceServer server = node->advertiseService("ping", &serve_pingpong);

    ros::spin();
    ros::shutdown();
    return 0;
}
