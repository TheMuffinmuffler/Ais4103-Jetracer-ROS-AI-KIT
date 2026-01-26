#include <ros/ros.h>


#include <ris_msgs/PingPong.h>

#include <thread>

std::atomic<bool> run = true;
boost::shared_ptr<ros::NodeHandle> node = nullptr;

void call_pingpong()
{
    using namespace std::chrono_literals;

    ros::ServiceClient client = node->serviceClient<ris_msgs::PingPong>("/hello_server/ping");
    uint32_t index = 0u;
    while(run)
    {
        ris_msgs::PingPong exchange;
        exchange.request.input = "Ping index " + std::to_string(index++);
        if(client.call(exchange.request, exchange.response)) // .call() returns true or false, as described in the hello_server-example.
            ROS_INFO("/hello_client received: \"%s\"", exchange.response.output.c_str());
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

int main(int argc, char **args)
{
    ros::init(argc, args, "hello_client");
    ros::start();
    node = boost::make_shared<ros::NodeHandle>("hello_client");
    ROS_INFO("/hello_client node started");

    std::thread worker([&]() { call_pingpong(); });

    ros::spin();
    run = false;
    worker.join();
    ros::shutdown();
    return 0;
}
