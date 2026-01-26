#include <ros/ros.h>
#include <ros/console.h>

#include <ris_msgs/Hello.h>

#include <thread>

std::atomic<bool> run = true;
boost::shared_ptr<ros::NodeHandle> node = nullptr;

void publish_hello()
{
    ros::Publisher publisher = node->advertise<ris_msgs::Hello>("message", 1u);
    uint32_t index = 0u;
    while(run)
    {
        ris_msgs::Hello hello;
        hello.text = "Hello nr: " + std::to_string(index++);
        ROS_INFO("/hello_publisher/message: publishing %s", hello.text.c_str());
        publisher.publish(hello);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

int main(int argc, char **args)
{
    ros::init(argc, args, "hello_publisher");
    ros::start();
    node = boost::make_shared<ros::NodeHandle>("hello_publisher");
    ROS_INFO("/hello_publisher node started");

    std::thread worker([&]() { publish_hello(); });

    ros::spin();
    run = false;
    worker.join();
    ros::shutdown();
    return 0;
}
