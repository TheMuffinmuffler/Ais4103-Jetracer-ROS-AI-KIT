#include <ros/ros.h>
#include <std_msgs/Empty.h>

#include <cstdlib>
#include <sstream>
#include <string>

class SlamRemoteSaver
{
public:
    SlamRemoteSaver()
        : nh_(), pnh_("~")
    {
        pnh_.param("save_trigger_topic", save_trigger_topic_, std::string("/slam/save_request"));
        pnh_.param("robot_map_dir", robot_map_dir_,
                   std::string("/home/jetson/catkin_ws/src/src/maps"));
        pnh_.param("robot_map_name", robot_map_name_, std::string("mymap"));

        std::stringstream mkdir_cmd;
        mkdir_cmd << "mkdir -p " << robot_map_dir_;
        std::system(mkdir_cmd.str().c_str());

        sub_ = nh_.subscribe(save_trigger_topic_, 1, &SlamRemoteSaver::callback, this);

        ROS_INFO_STREAM("slam_remote_saver ready. Saving to: "
                        << robot_map_dir_ << "/" << robot_map_name_);
    }

private:
    void callback(const std_msgs::Empty::ConstPtr&)
    {
        std::stringstream ss;
        ss << "mkdir -p " << robot_map_dir_
           << " && rosrun map_server map_saver -f "
           << robot_map_dir_ << "/" << robot_map_name_;

        ROS_INFO_STREAM("Saving robot map with command: " << ss.str());
        int ret = std::system(ss.str().c_str());
        if (ret != 0)
        {
            ROS_WARN_STREAM("Robot map save command returned non-zero code: " << ret);
        }
        else
        {
            ROS_INFO("Robot map saved successfully.");
        }
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber sub_;

    std::string save_trigger_topic_;
    std::string robot_map_dir_;
    std::string robot_map_name_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "slam_remote_saver");
    SlamRemoteSaver saver;
    ros::spin();
    return 0;
}