#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Twist.h>

#include <cstdlib>
#include <sstream>
#include <string>

class YoussefSlam
{
public:
    YoussefSlam()
        : nh_(), pnh_("~"),
          has_map_(false),
          have_previous_map_(false),
          save_started_(false),
          finished_(false),
          stable_time_acc_(0.0)
    {
        pnh_.param("map_topic", map_topic_, std::string("/map"));
        pnh_.param("laptop_map_dir", laptop_map_dir_, std::string("/home/jetson/laptop_slam_maps"));
        pnh_.param("laptop_map_name", laptop_map_name_, std::string("mymap"));
        pnh_.param("robot_map_dir", robot_map_dir_,
                   std::string("/home/jetson/catkin_ws/src/maps"));
        pnh_.param("robot_map_name", robot_map_name_, std::string("mymap"));
        pnh_.param("check_period_sec", check_period_sec_, 2.0);
        pnh_.param("stable_seconds_before_save", stable_seconds_before_save_, 8.0);
        pnh_.param("changed_cells_threshold", changed_cells_threshold_, 20);
        pnh_.param("gmapping_node_name", gmapping_node_name_, std::string("/slam_gmapping"));
        pnh_.param("stop_robot_before_save", stop_robot_before_save_, true);

        map_sub_ = nh_.subscribe(map_topic_, 1, &YoussefSlam::mapCallback, this);
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);

        timer_ = nh_.createTimer(ros::Duration(check_period_sec_), &YoussefSlam::timerCallback, this);

        std::stringstream mkdir_robot_cmd;
        mkdir_robot_cmd << "mkdir -p " << robot_map_dir_;
        std::system(mkdir_robot_cmd.str().c_str());

        std::stringstream mkdir_laptop_cmd;
        mkdir_laptop_cmd << "mkdir -p " << laptop_map_dir_;
        std::system(mkdir_laptop_cmd.str().c_str());

        ROS_INFO("youssef_slam started.");
        ROS_INFO_STREAM("Robot maps will be saved to: " << robot_map_dir_);
        ROS_INFO_STREAM("Laptop maps will be saved to: " << laptop_map_dir_);
    }

private:
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
    {
        latest_map_ = *msg;
        has_map_ = true;
    }

    int countChangedCells(const nav_msgs::OccupancyGrid& a, const nav_msgs::OccupancyGrid& b)
    {
        if (a.data.size() != b.data.size())
            return 1000000;

        int changed = 0;
        for (size_t i = 0; i < a.data.size(); ++i)
        {
            if (a.data[i] != b.data[i])
                ++changed;
        }
        return changed;
    }

    void publishStop()
    {
        geometry_msgs::Twist zero;
        for (int i = 0; i < 10; ++i)
        {
            cmd_pub_.publish(zero);
            ros::Duration(0.05).sleep();
        }
    }

    void saveRobotMap()
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

    void saveLaptopMap()
    {
        std::stringstream ss;
        ss << "mkdir -p " << laptop_map_dir_
           << " && rosrun map_server map_saver -f "
           << laptop_map_dir_ << "/" << laptop_map_name_;

        ROS_INFO_STREAM("Saving laptop map with command: " << ss.str());
        int ret = std::system(ss.str().c_str());
        if (ret != 0)
        {
            ROS_WARN_STREAM("Laptop map save command returned non-zero code: " << ret);
        }
        else
        {
            ROS_INFO("Laptop map saved successfully.");
        }
    }

    void stopGmapping()
    {
        std::stringstream ss;
        ss << "rosnode kill " << gmapping_node_name_;
        ROS_INFO_STREAM("Stopping gmapping with command: " << ss.str());
        std::system(ss.str().c_str());
    }

    void finishSequence()
    {
        if (save_started_ || finished_)
            return;

        save_started_ = true;

        ROS_INFO("Map appears stable. Starting automatic save sequence...");

        if (stop_robot_before_save_)
            publishStop();

        saveRobotMap();
        saveLaptopMap();
        stopGmapping();

        finished_ = true;
        ROS_INFO("Automatic SLAM save/stop sequence finished.");
        ros::shutdown();
    }

    void timerCallback(const ros::TimerEvent&)
    {
        if (!has_map_ || finished_)
            return;

        if (!have_previous_map_)
        {
            previous_map_ = latest_map_;
            have_previous_map_ = true;
            ROS_INFO("Received first map. Waiting for next check...");
            return;
        }

        int changed = countChangedCells(previous_map_, latest_map_);
        ROS_INFO_STREAM("Map stability check: changed_cells=" << changed);

        if (changed <= changed_cells_threshold_)
        {
            stable_time_acc_ += check_period_sec_;
            ROS_INFO_STREAM("Map stable for " << stable_time_acc_
                            << " / " << stable_seconds_before_save_ << " sec");
        }
        else
        {
            stable_time_acc_ = 0.0;
            ROS_INFO("Map still changing. Resetting stable timer.");
        }

        previous_map_ = latest_map_;

        if (stable_time_acc_ >= stable_seconds_before_save_)
        {
            finishSequence();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber map_sub_;
    ros::Publisher cmd_pub_;
    ros::Timer timer_;

    nav_msgs::OccupancyGrid latest_map_;
    nav_msgs::OccupancyGrid previous_map_;

    bool has_map_;
    bool have_previous_map_;
    bool save_started_;
    bool finished_;

    double stable_time_acc_;
    double check_period_sec_;
    double stable_seconds_before_save_;
    int changed_cells_threshold_;
    bool stop_robot_before_save_;

    std::string map_topic_;
    std::string laptop_map_dir_;
    std::string laptop_map_name_;
    std::string robot_map_dir_;
    std::string robot_map_name_;
    std::string gmapping_node_name_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "youssef_slam");
    YoussefSlam node;
    ros::spin();
    return 0;
}