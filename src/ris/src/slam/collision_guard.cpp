#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

#include <cmath>
#include <limits>
#include <string>

class CollisionGuard
{
public:
    CollisionGuard()
        : nh_(), pnh_("~"),
          has_scan_(false),
          block_until_(0.0),
          watchdog_active_(false)
    {
        pnh_.param("scan_topic", scan_topic_, std::string("/scan"));
        pnh_.param("input_cmd_topic", input_cmd_topic_, std::string("/cmd_vel_raw"));
        pnh_.param("output_cmd_topic", output_cmd_topic_, std::string("/cmd_vel"));

        pnh_.param("front_angle_deg", front_angle_deg_, 18.0);
        pnh_.param("stop_distance", stop_distance_, 0.28);
        pnh_.param("progress_check_distance", progress_check_distance_, 1.20);
        pnh_.param("progress_timeout_sec", progress_timeout_sec_, 1.5);
        pnh_.param("progress_epsilon", progress_epsilon_, 0.03);
        pnh_.param("cooldown_sec", cooldown_sec_, 1.0);
        pnh_.param("min_forward_cmd", min_forward_cmd_, 0.03);
        pnh_.param("max_turn_for_progress_check", max_turn_for_progress_check_, 0.25);
        pnh_.param("allow_turning_when_blocked", allow_turning_when_blocked_, true);

        scan_sub_ = nh_.subscribe(scan_topic_, 1, &CollisionGuard::scanCallback, this);
        cmd_sub_ = nh_.subscribe(input_cmd_topic_, 10, &CollisionGuard::cmdCallback, this);
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(output_cmd_topic_, 10);

        ROS_INFO("collision_guard started.");
        ROS_INFO_STREAM("scan_topic: " << scan_topic_);
        ROS_INFO_STREAM("input_cmd_topic: " << input_cmd_topic_);
        ROS_INFO_STREAM("output_cmd_topic: " << output_cmd_topic_);
        ROS_INFO_STREAM("stop_distance: " << stop_distance_);
    }

private:
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        latest_scan_ = *msg;
        has_scan_ = true;
    }

    double getFrontMinRange() const
    {
        if (!has_scan_ || latest_scan_.ranges.empty())
            return std::numeric_limits<double>::infinity();

        const double front_angle_rad = front_angle_deg_ * M_PI / 180.0;
        double min_range = std::numeric_limits<double>::infinity();

        for (size_t i = 0; i < latest_scan_.ranges.size(); ++i)
        {
            double angle = latest_scan_.angle_min + static_cast<double>(i) * latest_scan_.angle_increment;

            if (std::fabs(angle) > front_angle_rad)
                continue;

            double r = latest_scan_.ranges[i];

            if (std::isnan(r) || std::isinf(r))
                continue;

            if (r < latest_scan_.range_min || r > latest_scan_.range_max)
                continue;

            if (r < min_range)
                min_range = r;
        }

        return min_range;
    }

    void resetWatchdog()
    {
        watchdog_active_ = false;
    }

    void startWatchdog(double current_front_range)
    {
        watchdog_active_ = true;
        watchdog_start_time_ = ros::Time::now();
        watchdog_reference_range_ = current_front_range;
    }

    bool shouldHardStop(double front_range, const geometry_msgs::Twist& cmd) const
    {
        return (cmd.linear.x > min_forward_cmd_) && (front_range < stop_distance_);
    }

    bool shouldTriggerProgressStop(double front_range, const geometry_msgs::Twist& cmd)
    {
        if (cmd.linear.x <= min_forward_cmd_ || std::fabs(cmd.angular.z) > max_turn_for_progress_check_)
        {
            resetWatchdog();
            return false;
        }

        if (!std::isfinite(front_range) || front_range > progress_check_distance_)
        {
            resetWatchdog();
            return false;
        }

        if (!watchdog_active_)
        {
            startWatchdog(front_range);
            return false;
        }

        double elapsed = (ros::Time::now() - watchdog_start_time_).toSec();
        double progress = std::fabs(front_range - watchdog_reference_range_);

        if (elapsed >= progress_timeout_sec_ && progress < progress_epsilon_)
        {
            ROS_WARN_STREAM("collision_guard: forward command but little/no laser progress detected. "
                            << "front_range=" << front_range << ", progress=" << progress
                            << ", elapsed=" << elapsed);
            resetWatchdog();
            return true;
        }

        return false;
    }

    geometry_msgs::Twist makeSafeCmd(const geometry_msgs::Twist& raw_cmd)
    {
        geometry_msgs::Twist safe_cmd = raw_cmd;
        double front_range = getFrontMinRange();
        ros::Time now = ros::Time::now();

        if (now.toSec() < block_until_)
        {
            if (safe_cmd.linear.x > 0.0)
                safe_cmd.linear.x = 0.0;

            if (!allow_turning_when_blocked_)
                safe_cmd.angular.z = 0.0;

            return safe_cmd;
        }

        if (shouldHardStop(front_range, raw_cmd))
        {
            ROS_WARN_STREAM("collision_guard: obstacle too close in front. front_range=" << front_range
                            << " < stop_distance=" << stop_distance_);

            safe_cmd.linear.x = 0.0;
            if (!allow_turning_when_blocked_)
                safe_cmd.angular.z = 0.0;

            block_until_ = now.toSec() + cooldown_sec_;
            resetWatchdog();
            return safe_cmd;
        }

        if (shouldTriggerProgressStop(front_range, raw_cmd))
        {
            ROS_WARN("collision_guard: robot appears stuck or making no real front progress. Stopping command.");

            safe_cmd.linear.x = 0.0;
            if (!allow_turning_when_blocked_)
                safe_cmd.angular.z = 0.0;

            block_until_ = now.toSec() + cooldown_sec_;
            return safe_cmd;
        }

        if (raw_cmd.linear.x <= min_forward_cmd_)
            resetWatchdog();

        return safe_cmd;
    }

    void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {
        geometry_msgs::Twist safe_cmd = makeSafeCmd(*msg);
        cmd_pub_.publish(safe_cmd);
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber scan_sub_;
    ros::Subscriber cmd_sub_;
    ros::Publisher cmd_pub_;

    sensor_msgs::LaserScan latest_scan_;
    bool has_scan_;

    double block_until_;

    bool watchdog_active_;
    ros::Time watchdog_start_time_;
    double watchdog_reference_range_;

    double front_angle_deg_;
    double stop_distance_;
    double progress_check_distance_;
    double progress_timeout_sec_;
    double progress_epsilon_;
    double cooldown_sec_;
    double min_forward_cmd_;
    double max_turn_for_progress_check_;
    bool allow_turning_when_blocked_;

    std::string scan_topic_;
    std::string input_cmd_topic_;
    std::string output_cmd_topic_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "collision_guard");
    CollisionGuard node;
    ros::spin();
    return 0;
}