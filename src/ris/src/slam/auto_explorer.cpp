#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <cmath>
#include <limits>
#include <string>

class AutoExplorer
{
public:
    AutoExplorer()
        : nh_(), pnh_("~"),
          has_scan_(false),
          has_odom_(false),
          exploration_done_(false),
          state_(FORWARD),
          preferred_turn_left_(true),
          recovery_attempts_(0),
          tracking_progress_(false)
    {
        pnh_.param("scan_topic", scan_topic_, std::string("/scan"));
        pnh_.param("odom_topic", odom_topic_, std::string("/odom"));
        pnh_.param("cmd_topic", cmd_topic_, std::string("/cmd_vel"));
        pnh_.param("exploration_done_topic", exploration_done_topic_, std::string("/exploration_done"));
        pnh_.param("control_rate_hz", control_rate_hz_, 10.0);
        pnh_.param("front_clear_dist", front_clear_dist_, 0.90);
        pnh_.param("front_slow_dist", front_slow_dist_, 0.65);
        pnh_.param("front_block_dist", front_block_dist_, 0.42);
        pnh_.param("emergency_dist", emergency_dist_, 0.24);
        pnh_.param("side_open_dist", side_open_dist_, 0.65);
        pnh_.param("wall_follow_target", wall_follow_target_, 0.55);
        pnh_.param("wall_follow_gain", wall_follow_gain_, 1.2);
        pnh_.param("forward_speed_max", forward_speed_max_, 0.18);
        pnh_.param("forward_speed_min", forward_speed_min_, 0.08);
        pnh_.param("turn_speed_soft", turn_speed_soft_, 0.45);
        pnh_.param("turn_speed_hard", turn_speed_hard_, 0.75);
        pnh_.param("backup_speed", backup_speed_, -0.10);
        pnh_.param("backup_duration_sec", backup_duration_sec_, 0.7);
        pnh_.param("turn_duration_sec", turn_duration_sec_, 0.9);
        pnh_.param("escape_turn_duration_sec", escape_turn_duration_sec_, 1.5);
        pnh_.param("stuck_timeout_sec", stuck_timeout_sec_, 2.0);
        pnh_.param("stuck_min_progress", stuck_min_progress_, 0.05);
        pnh_.param("max_recovery_attempts", max_recovery_attempts_, 4);
        pnh_.param("front_sector_deg", front_sector_deg_, 20.0);
        pnh_.param("side_sector_min_deg", side_sector_min_deg_, 20.0);
        pnh_.param("side_sector_max_deg", side_sector_max_deg_, 80.0);
        scan_sub_ = nh_.subscribe(scan_topic_, 1, &AutoExplorer::scanCallback, this);
        odom_sub_ = nh_.subscribe(odom_topic_, 1, &AutoExplorer::odomCallback, this);
        done_sub_ = nh_.subscribe(exploration_done_topic_, 1, &AutoExplorer::doneCallback, this);

        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_topic_, 1);
        timer_ = nh_.createTimer(ros::Duration(1.0 / control_rate_hz_), &AutoExplorer::timerCallback, this);

        ROS_INFO("auto_explorer started.");
        ROS_INFO_STREAM("scan_topic: " << scan_topic_);
        ROS_INFO_STREAM("odom_topic: " << odom_topic_);
        ROS_INFO_STREAM("cmd_topic: " << cmd_topic_);
        ROS_INFO_STREAM("exploration_done_topic: " << exploration_done_topic_);
    }

private:
    enum State
    {
        FORWARD = 0,
        BACKUP = 1,
        TURN_LEFT = 2,
        TURN_RIGHT = 3,
        ESCAPE = 4,
        DONE = 5
    };

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        latest_scan_ = *msg;
        has_scan_ = true;
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        latest_odom_ = *msg;
        has_odom_ = true;
    }

    void doneCallback(const std_msgs::Bool::ConstPtr& msg)
    {
        if (msg->data)
        {
            exploration_done_ = true;
            state_ = DONE;
            publishZero();
            ROS_INFO("auto_explorer: received exploration_done=true, stopping.");
        }
    }

    void publishZero()
    {
        geometry_msgs::Twist cmd;
        cmd_pub_.publish(cmd);
    }

    double getMinRangeInSector(double deg_min, double deg_max) const
    {
        if (!has_scan_ || latest_scan_.ranges.empty())
            return std::numeric_limits<double>::infinity();

        const double a_min = deg_min * M_PI / 180.0;
        const double a_max = deg_max * M_PI / 180.0;

        double min_range = std::numeric_limits<double>::infinity();

        for (size_t i = 0; i < latest_scan_.ranges.size(); ++i)
        {
            double angle = latest_scan_.angle_min + static_cast<double>(i) * latest_scan_.angle_increment;
            if (angle < a_min || angle > a_max)
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

    void getSectorRanges(double& front, double& left, double& right,
                         double& front_left, double& front_right) const
    {
        front = getMinRangeInSector(-front_sector_deg_, front_sector_deg_);
        front_left = getMinRangeInSector(side_sector_min_deg_, side_sector_max_deg_);
        front_right = getMinRangeInSector(-side_sector_max_deg_, -side_sector_min_deg_);
        left = getMinRangeInSector(60.0, 100.0);
        right = getMinRangeInSector(-100.0, -60.0);
    }

    double currentOdomX() const
    {
        return latest_odom_.pose.pose.position.x;
    }

    double currentOdomY() const
    {
        return latest_odom_.pose.pose.position.y;
    }

    double odomDistanceFrom(double x0, double y0) const
    {
        const double dx = currentOdomX() - x0;
        const double dy = currentOdomY() - y0;
        return std::sqrt(dx * dx + dy * dy);
    }

    void resetProgressTracking()
    {
        tracking_progress_ = false;
    }

    void startProgressTracking()
    {
        if (!has_odom_)
            return;

        tracking_progress_ = true;
        progress_start_time_ = ros::Time::now();
        progress_start_x_ = currentOdomX();
        progress_start_y_ = currentOdomY();
    }

    bool isStuckForward() const
    {
        if (!tracking_progress_ || !has_odom_)
            return false;

        const double elapsed = (ros::Time::now() - progress_start_time_).toSec();
        if (elapsed < stuck_timeout_sec_)
            return false;

        const double dist = odomDistanceFrom(progress_start_x_, progress_start_y_);
        return dist < stuck_min_progress_;
    }

    void setTimedState(State new_state, double duration_sec)
    {
        state_ = new_state;
        state_end_time_ = ros::Time::now() + ros::Duration(duration_sec);
    }

    bool timedStateFinished() const
    {
        return ros::Time::now() >= state_end_time_;
    }

    void chooseTurnDirection(double left_open, double right_open)
    {
        if (std::isfinite(left_open) && std::isfinite(right_open))
            preferred_turn_left_ = (left_open >= right_open);
        else
            preferred_turn_left_ = !preferred_turn_left_;
    }

    geometry_msgs::Twist computeForwardCmd(double front, double left, double right)
    {
        geometry_msgs::Twist cmd;

        if (front <= front_slow_dist_)
        {
            double ratio = (front - front_block_dist_) / std::max(0.001, (front_slow_dist_ - front_block_dist_));
            if (ratio < 0.0) ratio = 0.0;
            if (ratio > 1.0) ratio = 1.0;
            cmd.linear.x = forward_speed_min_ + ratio * (forward_speed_max_ - forward_speed_min_);
        }
        else
        {
            cmd.linear.x = forward_speed_max_;
        }

        if (preferred_turn_left_)
        {
            if (std::isfinite(left) && left < 2.0)
                cmd.angular.z = wall_follow_gain_ * (left - wall_follow_target_);
            else if (std::isfinite(right) && right < 2.0)
                cmd.angular.z = -0.6 * wall_follow_gain_ * (right - wall_follow_target_);
        }
        else
        {
            if (std::isfinite(right) && right < 2.0)
                cmd.angular.z = -wall_follow_gain_ * (right - wall_follow_target_);
            else if (std::isfinite(left) && left < 2.0)
                cmd.angular.z = 0.6 * wall_follow_gain_ * (left - wall_follow_target_);
        }

        if (std::isfinite(left) && left < 0.30)
            cmd.angular.z -= 0.5;
        if (std::isfinite(right) && right < 0.30)
            cmd.angular.z += 0.5;
        if (cmd.angular.z > turn_speed_soft_) cmd.angular.z = turn_speed_soft_;
        if (cmd.angular.z < -turn_speed_soft_) cmd.angular.z = -turn_speed_soft_;

        return cmd;
    }

    void runRecovery(double front_left, double front_right)
    {
        recovery_attempts_++;

        if (recovery_attempts_ > max_recovery_attempts_)
        {
            chooseTurnDirection(front_left, front_right);
            setTimedState(ESCAPE, backup_duration_sec_ + escape_turn_duration_sec_);
            recovery_attempts_ = 0;
            resetProgressTracking();
            ROS_WARN("auto_explorer: entering ESCAPE maneuver.");
            return;
        }

        chooseTurnDirection(front_left, front_right);
        setTimedState(BACKUP, backup_duration_sec_);
        resetProgressTracking();
        ROS_WARN("auto_explorer: entering BACKUP recovery.");
    }

    geometry_msgs::Twist commandForTimedState()
    {
        geometry_msgs::Twist cmd;

        switch (state_)
        {
            case BACKUP:
                cmd.linear.x = backup_speed_;
                cmd.angular.z = 0.0;
                break;

            case TURN_LEFT:
                cmd.linear.x = 0.0;
                cmd.angular.z = turn_speed_hard_;
                break;

            case TURN_RIGHT:
                cmd.linear.x = 0.0;
                cmd.angular.z = -turn_speed_hard_;
                break;

            case ESCAPE:
            {
                double total = backup_duration_sec_ + escape_turn_duration_sec_;
                double elapsed = total - (state_end_time_ - ros::Time::now()).toSec();

                if (elapsed < backup_duration_sec_)
                {
                    cmd.linear.x = backup_speed_;
                    cmd.angular.z = 0.0;
                }
                else
                {
                    cmd.linear.x = 0.0;
                    cmd.angular.z = preferred_turn_left_ ? turn_speed_hard_ : -turn_speed_hard_;
                }
                break;
            }

            case DONE:
            default:
                break;
        }

        return cmd;
    }

    void advanceTimedStateAfterCompletion()
    {
        switch (state_)
        {
            case BACKUP:
                setTimedState(preferred_turn_left_ ? TURN_LEFT : TURN_RIGHT, turn_duration_sec_);
                ROS_INFO_STREAM("auto_explorer: backup finished, turning "
                                << (preferred_turn_left_ ? "LEFT" : "RIGHT"));
                break;

            case TURN_LEFT:
            case TURN_RIGHT:
            case ESCAPE:
                state_ = FORWARD;
                resetProgressTracking();
                ROS_INFO("auto_explorer: returning to FORWARD state.");
                break;

            default:
                state_ = FORWARD;
                break;
        }
    }

    void timerCallback(const ros::TimerEvent&)
    {
        if (exploration_done_)
        {
            state_ = DONE;
            publishZero();
            return;
        }

        if (!has_scan_)
        {
            publishZero();
            return;
        }

        double front, left, right, front_left, front_right;
        getSectorRanges(front, left, right, front_left, front_right);

        if (state_ == BACKUP || state_ == TURN_LEFT || state_ == TURN_RIGHT || state_ == ESCAPE)
        {
            if (timedStateFinished())
                advanceTimedStateAfterCompletion();

            geometry_msgs::Twist cmd = commandForTimedState();
            cmd_pub_.publish(cmd);
            return;
        }

        if (front < emergency_dist_)
        {
            ROS_WARN_STREAM("auto_explorer: emergency obstacle ahead. front=" << front);
            runRecovery(front_left, front_right);
            cmd_pub_.publish(commandForTimedState());
            return;
        }

        if (front < front_block_dist_)
        {
            chooseTurnDirection(front_left, front_right);
            setTimedState(preferred_turn_left_ ? TURN_LEFT : TURN_RIGHT, turn_duration_sec_);
            resetProgressTracking();
            cmd_pub_.publish(commandForTimedState());
            return;
        }

        if (!tracking_progress_)
            startProgressTracking();

        if (isStuckForward())
        {
            ROS_WARN("auto_explorer: stuck detected from odom progress. Running recovery.");
            runRecovery(front_left, front_right);
            cmd_pub_.publish(commandForTimedState());
            return;
        }

        geometry_msgs::Twist cmd = computeForwardCmd(front, left, right);
        cmd_pub_.publish(cmd);
        if (has_odom_ && tracking_progress_)
        {
            double dist = odomDistanceFrom(progress_start_x_, progress_start_y_);
            if (dist >= stuck_min_progress_)
                startProgressTracking();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber scan_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber done_sub_;
    ros::Publisher cmd_pub_;
    ros::Timer timer_;

    sensor_msgs::LaserScan latest_scan_;
    nav_msgs::Odometry latest_odom_;

    bool has_scan_;
    bool has_odom_;
    bool exploration_done_;

    State state_;
    ros::Time state_end_time_;
    bool preferred_turn_left_;
    int recovery_attempts_;
    bool tracking_progress_;
    ros::Time progress_start_time_;
    double progress_start_x_;
    double progress_start_y_;
    double control_rate_hz_;
    double front_clear_dist_;
    double front_slow_dist_;
    double front_block_dist_;
    double emergency_dist_;
    double side_open_dist_;
    double wall_follow_target_;
    double wall_follow_gain_;
    double forward_speed_max_;
    double forward_speed_min_;
    double turn_speed_soft_;
    double turn_speed_hard_;
    double backup_speed_;
    double backup_duration_sec_;
    double turn_duration_sec_;
    double escape_turn_duration_sec_;
    double stuck_timeout_sec_;
    double stuck_min_progress_;
    int max_recovery_attempts_;
    double front_sector_deg_;
    double side_sector_min_deg_;
    double side_sector_max_deg_;

    std::string scan_topic_;
    std::string odom_topic_;
    std::string cmd_topic_;
    std::string exploration_done_topic_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "auto_explorer");
    AutoExplorer node;
    ros::spin();
    return 0;
}