#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>

#include <cmath>
#include <limits>
#include <string>
#include <algorithm>

class AutoExplorer
{
public:
    AutoExplorer()
        : nh_(), pnh_("~"),
          has_scan_(false),
          has_odom_(false),
          exploration_done_(false),
          state_(FORWARD),
          preferred_left_(true),
          recovery_count_(0),
          progress_tracking_(false),
          front_block_count_(0),
          front_emergency_count_(0),
          rear_block_count_(0),
          rear_emergency_count_(0),
          in_corridor_(false),
          reverse_chain_count_(0)
    {
        pnh_.param("scan_topic", scan_topic_, std::string("/scan"));
        pnh_.param("odom_topic", odom_topic_, std::string("/odom"));
        pnh_.param("cmd_topic", cmd_topic_, std::string("/cmd_vel"));
        pnh_.param("exploration_done_topic", done_topic_, std::string("/exploration_done"));

        pnh_.param("control_rate_hz", control_rate_hz_, 12.0);

        pnh_.param("front_clear_dist", front_clear_dist_, 1.20);
        pnh_.param("front_caution_dist", front_caution_dist_, 0.90);
        pnh_.param("front_block_dist", front_block_dist_, 0.48);
        pnh_.param("front_emergency_dist", front_emergency_dist_, 0.28);

        pnh_.param("rear_block_dist", rear_block_dist_, 0.24);
        pnh_.param("rear_emergency_dist", rear_emergency_dist_, 0.16);

        pnh_.param("side_block_dist", side_block_dist_, 0.22);
        pnh_.param("side_emergency_dist", side_emergency_dist_, 0.14);

        pnh_.param("forward_speed_max", forward_speed_max_, 0.13);
        pnh_.param("forward_speed_min", forward_speed_min_, 0.045);
        pnh_.param("forward_speed_recovery", forward_speed_recovery_, 0.07);
        pnh_.param("corridor_speed_max", corridor_speed_max_, 0.09);
        pnh_.param("reverse_speed", reverse_speed_, -0.085);

        pnh_.param("turn_speed_soft", turn_speed_soft_, 0.38);
        pnh_.param("turn_speed_hard", turn_speed_hard_, 0.82);
        pnh_.param("reverse_turn_speed", reverse_turn_speed_, 0.58);

        pnh_.param("reverse_duration_sec", reverse_duration_sec_, 0.75);
        pnh_.param("turn_duration_sec", turn_duration_sec_, 1.00);
        pnh_.param("escape_turn_duration_sec", escape_turn_duration_sec_, 1.80);
        pnh_.param("post_recovery_guidance_sec", post_recovery_guidance_sec_, 1.20);

        pnh_.param("stuck_timeout_sec", stuck_timeout_sec_, 1.20);
        pnh_.param("stuck_min_progress", stuck_min_progress_, 0.025);

        pnh_.param("max_recovery_attempts", max_recovery_attempts_, 3);

        pnh_.param("front_sector_deg", front_sector_deg_, 35.0);
        pnh_.param("front_wide_sector_deg", front_wide_sector_deg_, 60.0);
        pnh_.param("side_sector_min_deg", side_sector_min_deg_, 20.0);
        pnh_.param("side_sector_max_deg", side_sector_max_deg_, 95.0);
        pnh_.param("rear_sector_deg", rear_sector_deg_, 35.0);

        pnh_.param("front_block_cycles_before_recovery", front_block_cycles_before_recovery_, 2);
        pnh_.param("front_emergency_cycles_before_recovery", front_emergency_cycles_before_recovery_, 1);
        pnh_.param("rear_block_cycles_before_stop_reverse", rear_block_cycles_before_stop_reverse_, 1);
        pnh_.param("rear_emergency_cycles_before_stop_reverse", rear_emergency_cycles_before_stop_reverse_, 1);

        pnh_.param("wall_follow_target", wall_follow_target_, 0.55);
        pnh_.param("wall_follow_gain", wall_follow_gain_, 0.42);
        pnh_.param("obstacle_avoid_gain", obstacle_avoid_gain_, 0.90);
        pnh_.param("steering_deadband", steering_deadband_, 0.06);
        pnh_.param("guidance_bias_strength", guidance_bias_strength_, 0.18);

        pnh_.param("corridor_width_threshold", corridor_width_threshold_, 1.30);
        pnh_.param("corridor_wall_presence_max", corridor_wall_presence_max_, 0.90);
        pnh_.param("corridor_center_gain", corridor_center_gain_, 1.10);
        pnh_.param("corridor_heading_gain", corridor_heading_gain_, 0.50);
        pnh_.param("corridor_front_slow_dist", corridor_front_slow_dist_, 0.75);

        pnh_.param("open_space_bias_gain", open_space_bias_gain_, 0.26);
        pnh_.param("turn_preference_hold_sec", turn_preference_hold_sec_, 1.50);

        pnh_.param("reverse_cooldown_sec", reverse_cooldown_sec_, 3.00);
        pnh_.param("max_reverse_chain", max_reverse_chain_, 2);

        scan_sub_ = nh_.subscribe(scan_topic_, 1, &AutoExplorer::scanCallback, this);
        odom_sub_ = nh_.subscribe(odom_topic_, 1, &AutoExplorer::odomCallback, this);
        done_sub_ = nh_.subscribe(done_topic_, 1, &AutoExplorer::doneCallback, this);

        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_topic_, 1);
        timer_ = nh_.createTimer(ros::Duration(1.0 / control_rate_hz_), &AutoExplorer::timerCallback, this);

        bias_until_ = ros::Time(0);
        preference_hold_until_ = ros::Time(0);
        reverse_cooldown_until_ = ros::Time(0);

        ROS_INFO("auto_explorer improved hybrid local exploration started.");
    }

private:
    enum State
    {
        FORWARD = 0,
        REVERSE_LEFT = 1,
        REVERSE_RIGHT = 2,
        TURN_LEFT = 3,
        TURN_RIGHT = 4,
        ESCAPE_LEFT = 5,
        ESCAPE_RIGHT = 6,
        DONE = 7
    };

    struct SectorInfo
    {
        double front;
        double front_wide;
        double front_left;
        double front_right;
        double left;
        double right;
        double rear;
        double rear_left;
        double rear_right;
        double left_diag;
        double right_diag;
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
        }
    }

    void publishZero()
    {
        geometry_msgs::Twist cmd;
        cmd_pub_.publish(cmd);
    }

    double clamp(double x, double lo, double hi) const
    {
        return std::max(lo, std::min(x, hi));
    }

    double finiteOr(const double x, const double fallback) const
    {
        return std::isfinite(x) ? x : fallback;
    }

    double minRangeSectorDeg(double deg_min, double deg_max) const
    {
        if (!has_scan_ || latest_scan_.ranges.empty())
            return std::numeric_limits<double>::infinity();

        const double a_min = deg_min * M_PI / 180.0;
        const double a_max = deg_max * M_PI / 180.0;
        double min_r = std::numeric_limits<double>::infinity();

        for (size_t i = 0; i < latest_scan_.ranges.size(); ++i)
        {
            const double a = latest_scan_.angle_min + static_cast<double>(i) * latest_scan_.angle_increment;
            if (a < a_min || a > a_max)
                continue;

            const double r = latest_scan_.ranges[i];
            if (std::isnan(r) || std::isinf(r))
                continue;
            if (r < latest_scan_.range_min || r > latest_scan_.range_max)
                continue;

            if (r < min_r)
                min_r = r;
        }

        return min_r;
    }

    SectorInfo readSectors() const
    {
        SectorInfo s;
        s.front = minRangeSectorDeg(-front_sector_deg_, front_sector_deg_);
        s.front_wide = minRangeSectorDeg(-front_wide_sector_deg_, front_wide_sector_deg_);
        s.front_left = minRangeSectorDeg(side_sector_min_deg_, side_sector_max_deg_);
        s.front_right = minRangeSectorDeg(-side_sector_max_deg_, -side_sector_min_deg_);
        s.left = minRangeSectorDeg(70.0, 100.0);
        s.right = minRangeSectorDeg(-100.0, -70.0);
        s.left_diag = minRangeSectorDeg(35.0, 65.0);
        s.right_diag = minRangeSectorDeg(-65.0, -35.0);

        s.rear = minRangeSectorDeg(180.0 - rear_sector_deg_, 180.0);
        const double rear2 = minRangeSectorDeg(-180.0, -180.0 + rear_sector_deg_);
        if (rear2 < s.rear)
            s.rear = rear2;

        s.rear_left = minRangeSectorDeg(110.0, 160.0);
        s.rear_right = minRangeSectorDeg(-160.0, -110.0);
        return s;
    }

    double odomX() const
    {
        return latest_odom_.pose.pose.position.x;
    }

    double odomY() const
    {
        return latest_odom_.pose.pose.position.y;
    }

    double odomDistanceFrom(double x0, double y0) const
    {
        const double dx = odomX() - x0;
        const double dy = odomY() - y0;
        return std::sqrt(dx * dx + dy * dy);
    }

    void resetProgressTracking()
    {
        progress_tracking_ = false;
    }

    void startProgressTracking()
    {
        if (!has_odom_)
            return;

        progress_tracking_ = true;
        progress_start_time_ = ros::Time::now();
        progress_start_x_ = odomX();
        progress_start_y_ = odomY();
    }

    bool stuckForward() const
    {
        if (!progress_tracking_ || !has_odom_)
            return false;

        const double elapsed = (ros::Time::now() - progress_start_time_).toSec();
        if (elapsed < stuck_timeout_sec_)
            return false;

        return odomDistanceFrom(progress_start_x_, progress_start_y_) < stuck_min_progress_;
    }

    void setTimedState(State s, double sec)
    {
        state_ = s;
        state_end_time_ = ros::Time::now() + ros::Duration(sec);
    }

    bool timedStateFinished() const
    {
        return ros::Time::now() >= state_end_time_;
    }

    void updateCounters(double front, double rear)
    {
        front_block_count_ = (std::isfinite(front) && front < front_block_dist_) ? front_block_count_ + 1 : 0;
        front_emergency_count_ = (std::isfinite(front) && front < front_emergency_dist_) ? front_emergency_count_ + 1 : 0;
        rear_block_count_ = (std::isfinite(rear) && rear < rear_block_dist_) ? rear_block_count_ + 1 : 0;
        rear_emergency_count_ = (std::isfinite(rear) && rear < rear_emergency_dist_) ? rear_emergency_count_ + 1 : 0;
    }

    bool isCorridor(const SectorInfo& s, double& corridor_width, double& center_error) const
    {
        const bool left_seen = std::isfinite(s.left) && s.left < corridor_wall_presence_max_;
        const bool right_seen = std::isfinite(s.right) && s.right < corridor_wall_presence_max_;

        if (!left_seen || !right_seen)
        {
            corridor_width = std::numeric_limits<double>::infinity();
            center_error = 0.0;
            return false;
        }

        corridor_width = s.left + s.right;
        center_error = s.left - s.right; // موجب => أقرب لليمين ويجب أن يميل يسارًا قليلًا

        return corridor_width <= corridor_width_threshold_;
    }

    void choosePreferredTurn(const SectorInfo& s, bool force_refresh = false)
    {
        if (!force_refresh && ros::Time::now() < preference_hold_until_)
            return;

        double left_score = 0.0;
        double right_score = 0.0;

        if (std::isfinite(s.front_left)) left_score += 1.6 * s.front_left;
        if (std::isfinite(s.left_diag))  left_score += 1.0 * s.left_diag;
        if (std::isfinite(s.left))       left_score += 0.9 * s.left;
        if (std::isfinite(s.rear_left))  left_score += 0.3 * s.rear_left;

        if (std::isfinite(s.front_right)) right_score += 1.6 * s.front_right;
        if (std::isfinite(s.right_diag))  right_score += 1.0 * s.right_diag;
        if (std::isfinite(s.right))       right_score += 0.9 * s.right;
        if (std::isfinite(s.rear_right))  right_score += 0.3 * s.rear_right;

        if (std::fabs(left_score - right_score) < 0.12)
            preferred_left_ = !preferred_left_;
        else
            preferred_left_ = left_score > right_score;

        preference_hold_until_ = ros::Time::now() + ros::Duration(turn_preference_hold_sec_);
    }

    double computeForwardSpeedOpen(const SectorInfo& s) const
    {
        const double front = s.front;

        if (!std::isfinite(front))
            return forward_speed_max_;

        if (front <= front_block_dist_)
            return 0.0;

        if (front >= front_clear_dist_)
            return forward_speed_max_;

        if (front <= front_caution_dist_)
        {
            const double ratio = clamp((front - front_block_dist_) /
                                       std::max(0.001, front_caution_dist_ - front_block_dist_), 0.0, 1.0);
            return forward_speed_min_ + ratio * (forward_speed_recovery_ - forward_speed_min_);
        }

        const double ratio = clamp((front - front_caution_dist_) /
                                   std::max(0.001, front_clear_dist_ - front_caution_dist_), 0.0, 1.0);
        return forward_speed_recovery_ + ratio * (forward_speed_max_ - forward_speed_recovery_);
    }

    double computeForwardSpeedCorridor(const SectorInfo& s) const
    {
        double speed = corridor_speed_max_;

        if (std::isfinite(s.front))
        {
            if (s.front < corridor_front_slow_dist_)
            {
                const double ratio = clamp((s.front - front_block_dist_) /
                                           std::max(0.001, corridor_front_slow_dist_ - front_block_dist_), 0.0, 1.0);
                speed = forward_speed_min_ + ratio * (corridor_speed_max_ - forward_speed_min_);
            }
        }

        return clamp(speed, forward_speed_min_, corridor_speed_max_);
    }

    geometry_msgs::Twist computeCorridorCommand(const SectorInfo& s, double corridor_width, double center_error)
    {
        geometry_msgs::Twist cmd;

        cmd.linear.x = computeForwardSpeedCorridor(s);

        double steer = 0.0;

        // تمركز في منتصف الممر
        steer += corridor_center_gain_ * center_error;

        // تصحيح أمامي إذا أحد الجانبين الأماميين أضيق
        if (std::isfinite(s.front_left) && std::isfinite(s.front_right))
            steer += corridor_heading_gain_ * (s.front_left - s.front_right);

        // حماية جانبية قوية
        if (std::isfinite(s.left) && s.left < side_emergency_dist_)
            steer -= 0.70;
        else if (std::isfinite(s.left) && s.left < side_block_dist_)
            steer -= 0.35;

        if (std::isfinite(s.right) && s.right < side_emergency_dist_)
            steer += 0.70;
        else if (std::isfinite(s.right) && s.right < side_block_dist_)
            steer += 0.35;

        // في الممر نريد قيادة هادئة ومتمركزة
        if (std::fabs(steer) < steering_deadband_)
            steer = 0.0;

        cmd.angular.z = clamp(steer, -turn_speed_soft_, turn_speed_soft_);
        return cmd;
    }

    geometry_msgs::Twist computeOpenSpaceCommand(const SectorInfo& s)
    {
        geometry_msgs::Twist cmd;
        cmd.linear.x = computeForwardSpeedOpen(s);

        double steer = 0.0;

        // توجيه نحو المساحة الأوسع
        double open_bias = 0.0;
        const double left_open = finiteOr(s.front_left, front_clear_dist_ + 0.5) + 0.5 * finiteOr(s.left_diag, 1.0);
        const double right_open = finiteOr(s.front_right, front_clear_dist_ + 0.5) + 0.5 * finiteOr(s.right_diag, 1.0);
        open_bias += open_space_bias_gain_ * (left_open - right_open);

        steer += open_bias;

        // obstacle avoidance أمامي
        if (std::isfinite(s.front_left) && s.front_left < front_clear_dist_)
            steer -= obstacle_avoid_gain_ * clamp((front_clear_dist_ - s.front_left) /
                                                  std::max(0.001, front_clear_dist_), 0.0, 1.0);

        if (std::isfinite(s.front_right) && s.front_right < front_clear_dist_)
            steer += obstacle_avoid_gain_ * clamp((front_clear_dist_ - s.front_right) /
                                                  std::max(0.001, front_clear_dist_), 0.0, 1.0);

        // wall-follow خفيف فقط إذا أحد الجدارين قريب
        if (preferred_left_)
        {
            if (std::isfinite(s.left) && s.left < 1.3)
                steer += wall_follow_gain_ * (s.left - wall_follow_target_);
        }
        else
        {
            if (std::isfinite(s.right) && s.right < 1.3)
                steer -= wall_follow_gain_ * (s.right - wall_follow_target_);
        }

        // حماية جانبية
        if (std::isfinite(s.left) && s.left < side_emergency_dist_)
            steer -= 0.70;
        else if (std::isfinite(s.left) && s.left < side_block_dist_)
            steer -= 0.35;

        if (std::isfinite(s.right) && s.right < side_emergency_dist_)
            steer += 0.70;
        else if (std::isfinite(s.right) && s.right < side_block_dist_)
            steer += 0.35;

        // انحياز قصير بعد recovery
        if (ros::Time::now() < bias_until_)
            steer += preferred_left_ ? guidance_bias_strength_ : -guidance_bias_strength_;

        if (std::fabs(steer) < steering_deadband_)
            steer = 0.0;

        cmd.angular.z = clamp(steer, -turn_speed_soft_, turn_speed_soft_);
        return cmd;
    }

    void resetRecoveryIfHealthy(const SectorInfo& s)
    {
        if (s.front > front_caution_dist_ && s.front_wide > front_caution_dist_)
            recovery_count_ = 0;
    }

    bool reverseAllowed() const
    {
        if (ros::Time::now() < reverse_cooldown_until_)
            return false;
        if (reverse_chain_count_ >= max_reverse_chain_)
            return false;
        return true;
    }

    void beginRecovery(const SectorInfo& s, bool emergency)
    {
        recovery_count_++;
        choosePreferredTurn(s, true);

        const bool rear_safe = std::isfinite(s.rear) && s.rear > rear_block_dist_;
        const bool rear_emergency = std::isfinite(s.rear) && s.rear < rear_emergency_dist_;

        // في الممر الضيق: لا نحب الرجوع للخلف إلا إذا اضطررنا
        if (in_corridor_)
        {
            if (!emergency || rear_emergency || !reverseAllowed())
            {
                setTimedState(preferred_left_ ? TURN_LEFT : TURN_RIGHT, turn_duration_sec_);
                resetProgressTracking();
                return;
            }
        }

        // في المساحة الواسعة: فضّل turn أولاً إذا هناك فراغ جانبي جيد
        const double left_space = finiteOr(s.front_left, 2.0) + 0.5 * finiteOr(s.left_diag, 1.0);
        const double right_space = finiteOr(s.front_right, 2.0) + 0.5 * finiteOr(s.right_diag, 1.0);
        const bool strong_side_escape = std::max(left_space, right_space) > 1.15;

        if (strong_side_escape && !emergency)
        {
            preferred_left_ = left_space >= right_space;
            setTimedState(preferred_left_ ? TURN_LEFT : TURN_RIGHT, turn_duration_sec_);
            resetProgressTracking();
            return;
        }

        if (rear_safe && !rear_emergency && reverseAllowed())
        {
            setTimedState(preferred_left_ ? REVERSE_LEFT : REVERSE_RIGHT, reverse_duration_sec_);
            reverse_chain_count_++;
            reverse_cooldown_until_ = ros::Time::now() + ros::Duration(reverse_cooldown_sec_);
            resetProgressTracking();
            return;
        }

        if (recovery_count_ <= max_recovery_attempts_)
        {
            setTimedState(preferred_left_ ? TURN_LEFT : TURN_RIGHT, turn_duration_sec_);
            resetProgressTracking();
            return;
        }

        setTimedState(preferred_left_ ? ESCAPE_LEFT : ESCAPE_RIGHT, escape_turn_duration_sec_);
        recovery_count_ = 0;
        reverse_chain_count_ = 0;
        resetProgressTracking();
    }

    geometry_msgs::Twist timedCommand() const
    {
        geometry_msgs::Twist cmd;

        switch (state_)
        {
            case REVERSE_LEFT:
                cmd.linear.x = reverse_speed_;
                cmd.angular.z = reverse_turn_speed_;
                break;

            case REVERSE_RIGHT:
                cmd.linear.x = reverse_speed_;
                cmd.angular.z = -reverse_turn_speed_;
                break;

            case TURN_LEFT:
                cmd.linear.x = 0.0;
                cmd.angular.z = turn_speed_hard_;
                break;

            case TURN_RIGHT:
                cmd.linear.x = 0.0;
                cmd.angular.z = -turn_speed_hard_;
                break;

            case ESCAPE_LEFT:
                cmd.linear.x = 0.0;
                cmd.angular.z = turn_speed_hard_;
                break;

            case ESCAPE_RIGHT:
                cmd.linear.x = 0.0;
                cmd.angular.z = -turn_speed_hard_;
                break;

            case DONE:
            case FORWARD:
            default:
                break;
        }

        return cmd;
    }

    void advanceTimedState()
    {
        switch (state_)
        {
            case REVERSE_LEFT:
                setTimedState(TURN_LEFT, turn_duration_sec_);
                break;

            case REVERSE_RIGHT:
                setTimedState(TURN_RIGHT, turn_duration_sec_);
                break;

            case TURN_LEFT:
            case TURN_RIGHT:
            case ESCAPE_LEFT:
            case ESCAPE_RIGHT:
                state_ = FORWARD;
                bias_until_ = ros::Time::now() + ros::Duration(post_recovery_guidance_sec_);
                resetProgressTracking();
                break;

            default:
                state_ = FORWARD;
                break;
        }
    }

    geometry_msgs::Twist safetyGate(const geometry_msgs::Twist& in, const SectorInfo& s) const
    {
        geometry_msgs::Twist out = in;

        if (out.linear.x > 0.0 && std::isfinite(s.front) && s.front < front_block_dist_)
            out.linear.x = 0.0;

        if (out.linear.x < 0.0 && std::isfinite(s.rear) && s.rear < rear_block_dist_)
            out.linear.x = 0.0;

        return out;
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

        SectorInfo s = readSectors();
        updateCounters(s.front, s.rear);

        double corridor_width = std::numeric_limits<double>::infinity();
        double center_error = 0.0;
        in_corridor_ = isCorridor(s, corridor_width, center_error);

        if (state_ == REVERSE_LEFT || state_ == REVERSE_RIGHT ||
            state_ == TURN_LEFT || state_ == TURN_RIGHT ||
            state_ == ESCAPE_LEFT || state_ == ESCAPE_RIGHT)
        {
            // إذا الخلف لم يعد آمنًا أثناء الرجوع، انتقل فورًا إلى turning
            if ((state_ == REVERSE_LEFT || state_ == REVERSE_RIGHT) &&
                (rear_emergency_count_ >= rear_emergency_cycles_before_stop_reverse_ ||
                 rear_block_count_ >= rear_block_cycles_before_stop_reverse_))
            {
                choosePreferredTurn(s, true);
                setTimedState(preferred_left_ ? TURN_LEFT : TURN_RIGHT, turn_duration_sec_);
            }

            // حماية جانبية أثناء الدوران
            if ((state_ == TURN_LEFT || state_ == ESCAPE_LEFT) &&
                std::isfinite(s.left) && s.left < side_emergency_dist_)
            {
                state_ = TURN_RIGHT;
                state_end_time_ = ros::Time::now() + ros::Duration(turn_duration_sec_);
                preferred_left_ = false;
            }

            if ((state_ == TURN_RIGHT || state_ == ESCAPE_RIGHT) &&
                std::isfinite(s.right) && s.right < side_emergency_dist_)
            {
                state_ = TURN_LEFT;
                state_end_time_ = ros::Time::now() + ros::Duration(turn_duration_sec_);
                preferred_left_ = true;
            }

            if (timedStateFinished())
                advanceTimedState();

            geometry_msgs::Twist cmd = timedCommand();
            cmd = safetyGate(cmd, s);
            cmd_pub_.publish(cmd);
            return;
        }

        // حالات recovery
        if (front_emergency_count_ >= front_emergency_cycles_before_recovery_)
        {
            beginRecovery(s, true);
            geometry_msgs::Twist cmd = timedCommand();
            cmd = safetyGate(cmd, s);
            cmd_pub_.publish(cmd);
            return;
        }

        if (front_block_count_ >= front_block_cycles_before_recovery_)
        {
            beginRecovery(s, false);
            geometry_msgs::Twist cmd = timedCommand();
            cmd = safetyGate(cmd, s);
            cmd_pub_.publish(cmd);
            return;
        }

        if (!progress_tracking_)
            startProgressTracking();

        if (stuckForward())
        {
            beginRecovery(s, false);
            geometry_msgs::Twist cmd = timedCommand();
            cmd = safetyGate(cmd, s);
            cmd_pub_.publish(cmd);
            return;
        }

        // اختيار تفضيل الالتفاف بين الحين والآخر فقط
        choosePreferredTurn(s, false);

        geometry_msgs::Twist cmd;
        if (in_corridor_)
            cmd = computeCorridorCommand(s, corridor_width, center_error);
        else
            cmd = computeOpenSpaceCommand(s);

        cmd = safetyGate(cmd, s);
        cmd_pub_.publish(cmd);

        if (has_odom_ && progress_tracking_)
        {
            const double dist = odomDistanceFrom(progress_start_x_, progress_start_y_);
            if (dist >= stuck_min_progress_)
            {
                startProgressTracking();
                resetRecoveryIfHealthy(s);
                if (s.front > front_caution_dist_)
                    reverse_chain_count_ = 0;
            }
        }

        ROS_INFO_THROTTLE(1.0,
                          "mode=%s state=%d front=%.2f left=%.2f right=%.2f rear=%.2f corridor=%s width=%.2f center_err=%.2f",
                          in_corridor_ ? "CORRIDOR" : "OPEN",
                          static_cast<int>(state_),
                          finiteOr(s.front, -1.0),
                          finiteOr(s.left, -1.0),
                          finiteOr(s.right, -1.0),
                          finiteOr(s.rear, -1.0),
                          in_corridor_ ? "yes" : "no",
                          std::isfinite(corridor_width) ? corridor_width : -1.0,
                          center_error);
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
    bool in_corridor_;

    State state_;
    ros::Time state_end_time_;
    ros::Time bias_until_;
    ros::Time preference_hold_until_;
    ros::Time reverse_cooldown_until_;

    bool preferred_left_;
    int recovery_count_;
    bool progress_tracking_;
    ros::Time progress_start_time_;
    double progress_start_x_;
    double progress_start_y_;

    int front_block_count_;
    int front_emergency_count_;
    int rear_block_count_;
    int rear_emergency_count_;
    int reverse_chain_count_;

    double control_rate_hz_;

    double front_clear_dist_;
    double front_caution_dist_;
    double front_block_dist_;
    double front_emergency_dist_;

    double rear_block_dist_;
    double rear_emergency_dist_;

    double side_block_dist_;
    double side_emergency_dist_;

    double forward_speed_max_;
    double forward_speed_min_;
    double forward_speed_recovery_;
    double corridor_speed_max_;
    double reverse_speed_;

    double turn_speed_soft_;
    double turn_speed_hard_;
    double reverse_turn_speed_;

    double reverse_duration_sec_;
    double turn_duration_sec_;
    double escape_turn_duration_sec_;
    double post_recovery_guidance_sec_;

    double stuck_timeout_sec_;
    double stuck_min_progress_;
    int max_recovery_attempts_;

    double front_sector_deg_;
    double front_wide_sector_deg_;
    double side_sector_min_deg_;
    double side_sector_max_deg_;
    double rear_sector_deg_;

    int front_block_cycles_before_recovery_;
    int front_emergency_cycles_before_recovery_;
    int rear_block_cycles_before_stop_reverse_;
    int rear_emergency_cycles_before_stop_reverse_;

    double wall_follow_target_;
    double wall_follow_gain_;
    double obstacle_avoid_gain_;
    double steering_deadband_;
    double guidance_bias_strength_;

    double corridor_width_threshold_;
    double corridor_wall_presence_max_;
    double corridor_center_gain_;
    double corridor_heading_gain_;
    double corridor_front_slow_dist_;

    double open_space_bias_gain_;
    double turn_preference_hold_sec_;

    double reverse_cooldown_sec_;
    int max_reverse_chain_;

    std::string scan_topic_;
    std::string odom_topic_;
    std::string cmd_topic_;
    std::string done_topic_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "auto_explorer");
    AutoExplorer node;
    ros::spin();
    return 0;
}