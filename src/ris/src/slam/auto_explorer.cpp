#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <cmath>
#include <limits>
#include <string>
#include <vector>
#include <queue>
#include <deque>
#include <algorithm>

class AutoExplorer
{
public:
    AutoExplorer()
        : nh_(),
          pnh_("~"),
          tf_buffer_(),
          tf_listener_(tf_buffer_),
          has_scan_(false),
          has_odom_(false),
          has_map_(false),
          has_pose_in_map_(false),
          exploration_done_(false),
          state_(SELECT_GOAL),
          preferred_left_(true),
          in_corridor_(false),
          recovery_count_(0),
          progress_tracking_(false),
          last_heading_deg_(0.0),
          last_goal_valid_(false),
          goal_progress_tracking_(false)
    {
        pnh_.param("scan_topic", scan_topic_, std::string("/scan"));
        pnh_.param("odom_topic", odom_topic_, std::string("/odom"));
        pnh_.param("map_topic", map_topic_, std::string("/map"));
        pnh_.param("cmd_topic", cmd_topic_, std::string("/cmd_vel"));
        pnh_.param("exploration_done_topic", done_topic_, std::string("/exploration_done"));
        pnh_.param("map_frame", map_frame_, std::string("map"));
        pnh_.param("base_frame", base_frame_, std::string("base_footprint"));

        pnh_.param("control_rate_hz", control_rate_hz_, 10.0);

        // Distances
        pnh_.param("front_clear_dist", front_clear_dist_, 1.40);
        pnh_.param("front_caution_dist", front_caution_dist_, 1.00);
        pnh_.param("front_block_dist", front_block_dist_, 0.55);
        pnh_.param("front_emergency_dist", front_emergency_dist_, 0.32);

        pnh_.param("rear_clear_dist", rear_clear_dist_, 0.65);
        pnh_.param("rear_block_dist", rear_block_dist_, 0.30);
        pnh_.param("rear_emergency_dist", rear_emergency_dist_, 0.20);

        pnh_.param("side_block_dist", side_block_dist_, 0.25);
        pnh_.param("side_emergency_dist", side_emergency_dist_, 0.16);

        // Speeds
        pnh_.param("forward_speed_max", forward_speed_max_, 0.34);
        pnh_.param("forward_speed_min", forward_speed_min_, 0.20);
        pnh_.param("forward_speed_recovery", forward_speed_recovery_, 0.24);
        pnh_.param("corridor_speed_max", corridor_speed_max_, 0.24);
        pnh_.param("reverse_speed", reverse_speed_, -0.20);

        pnh_.param("turn_speed_soft", turn_speed_soft_, 0.65);
        pnh_.param("turn_speed_hard", turn_speed_hard_, 1.20);
        pnh_.param("reverse_turn_speed", reverse_turn_speed_, 0.95);

        pnh_.param("reverse_duration_sec", reverse_duration_sec_, 0.65);
        pnh_.param("turn_duration_sec", turn_duration_sec_, 0.85);
        pnh_.param("escape_turn_duration_sec", escape_turn_duration_sec_, 1.60);
        pnh_.param("recovery_forward_duration_sec", recovery_forward_duration_sec_, 0.55);

        pnh_.param("stuck_timeout_sec", stuck_timeout_sec_, 1.20);
        pnh_.param("stuck_min_progress", stuck_min_progress_, 0.05);

        pnh_.param("max_recovery_attempts", max_recovery_attempts_, 4);
        pnh_.param("max_reverse_chain", max_reverse_chain_, 1);
        pnh_.param("reverse_cooldown_sec", reverse_cooldown_sec_, 2.5);

        // Sectors
        pnh_.param("front_sector_deg", front_sector_deg_, 35.0);
        pnh_.param("front_wide_sector_deg", front_wide_sector_deg_, 60.0);
        pnh_.param("side_sector_min_deg", side_sector_min_deg_, 20.0);
        pnh_.param("side_sector_max_deg", side_sector_max_deg_, 95.0);
        pnh_.param("rear_sector_deg", rear_sector_deg_, 35.0);

        // Steering / corridor
        pnh_.param("steering_deadband", steering_deadband_, 0.04);
        pnh_.param("corridor_width_threshold", corridor_width_threshold_, 1.30);
        pnh_.param("corridor_wall_presence_max", corridor_wall_presence_max_, 0.95);
        pnh_.param("corridor_center_gain", corridor_center_gain_, 1.15);
        pnh_.param("corridor_heading_gain", corridor_heading_gain_, 0.55);
        pnh_.param("corridor_front_slow_dist", corridor_front_slow_dist_, 0.80);

        // Frontier / goal
        pnh_.param("frontier_min_cluster_size", frontier_min_cluster_size_, 8);
        pnh_.param("frontier_robot_clearance_m", frontier_robot_clearance_m_, 0.24);
        pnh_.param("frontier_goal_pullback_m", frontier_goal_pullback_m_, 0.35);
        pnh_.param("frontier_reselect_period_sec", frontier_reselect_period_sec_, 0.8);
        pnh_.param("frontier_size_weight", frontier_size_weight_, 1.3);
        pnh_.param("frontier_distance_weight", frontier_distance_weight_, 0.90);
        pnh_.param("frontier_heading_weight", frontier_heading_weight_, 0.70);
        pnh_.param("frontier_visit_penalty_weight", frontier_visit_penalty_weight_, 1.3);
        pnh_.param("frontier_unknown_density_weight", frontier_unknown_density_weight_, 1.2);
        pnh_.param("frontier_same_goal_bonus", frontier_same_goal_bonus_, 0.35);
        pnh_.param("frontier_narrow_penalty_weight", frontier_narrow_penalty_weight_, 1.2);

        // Goal failure / blacklist
        pnh_.param("goal_reach_dist_m", goal_reach_dist_m_, 0.35);
        pnh_.param("goal_progress_timeout_sec", goal_progress_timeout_sec_, 3.0);
        pnh_.param("goal_progress_min_dist_m", goal_progress_min_dist_m_, 0.12);
        pnh_.param("goal_fail_limit", goal_fail_limit_, 2);
        pnh_.param("goal_blacklist_radius_m", goal_blacklist_radius_m_, 0.70);
        pnh_.param("goal_blacklist_duration_sec", goal_blacklist_duration_sec_, 45.0);

        // Local waypoint
        pnh_.param("waypoint_step_m", waypoint_step_m_, 0.40);
        pnh_.param("waypoint_max_lookahead_m", waypoint_max_lookahead_m_, 1.20);

        // Visit memory
        pnh_.param("visit_update_radius_m", visit_update_radius_m_, 0.25);
        pnh_.param("visit_penalty_cap", visit_penalty_cap_, 10.0);

        // Local planner
        pnh_.param("traj_sim_time_sec", traj_sim_time_sec_, 0.90);
        pnh_.param("traj_dt_sec", traj_dt_sec_, 0.10);
        pnh_.param("traj_goal_weight", traj_goal_weight_, 2.5);
        pnh_.param("traj_clearance_weight", traj_clearance_weight_, 1.8);
        pnh_.param("traj_smoothness_weight", traj_smoothness_weight_, 0.55);
        pnh_.param("traj_visit_weight", traj_visit_weight_, 0.90);
        pnh_.param("traj_progress_weight", traj_progress_weight_, 1.20);

        // Laser filtering
        pnh_.param("laser_sector_quantile", laser_sector_quantile_, 0.25);
        pnh_.param("laser_neighbor_reject_jump", laser_neighbor_reject_jump_, 0.50);

        scan_sub_ = nh_.subscribe(scan_topic_, 1, &AutoExplorer::scanCallback, this);
        odom_sub_ = nh_.subscribe(odom_topic_, 1, &AutoExplorer::odomCallback, this);
        map_sub_ = nh_.subscribe(map_topic_, 1, &AutoExplorer::mapCallback, this);
        done_sub_ = nh_.subscribe(done_topic_, 1, &AutoExplorer::doneCallback, this);

        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_topic_, 1);
        timer_ = nh_.createTimer(ros::Duration(1.0 / control_rate_hz_), &AutoExplorer::timerCallback, this);

        reverse_cooldown_until_ = ros::Time(0);
        last_visit_update_time_ = ros::Time(0);
        last_frontier_select_time_ = ros::Time(0);

        ROS_INFO("auto_explorer advanced production-style explorer started.");
    }

private:
    enum State
    {
        SELECT_GOAL = 0,
        DRIVE_TO_WAYPOINT = 1,
        RECOVERY_TURN = 2,
        RECOVERY_REVERSE = 3,
        RECOVERY_ESCAPE = 4,
        DONE = 5
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

    struct FrontierCluster
    {
        std::vector<int> cells;
        double centroid_x;
        double centroid_y;
        double nearest_x;
        double nearest_y;
        double unknown_density_score;
        double width_score;
        double score;
        double distance_to_robot;
        double heading_error_deg;
        double visit_penalty;
    };

    struct BlacklistedGoal
    {
        double x;
        double y;
        ros::Time until;
    };

    struct TrajectoryCandidate
    {
        double v;
        double w;
        double score;
        double final_x;
        double final_y;
        double final_yaw;
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

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
    {
        latest_map_ = *msg;
        has_map_ = true;

        const std::size_t needed =
            static_cast<std::size_t>(latest_map_.info.width) *
            static_cast<std::size_t>(latest_map_.info.height);

        if (visit_counts_.size() != needed)
        {
            visit_counts_.assign(needed, 0.0);
            ROS_INFO("Visit memory resized to match /map.");
        }
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

    double finiteOr(double x, double fallback) const
    {
        return std::isfinite(x) ? x : fallback;
    }

    double deg2rad(double deg) const
    {
        return deg * M_PI / 180.0;
    }

    double rad2deg(double rad) const
    {
        return rad * 180.0 / M_PI;
    }

    double wrapAngleRad(double a) const
    {
        while (a > M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }

    bool updateRobotPoseInMap()
    {
        try
        {
            geometry_msgs::TransformStamped tf =
                tf_buffer_.lookupTransform(map_frame_, base_frame_, ros::Time(0), ros::Duration(0.03));

            robot_map_x_ = tf.transform.translation.x;
            robot_map_y_ = tf.transform.translation.y;

            const double qx = tf.transform.rotation.x;
            const double qy = tf.transform.rotation.y;
            const double qz = tf.transform.rotation.z;
            const double qw = tf.transform.rotation.w;

            const double siny_cosp = 2.0 * (qw * qz + qx * qy);
            const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
            robot_map_yaw_ = std::atan2(siny_cosp, cosy_cosp);

            has_pose_in_map_ = true;
            return true;
        }
        catch (const tf2::TransformException&)
        {
            has_pose_in_map_ = false;
            return false;
        }
    }

    std::vector<double> collectSectorRanges(double deg_min, double deg_max) const
    {
        std::vector<double> vals;
        if (!has_scan_ || latest_scan_.ranges.empty())
            return vals;

        const double a_min = deg2rad(deg_min);
        const double a_max = deg2rad(deg_max);

        for (size_t i = 0; i < latest_scan_.ranges.size(); ++i)
        {
            const double a = latest_scan_.angle_min + static_cast<double>(i) * latest_scan_.angle_increment;
            if (a < a_min || a > a_max)
                continue;

            double r = latest_scan_.ranges[i];
            if (std::isnan(r) || std::isinf(r))
                continue;
            if (r < latest_scan_.range_min || r > latest_scan_.range_max)
                continue;

            // neighbor-based spike rejection
            if (i > 0 && i + 1 < latest_scan_.ranges.size())
            {
                double l = latest_scan_.ranges[i - 1];
                double rr = latest_scan_.ranges[i + 1];
                bool l_ok = std::isfinite(l) && !std::isnan(l);
                bool r_ok = std::isfinite(rr) && !std::isnan(rr);

                if (l_ok && r_ok)
                {
                    double neigh = 0.5 * (l + rr);
                    if (std::fabs(r - neigh) > laser_neighbor_reject_jump_ && r < neigh)
                        continue;
                }
            }

            vals.push_back(r);
        }

        return vals;
    }

    double robustSectorDistance(double deg_min, double deg_max) const
    {
        std::vector<double> vals = collectSectorRanges(deg_min, deg_max);
        if (vals.empty())
            return std::numeric_limits<double>::infinity();

        std::sort(vals.begin(), vals.end());
        size_t idx = static_cast<size_t>(clamp(laser_sector_quantile_, 0.0, 1.0) * static_cast<double>(vals.size() - 1));
        return vals[idx];
    }

    double headingScanClearanceDeg(double heading_deg, double half_width_deg) const
    {
        return robustSectorDistance(heading_deg - half_width_deg, heading_deg + half_width_deg);
    }

    SectorInfo readSectors() const
    {
        SectorInfo s;
        s.front = robustSectorDistance(-front_sector_deg_, front_sector_deg_);
        s.front_wide = robustSectorDistance(-front_wide_sector_deg_, front_wide_sector_deg_);
        s.front_right = robustSectorDistance(side_sector_min_deg_, side_sector_max_deg_);
        s.front_left = robustSectorDistance(-side_sector_max_deg_, -side_sector_min_deg_);
        s.right = robustSectorDistance(70.0, 100.0);
        s.left = robustSectorDistance(-100.0, -70.0);
        s.right_diag = robustSectorDistance(35.0, 65.0);
        s.left_diag = robustSectorDistance(-65.0, -35.0);

        s.rear = robustSectorDistance(180.0 - rear_sector_deg_, 180.0);
        const double rear2 = robustSectorDistance(-180.0, -180.0 + rear_sector_deg_);
        if (rear2 < s.rear)
            s.rear = rear2;

        s.rear_left = robustSectorDistance(110.0, 160.0);
        s.rear_right = robustSectorDistance(-160.0, -110.0);
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

    void setTimedRecoveryState(State s, double sec)
    {
        state_ = s;
        state_end_time_ = ros::Time::now() + ros::Duration(sec);
    }

    bool timedStateFinished() const
    {
        return ros::Time::now() >= state_end_time_;
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
        center_error = s.left - s.right;

        // slightly stronger corridor validation
        const bool roughly_parallel =
            std::isfinite(s.front_left) && std::isfinite(s.front_right) &&
            std::fabs(s.front_left - s.left) < 0.35 &&
            std::fabs(s.front_right - s.right) < 0.35;

        return corridor_width <= corridor_width_threshold_ && roughly_parallel;
    }

    void choosePreferredTurn(const SectorInfo& s)
    {
        double left_score = 0.0;
        double right_score = 0.0;

        if (std::isfinite(s.front_left)) left_score += 1.6 * s.front_left;
        if (std::isfinite(s.left_diag))  left_score += 1.0 * s.left_diag;
        if (std::isfinite(s.left))       left_score += 0.9 * s.left;
        if (std::isfinite(s.rear_left))  left_score += 0.4 * s.rear_left;

        if (std::isfinite(s.front_right)) right_score += 1.6 * s.front_right;
        if (std::isfinite(s.right_diag))  right_score += 1.0 * s.right_diag;
        if (std::isfinite(s.right))       right_score += 0.9 * s.right;
        if (std::isfinite(s.rear_right))  right_score += 0.4 * s.rear_right;

        if (std::fabs(left_score - right_score) < 0.10)
            preferred_left_ = !preferred_left_;
        else
            preferred_left_ = left_score > right_score;
    }

    bool worldToMap(double wx, double wy, int& mx, int& my) const
    {
        if (!has_map_)
            return false;

        const double origin_x = latest_map_.info.origin.position.x;
        const double origin_y = latest_map_.info.origin.position.y;
        const double res = latest_map_.info.resolution;

        mx = static_cast<int>(std::floor((wx - origin_x) / res));
        my = static_cast<int>(std::floor((wy - origin_y) / res));

        if (mx < 0 || my < 0)
            return false;
        if (mx >= static_cast<int>(latest_map_.info.width) || my >= static_cast<int>(latest_map_.info.height))
            return false;

        return true;
    }

    void mapToWorld(int mx, int my, double& wx, double& wy) const
    {
        const double origin_x = latest_map_.info.origin.position.x;
        const double origin_y = latest_map_.info.origin.position.y;
        const double res = latest_map_.info.resolution;

        wx = origin_x + (static_cast<double>(mx) + 0.5) * res;
        wy = origin_y + (static_cast<double>(my) + 0.5) * res;
    }

    int gridIndex(int mx, int my) const
    {
        return my * static_cast<int>(latest_map_.info.width) + mx;
    }

    bool validCell(int mx, int my) const
    {
        return mx >= 0 && my >= 0 &&
               mx < static_cast<int>(latest_map_.info.width) &&
               my < static_cast<int>(latest_map_.info.height);
    }

    int cellValue(int mx, int my) const
    {
        if (!validCell(mx, my))
            return 100;
        return latest_map_.data[gridIndex(mx, my)];
    }

    bool isFreeCell(int mx, int my) const
    {
        return cellValue(mx, my) == 0;
    }

    bool isUnknownCell(int mx, int my) const
    {
        return cellValue(mx, my) == -1;
    }

    bool isOccupiedCell(int mx, int my) const
    {
        const int v = cellValue(mx, my);
        return v >= 70;
    }

    bool hasUnknownNeighbor(int mx, int my) const
    {
        for (int dy = -1; dy <= 1; ++dy)
        {
            for (int dx = -1; dx <= 1; ++dx)
            {
                if (dx == 0 && dy == 0)
                    continue;

                const int nx = mx + dx;
                const int ny = my + dy;
                if (!validCell(nx, ny))
                    continue;

                if (isUnknownCell(nx, ny))
                    return true;
            }
        }
        return false;
    }

    bool isFrontierCell(int mx, int my) const
    {
        return isFreeCell(mx, my) && hasUnknownNeighbor(mx, my);
    }

    double visitValue(int mx, int my) const
    {
        if (!validCell(mx, my))
            return visit_penalty_cap_;

        const int idx = gridIndex(mx, my);
        if (idx < 0 || idx >= static_cast<int>(visit_counts_.size()))
            return visit_penalty_cap_;

        return visit_counts_[idx];
    }

    void updateVisitMemory()
    {
        if (!has_map_ || !has_pose_in_map_)
            return;

        const ros::Time now = ros::Time::now();
        if ((now - last_visit_update_time_).toSec() < 0.35)
            return;

        int mx, my;
        if (!worldToMap(robot_map_x_, robot_map_y_, mx, my))
            return;

        const int radius_cells =
            std::max(1, static_cast<int>(std::round(visit_update_radius_m_ / latest_map_.info.resolution)));

        for (int dy = -radius_cells; dy <= radius_cells; ++dy)
        {
            for (int dx = -radius_cells; dx <= radius_cells; ++dx)
            {
                const int nx = mx + dx;
                const int ny = my + dy;
                if (!validCell(nx, ny))
                    continue;

                const double dist = std::sqrt(static_cast<double>(dx * dx + dy * dy));
                if (dist > radius_cells)
                    continue;

                const int idx = gridIndex(nx, ny);
                if (latest_map_.data[idx] == 0)
                    visit_counts_[idx] = std::min(visit_penalty_cap_, visit_counts_[idx] + 1.0);
            }
        }

        last_visit_update_time_ = now;
    }

    bool areaIsTraversable(int mx, int my, int radius_cells) const
    {
        for (int dy = -radius_cells; dy <= radius_cells; ++dy)
        {
            for (int dx = -radius_cells; dx <= radius_cells; ++dx)
            {
                const int nx = mx + dx;
                const int ny = my + dy;
                if (!validCell(nx, ny))
                    return false;

                const double dist = std::sqrt(static_cast<double>(dx * dx + dy * dy));
                if (dist > radius_cells)
                    continue;

                if (isOccupiedCell(nx, ny))
                    return false;
            }
        }
        return true;
    }

    bool lineOfSightFree(double wx0, double wy0, double wx1, double wy1, double clearance_m) const
    {
        if (!has_map_)
            return false;

        const double dx = wx1 - wx0;
        const double dy = wy1 - wy0;
        const double dist = std::hypot(dx, dy);
        if (dist < 1e-6)
            return true;

        const double step = std::max(0.03, latest_map_.info.resolution * 0.7);
        const int clearance_cells =
            std::max(1, static_cast<int>(std::round(clearance_m / latest_map_.info.resolution)));

        for (double d = 0.0; d <= dist; d += step)
        {
            const double t = d / dist;
            const double wx = wx0 + t * dx;
            const double wy = wy0 + t * dy;

            int mx, my;
            if (!worldToMap(wx, wy, mx, my))
                return false;

            if (!areaIsTraversable(mx, my, clearance_cells))
                return false;
        }

        return true;
    }

    bool isGoalBlacklisted(double gx, double gy)
    {
        const ros::Time now = ros::Time::now();

        std::vector<BlacklistedGoal> filtered;
        filtered.reserve(goal_blacklist_.size());
        for (size_t i = 0; i < goal_blacklist_.size(); ++i)
        {
            if (goal_blacklist_[i].until > now)
                filtered.push_back(goal_blacklist_[i]);
        }
        goal_blacklist_.swap(filtered);

        for (size_t i = 0; i < goal_blacklist_.size(); ++i)
        {
            const double d = std::hypot(goal_blacklist_[i].x - gx, goal_blacklist_[i].y - gy);
            if (d <= goal_blacklist_radius_m_)
                return true;
        }
        return false;
    }

    void addGoalToBlacklist(double gx, double gy)
    {
        BlacklistedGoal b;
        b.x = gx;
        b.y = gy;
        b.until = ros::Time::now() + ros::Duration(goal_blacklist_duration_sec_);
        goal_blacklist_.push_back(b);

        ROS_WARN_STREAM("Blacklisted failed goal near (" << gx << ", " << gy
                        << ") for " << goal_blacklist_duration_sec_ << " sec");
    }

    double estimateUnknownDensityAroundFrontierCell(int mx, int my) const
    {
        int unknown_count = 0;
        int total = 0;
        for (int dy = -2; dy <= 2; ++dy)
        {
            for (int dx = -2; dx <= 2; ++dx)
            {
                const int nx = mx + dx;
                const int ny = my + dy;
                if (!validCell(nx, ny))
                    continue;

                ++total;
                if (isUnknownCell(nx, ny))
                    ++unknown_count;
            }
        }

        if (total == 0)
            return 0.0;

        return static_cast<double>(unknown_count) / static_cast<double>(total);
    }

    std::vector<FrontierCluster> extractFrontierClusters()
    {
        std::vector<FrontierCluster> clusters;
        if (!has_map_ || !has_pose_in_map_)
            return clusters;

        const int width = static_cast<int>(latest_map_.info.width);
        const int height = static_cast<int>(latest_map_.info.height);
        const int total = width * height;

        std::vector<unsigned char> frontier_mask(total, 0);
        std::vector<unsigned char> visited(total, 0);

        for (int my = 1; my < height - 1; ++my)
        {
            for (int mx = 1; mx < width - 1; ++mx)
            {
                const int idx = gridIndex(mx, my);
                if (isFrontierCell(mx, my))
                    frontier_mask[idx] = 1;
            }
        }

        for (int my = 1; my < height - 1; ++my)
        {
            for (int mx = 1; mx < width - 1; ++mx)
            {
                const int start_idx = gridIndex(mx, my);
                if (!frontier_mask[start_idx] || visited[start_idx])
                    continue;

                std::queue<std::pair<int, int> > q;
                FrontierCluster cluster;
                cluster.centroid_x = 0.0;
                cluster.centroid_y = 0.0;
                cluster.nearest_x = 0.0;
                cluster.nearest_y = 0.0;
                cluster.unknown_density_score = 0.0;
                cluster.width_score = 0.0;
                cluster.score = -1e9;
                cluster.distance_to_robot = 1e9;
                cluster.heading_error_deg = 0.0;
                cluster.visit_penalty = 0.0;

                q.push(std::make_pair(mx, my));
                visited[start_idx] = 1;

                while (!q.empty())
                {
                    std::pair<int, int> cur = q.front();
                    q.pop();

                    const int cx = cur.first;
                    const int cy = cur.second;
                    const int cidx = gridIndex(cx, cy);

                    cluster.cells.push_back(cidx);

                    double wx, wy;
                    mapToWorld(cx, cy, wx, wy);
                    cluster.centroid_x += wx;
                    cluster.centroid_y += wy;

                    const double dist = std::hypot(wx - robot_map_x_, wy - robot_map_y_);
                    if (dist < cluster.distance_to_robot)
                    {
                        cluster.distance_to_robot = dist;
                        cluster.nearest_x = wx;
                        cluster.nearest_y = wy;
                    }

                    cluster.visit_penalty += visitValue(cx, cy);
                    cluster.unknown_density_score += estimateUnknownDensityAroundFrontierCell(cx, cy);

                    // width proxy: free clearance around frontier
                    if (areaIsTraversable(cx, cy, 1))
                        cluster.width_score += 1.0;

                    for (int dy = -1; dy <= 1; ++dy)
                    {
                        for (int dx = -1; dx <= 1; ++dx)
                        {
                            if (dx == 0 && dy == 0)
                                continue;

                            const int nx = cx + dx;
                            const int ny = cy + dy;
                            if (!validCell(nx, ny))
                                continue;

                            const int nidx = gridIndex(nx, ny);
                            if (!frontier_mask[nidx] || visited[nidx])
                                continue;

                            visited[nidx] = 1;
                            q.push(std::make_pair(nx, ny));
                        }
                    }
                }

                if (static_cast<int>(cluster.cells.size()) < frontier_min_cluster_size_)
                    continue;

                const double denom = static_cast<double>(cluster.cells.size());
                cluster.centroid_x /= denom;
                cluster.centroid_y /= denom;
                cluster.visit_penalty /= denom;
                cluster.unknown_density_score /= denom;
                cluster.width_score /= denom;

                clusters.push_back(cluster);
            }
        }

        return clusters;
    }

    bool computeGoalForCluster(FrontierCluster& cluster, double& goal_x, double& goal_y)
    {
        if (!has_map_ || !has_pose_in_map_)
            return false;

        const double dx = cluster.centroid_x - robot_map_x_;
        const double dy = cluster.centroid_y - robot_map_y_;
        const double dist = std::hypot(dx, dy);

        if (dist < 1e-6)
            return false;

        const double ux = dx / dist;
        const double uy = dy / dist;

        goal_x = cluster.centroid_x - frontier_goal_pullback_m_ * ux;
        goal_y = cluster.centroid_y - frontier_goal_pullback_m_ * uy;

        int mx, my;
        if (!worldToMap(goal_x, goal_y, mx, my))
            return false;

        const int clearance_cells =
            std::max(1, static_cast<int>(std::round(frontier_robot_clearance_m_ / latest_map_.info.resolution)));

        if (!isFreeCell(mx, my))
            return false;

        if (!areaIsTraversable(mx, my, clearance_cells))
            return false;

        if (!lineOfSightFree(robot_map_x_, robot_map_y_, goal_x, goal_y, frontier_robot_clearance_m_))
            return false;

        if (isGoalBlacklisted(goal_x, goal_y))
            return false;

        return true;
    }

    bool selectBestFrontierGoal(double& goal_x, double& goal_y, double& heading_deg_out)
    {
        if (!has_map_ || !has_pose_in_map_)
            return false;

        std::vector<FrontierCluster> clusters = extractFrontierClusters();
        if (clusters.empty())
            return false;

        FrontierCluster best;
        best.score = -1e9;
        bool found = false;

        for (size_t i = 0; i < clusters.size(); ++i)
        {
            double gx, gy;
            if (!computeGoalForCluster(clusters[i], gx, gy))
                continue;

            const double dx = gx - robot_map_x_;
            const double dy = gy - robot_map_y_;
            const double dist = std::hypot(dx, dy);
            if (dist < 1e-4)
                continue;

            const double target_yaw = std::atan2(dy, dx);
            const double heading_err = wrapAngleRad(target_yaw - robot_map_yaw_);
            const double heading_err_deg = rad2deg(heading_err);

            // dynamic obstacle suppression
            const double scan_clear = headingScanClearanceDeg(heading_err_deg, 12.0);
            if (std::isfinite(scan_clear) && scan_clear < front_block_dist_)
                continue;

            const double size_term =
                frontier_size_weight_ * std::log(1.0 + static_cast<double>(clusters[i].cells.size()));
            const double dist_term = frontier_distance_weight_ * dist;
            const double heading_term =
                frontier_heading_weight_ * (1.0 - clamp(std::fabs(heading_err_deg) / 140.0, 0.0, 1.0));
            const double visit_term = frontier_visit_penalty_weight_ * clusters[i].visit_penalty;
            const double unknown_term = frontier_unknown_density_weight_ * clusters[i].unknown_density_score;
            const double narrow_penalty =
                frontier_narrow_penalty_weight_ * (1.0 - clamp(clusters[i].width_score, 0.0, 1.0));

            double same_goal_bonus = 0.0;
            if (last_goal_valid_)
            {
                const double dg = std::hypot(gx - last_goal_x_, gy - last_goal_y_);
                if (dg < 0.50)
                    same_goal_bonus = frontier_same_goal_bonus_;
            }

            clusters[i].score =
                size_term - dist_term + heading_term + unknown_term + same_goal_bonus
                - visit_term - narrow_penalty;
            clusters[i].heading_error_deg = heading_err_deg;
            clusters[i].distance_to_robot = dist;

            if (!found || clusters[i].score > best.score)
            {
                best = clusters[i];
                goal_x = gx;
                goal_y = gy;
                heading_deg_out = heading_err_deg;
                found = true;
            }
        }

        if (!found)
            return false;

        ROS_INFO_THROTTLE(1.0,
                          "Frontier selected: score=%.2f dist=%.2f heading=%.1f cells=%zu visit_pen=%.2f unk=%.2f width=%.2f",
                          best.score, best.distance_to_robot, best.heading_error_deg,
                          best.cells.size(), best.visit_penalty,
                          best.unknown_density_score, best.width_score);

        return true;
    }

    bool computeLocalWaypoint(double goal_x, double goal_y,
                              double& wx_out, double& wy_out, double& heading_deg_out)
    {
        const double dx = goal_x - robot_map_x_;
        const double dy = goal_y - robot_map_y_;
        const double dist = std::hypot(dx, dy);
        if (dist < 1e-6)
            return false;

        const double ux = dx / dist;
        const double uy = dy / dist;

        const double max_look = std::min(waypoint_max_lookahead_m_, dist);
        double chosen_x = goal_x;
        double chosen_y = goal_y;
        bool found = false;

        for (double d = waypoint_step_m_; d <= max_look + 1e-6; d += waypoint_step_m_)
        {
            const double tx = robot_map_x_ + d * ux;
            const double ty = robot_map_y_ + d * uy;

            int mx, my;
            if (!worldToMap(tx, ty, mx, my))
                break;

            if (!isFreeCell(mx, my))
                break;

            if (!lineOfSightFree(robot_map_x_, robot_map_y_, tx, ty, frontier_robot_clearance_m_))
                break;

            chosen_x = tx;
            chosen_y = ty;
            found = true;
        }

        if (!found)
        {
            if (!lineOfSightFree(robot_map_x_, robot_map_y_, goal_x, goal_y, frontier_robot_clearance_m_))
                return false;

            chosen_x = goal_x;
            chosen_y = goal_y;
        }

        const double hx = chosen_x - robot_map_x_;
        const double hy = chosen_y - robot_map_y_;
        const double target_yaw = std::atan2(hy, hx);
        heading_deg_out = rad2deg(wrapAngleRad(target_yaw - robot_map_yaw_));

        wx_out = chosen_x;
        wy_out = chosen_y;
        return true;
    }

    void startGoalProgressTracking(double gx, double gy)
    {
        goal_progress_tracking_ = true;
        goal_track_start_time_ = ros::Time::now();
        goal_track_start_dist_ = std::hypot(gx - robot_map_x_, gy - robot_map_y_);
        tracked_goal_x_ = gx;
        tracked_goal_y_ = gy;
    }

    bool goalProgressFailed()
    {
        if (!goal_progress_tracking_)
            return false;

        const double elapsed = (ros::Time::now() - goal_track_start_time_).toSec();
        if (elapsed < goal_progress_timeout_sec_)
            return false;

        const double now_dist = std::hypot(tracked_goal_x_ - robot_map_x_, tracked_goal_y_ - robot_map_y_);
        const double progress = goal_track_start_dist_ - now_dist;
        return progress < goal_progress_min_dist_m_;
    }

    void handleGoalFailureIfNeeded()
    {
        if (!last_goal_valid_ || !has_pose_in_map_)
            return;

        const double dist_to_goal = std::hypot(last_goal_x_ - robot_map_x_, last_goal_y_ - robot_map_y_);
        if (dist_to_goal <= goal_reach_dist_m_)
        {
            goal_fail_count_ = 0;
            goal_progress_tracking_ = false;
            last_goal_valid_ = false;
            state_ = SELECT_GOAL;
            ROS_INFO("Frontier goal reached, selecting a new goal.");
            return;
        }

        if (!goal_progress_tracking_)
            startGoalProgressTracking(last_goal_x_, last_goal_y_);

        if (goalProgressFailed())
        {
            goal_fail_count_++;
            goal_progress_tracking_ = false;
            ROS_WARN_STREAM("Goal progress failed. Count=" << goal_fail_count_);

            if (goal_fail_count_ >= goal_fail_limit_)
            {
                addGoalToBlacklist(last_goal_x_, last_goal_y_);
                goal_fail_count_ = 0;
                last_goal_valid_ = false;
            }
            else
            {
                last_goal_valid_ = false;
            }

            state_ = SELECT_GOAL;
        }
    }

    double computeBaseForwardSpeed(const SectorInfo& s, double heading_deg) const
    {
        double base_speed;

        if (!std::isfinite(s.front))
            base_speed = forward_speed_max_;
        else if (s.front <= front_block_dist_)
            base_speed = 0.0;
        else if (s.front >= front_clear_dist_)
            base_speed = forward_speed_max_;
        else if (s.front <= front_caution_dist_)
        {
            const double ratio = clamp((s.front - front_block_dist_) /
                                       std::max(0.001, front_caution_dist_ - front_block_dist_), 0.0, 1.0);
            base_speed = forward_speed_min_ + ratio * (forward_speed_recovery_ - forward_speed_min_);
        }
        else
        {
            const double ratio = clamp((s.front - front_caution_dist_) /
                                       std::max(0.001, front_clear_dist_ - front_caution_dist_), 0.0, 1.0);
            base_speed = forward_speed_recovery_ + ratio * (forward_speed_max_ - forward_speed_recovery_);
        }

        if (in_corridor_)
        {
            if (std::isfinite(s.front) && s.front < corridor_front_slow_dist_)
            {
                const double ratio = clamp((s.front - front_block_dist_) /
                                           std::max(0.001, corridor_front_slow_dist_ - front_block_dist_), 0.0, 1.0);
                base_speed = forward_speed_min_ + ratio * (corridor_speed_max_ - forward_speed_min_);
            }
            else
            {
                base_speed = std::min(base_speed, corridor_speed_max_);
            }
        }

        const double heading_penalty = clamp(std::fabs(heading_deg) / (in_corridor_ ? 60.0 : 95.0), 0.0, 1.0);
        const double speed_scale = clamp(1.0 - 0.40 * heading_penalty, 0.60, 1.0);
        return clamp(base_speed * speed_scale, forward_speed_min_, forward_speed_max_);
    }

    bool trajectoryEndpointTraversable(double x0, double y0, double yaw0,
                                      double v, double w,
                                      double& xf, double& yf, double& yawf) const
    {
        xf = x0;
        yf = y0;
        yawf = yaw0;

        for (double t = 0.0; t < traj_sim_time_sec_; t += traj_dt_sec_)
        {
            yawf = wrapAngleRad(yawf + w * traj_dt_sec_);
            xf += v * std::cos(yawf) * traj_dt_sec_;
            yf += v * std::sin(yawf) * traj_dt_sec_;

            int mx, my;
            if (!worldToMap(xf, yf, mx, my))
                return false;

            const int clearance_cells =
                std::max(1, static_cast<int>(std::round(frontier_robot_clearance_m_ / latest_map_.info.resolution)));

            if (!areaIsTraversable(mx, my, clearance_cells))
                return false;
        }

        return true;
    }

    TrajectoryCandidate chooseBestTrajectory(const SectorInfo& s,
                                             double waypoint_x, double waypoint_y,
                                             double center_error)
    {
        std::vector<TrajectoryCandidate> candidates;
        const double desired_heading =
            rad2deg(wrapAngleRad(std::atan2(waypoint_y - robot_map_y_, waypoint_x - robot_map_x_) - robot_map_yaw_));

        const double v_forward = computeBaseForwardSpeed(s, desired_heading);
        const bool rear_blocked = std::isfinite(s.rear) && s.rear < rear_clear_dist_;

        // candidate set
        std::vector<double> v_set;
        v_set.push_back(v_forward);
        v_set.push_back(std::max(forward_speed_min_, 0.75 * v_forward));
        if (!rear_blocked && ros::Time::now() >= reverse_cooldown_until_)
            v_set.push_back(reverse_speed_);

        std::vector<double> w_set;
        w_set.push_back(-turn_speed_soft_);
        w_set.push_back(-0.5 * turn_speed_soft_);
        w_set.push_back(0.0);
        w_set.push_back(0.5 * turn_speed_soft_);
        w_set.push_back(turn_speed_soft_);

        if (std::fabs(desired_heading) > 25.0)
        {
            w_set.push_back(-turn_speed_hard_);
            w_set.push_back(turn_speed_hard_);
        }

        TrajectoryCandidate best;
        best.score = -1e9;
        best.v = 0.0;
        best.w = 0.0;
        best.final_x = robot_map_x_;
        best.final_y = robot_map_y_;
        best.final_yaw = robot_map_yaw_;

        for (size_t iv = 0; iv < v_set.size(); ++iv)
        {
            for (size_t iw = 0; iw < w_set.size(); ++iw)
            {
                const double v = v_set[iv];
                const double w = w_set[iw];

                // do not consider reverse if rear blocked
                if (v < 0.0 && rear_blocked)
                    continue;

                double xf, yf, yawf;
                if (!trajectoryEndpointTraversable(robot_map_x_, robot_map_y_, robot_map_yaw_, v, w, xf, yf, yawf))
                    continue;

                const double dist_before = std::hypot(waypoint_x - robot_map_x_, waypoint_y - robot_map_y_);
                const double dist_after = std::hypot(waypoint_x - xf, waypoint_y - yf);
                const double progress = dist_before - dist_after;

                const double final_heading_err =
                    std::fabs(rad2deg(wrapAngleRad(std::atan2(waypoint_y - yf, waypoint_x - xf) - yawf)));

                int mx, my;
                double visit_pen = 0.0;
                if (worldToMap(xf, yf, mx, my))
                    visit_pen = visitValue(mx, my);

                // dynamic obstacle suppression with scan
                const double heading_sector =
                    rad2deg(wrapAngleRad(std::atan2(yf - robot_map_y_, xf - robot_map_x_) - robot_map_yaw_));
                const double scan_clear = headingScanClearanceDeg(heading_sector, 10.0);

                double scan_clear_norm = 1.0;
                if (std::isfinite(scan_clear))
                    scan_clear_norm = clamp(scan_clear / front_clear_dist_, 0.0, 1.2);

                double smoothness = 1.0 - clamp(std::fabs(w) / std::max(0.001, turn_speed_hard_), 0.0, 1.0);

                double corridor_bonus = 0.0;
                if (in_corridor_ && v > 0.0)
                {
                    corridor_bonus =
                        1.0 - clamp(std::fabs(center_error) / std::max(0.001, corridor_width_threshold_), 0.0, 1.0);
                }

                double reverse_penalty = (v < 0.0) ? 1.4 : 0.0;

                double score =
                    traj_goal_weight_ * (1.0 - clamp(final_heading_err / 120.0, 0.0, 1.0)) +
                    traj_progress_weight_ * progress +
                    traj_clearance_weight_ * scan_clear_norm +
                    traj_smoothness_weight_ * smoothness +
                    0.45 * corridor_bonus -
                    traj_visit_weight_ * visit_pen -
                    reverse_penalty;

                if (v > 0.0 && std::isfinite(s.rear) && s.rear < rear_clear_dist_)
                    score += 0.3; // prefer forward if rear has obstacle

                if (score > best.score)
                {
                    best.score = score;
                    best.v = v;
                    best.w = w;
                    best.final_x = xf;
                    best.final_y = yf;
                    best.final_yaw = yawf;
                }
            }
        }

        return best;
    }

    void selectGoalIfNeeded()
    {
        const ros::Time now = ros::Time::now();
        const bool time_to_reselect =
            last_frontier_select_time_.isZero() ||
            (now - last_frontier_select_time_).toSec() >= frontier_reselect_period_sec_;

        if (!time_to_reselect && last_goal_valid_)
            return;

        double gx = 0.0, gy = 0.0, heading_deg = 0.0;
        if (selectBestFrontierGoal(gx, gy, heading_deg))
        {
            last_goal_x_ = gx;
            last_goal_y_ = gy;
            last_heading_deg_ = heading_deg;
            last_goal_valid_ = true;
            last_frontier_select_time_ = now;
            goal_progress_tracking_ = false;
            state_ = DRIVE_TO_WAYPOINT;
            ROS_INFO_STREAM("Selected new frontier goal at (" << gx << ", " << gy << ")");
        }
        else
        {
            last_goal_valid_ = false;
        }
    }

    void beginRecovery(const SectorInfo& s, bool emergency)
    {
        recovery_count_++;
        choosePreferredTurn(s);

        const bool rear_safe = std::isfinite(s.rear) && s.rear > rear_clear_dist_;
        const bool rear_emergency = std::isfinite(s.rear) && s.rear < rear_emergency_dist_;
        const bool front_emergency = std::isfinite(s.front) && s.front < front_emergency_dist_;

        // Recovery arbitration:
        // 1) if rear blocked -> never reverse
        if (!rear_safe || rear_emergency)
        {
            state_ = RECOVERY_TURN;
            setTimedRecoveryState(preferred_left_ ? RECOVERY_TURN : RECOVERY_TURN, turn_duration_sec_);
            recovery_turn_left_ = preferred_left_;
            resetProgressTracking();
            return;
        }

        // 2) corridor -> prefer turn only
        if (in_corridor_)
        {
            state_ = RECOVERY_TURN;
            setTimedRecoveryState(preferred_left_ ? RECOVERY_TURN : RECOVERY_TURN, turn_duration_sec_);
            recovery_turn_left_ = preferred_left_;
            resetProgressTracking();
            return;
        }

        // 3) side escape if open
        const double left_space = finiteOr(s.front_left, 2.0) + 0.5 * finiteOr(s.left_diag, 1.0);
        const double right_space = finiteOr(s.front_right, 2.0) + 0.5 * finiteOr(s.right_diag, 1.0);
        const bool strong_side_escape = std::max(left_space, right_space) > 1.05;

        if (strong_side_escape && !front_emergency)
        {
            recovery_turn_left_ = left_space >= right_space;
            state_ = RECOVERY_TURN;
            setTimedRecoveryState(RECOVERY_TURN, turn_duration_sec_);
            resetProgressTracking();
            return;
        }

        // 4) reverse only if allowed
        if (ros::Time::now() >= reverse_cooldown_until_ &&
            reverse_chain_count_ < max_reverse_chain_ &&
            rear_safe)
        {
            recovery_turn_left_ = preferred_left_;
            state_ = RECOVERY_REVERSE;
            setTimedRecoveryState(RECOVERY_REVERSE, reverse_duration_sec_);
            reverse_chain_count_++;
            reverse_cooldown_until_ = ros::Time::now() + ros::Duration(reverse_cooldown_sec_);
            resetProgressTracking();
            return;
        }

        // 5) final escape
        recovery_turn_left_ = preferred_left_;
        state_ = RECOVERY_ESCAPE;
        setTimedRecoveryState(RECOVERY_ESCAPE, escape_turn_duration_sec_);
        recovery_count_ = 0;
        reverse_chain_count_ = 0;
        resetProgressTracking();
    }

    geometry_msgs::Twist commandForRecoveryState() const
    {
        geometry_msgs::Twist cmd;

        switch (state_)
        {
            case RECOVERY_TURN:
                cmd.linear.x = 0.0;
                cmd.angular.z = recovery_turn_left_ ? turn_speed_hard_ : -turn_speed_hard_;
                break;

            case RECOVERY_REVERSE:
                cmd.linear.x = reverse_speed_;
                cmd.angular.z = recovery_turn_left_ ? reverse_turn_speed_ : -reverse_turn_speed_;
                break;

            case RECOVERY_ESCAPE:
                cmd.linear.x = forward_speed_min_;
                cmd.angular.z = recovery_turn_left_ ? turn_speed_hard_ : -turn_speed_hard_;
                break;

            default:
                break;
        }

        return cmd;
    }

    void advanceRecoveryState()
    {
        if (state_ == RECOVERY_REVERSE)
        {
            state_ = RECOVERY_TURN;
            setTimedRecoveryState(RECOVERY_TURN, turn_duration_sec_);
            return;
        }

        if (state_ == RECOVERY_TURN || state_ == RECOVERY_ESCAPE)
        {
            state_ = SELECT_GOAL;
            return;
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

        updateRobotPoseInMap();
        updateVisitMemory();
        handleGoalFailureIfNeeded();

        SectorInfo s = readSectors();

        double corridor_width = std::numeric_limits<double>::infinity();
        double center_error = 0.0;
        in_corridor_ = isCorridor(s, corridor_width, center_error);

        // Active recovery states
        if (state_ == RECOVERY_TURN || state_ == RECOVERY_REVERSE || state_ == RECOVERY_ESCAPE)
        {
            if (timedStateFinished())
                advanceRecoveryState();

            geometry_msgs::Twist cmd = commandForRecoveryState();
            cmd = safetyGate(cmd, s);
            cmd_pub_.publish(cmd);
            return;
        }

        // Safety-triggered recovery
        const bool front_emergency = std::isfinite(s.front) && s.front < front_emergency_dist_;
        const bool front_block = std::isfinite(s.front) && s.front < front_block_dist_;

        if (front_emergency)
        {
            beginRecovery(s, true);
            geometry_msgs::Twist cmd = commandForRecoveryState();
            cmd = safetyGate(cmd, s);
            cmd_pub_.publish(cmd);
            return;
        }

        if (front_block)
        {
            beginRecovery(s, false);
            geometry_msgs::Twist cmd = commandForRecoveryState();
            cmd = safetyGate(cmd, s);
            cmd_pub_.publish(cmd);
            return;
        }

        if (!progress_tracking_)
            startProgressTracking();

        if (stuckForward())
        {
            beginRecovery(s, false);
            geometry_msgs::Twist cmd = commandForRecoveryState();
            cmd = safetyGate(cmd, s);
            cmd_pub_.publish(cmd);
            return;
        }

        // Hierarchical planning
        if (state_ == SELECT_GOAL || !last_goal_valid_)
            selectGoalIfNeeded();

        geometry_msgs::Twist cmd;

        if (!last_goal_valid_)
        {
            choosePreferredTurn(s);
            const bool rear_blocked = std::isfinite(s.rear) && s.rear < rear_clear_dist_;
            cmd.linear.x = rear_blocked ? forward_speed_min_ : forward_speed_recovery_;
            cmd.angular.z = preferred_left_ ? 0.35 : -0.35;
            cmd = safetyGate(cmd, s);
            cmd_pub_.publish(cmd);
            return;
        }

        double waypoint_x = last_goal_x_;
        double waypoint_y = last_goal_y_;
        double heading_deg = 0.0;

        if (!computeLocalWaypoint(last_goal_x_, last_goal_y_, waypoint_x, waypoint_y, heading_deg))
        {
            last_goal_valid_ = false;
            state_ = SELECT_GOAL;
            choosePreferredTurn(s);
            cmd.linear.x = forward_speed_min_;
            cmd.angular.z = preferred_left_ ? 0.40 : -0.40;
            cmd = safetyGate(cmd, s);
            cmd_pub_.publish(cmd);
            return;
        }

        TrajectoryCandidate best = chooseBestTrajectory(s, waypoint_x, waypoint_y, center_error);

        if (best.score < -1e8)
        {
            beginRecovery(s, false);
            cmd = commandForRecoveryState();
            cmd = safetyGate(cmd, s);
            cmd_pub_.publish(cmd);
            return;
        }

        cmd.linear.x = best.v;
        cmd.angular.z = best.w;

        // corridor centering correction
        if (in_corridor_ && cmd.linear.x > 0.0)
        {
            cmd.angular.z += corridor_center_gain_ * center_error;
            if (std::isfinite(s.front_left) && std::isfinite(s.front_right))
                cmd.angular.z += corridor_heading_gain_ * (s.front_left - s.front_right);
            cmd.angular.z = clamp(cmd.angular.z, -turn_speed_soft_, turn_speed_soft_);
            cmd.linear.x = std::min(cmd.linear.x, corridor_speed_max_);
        }

        // dynamic obstacle suppression: do not be attracted to feet / short-lived close obstacles
        const double heading_clear = headingScanClearanceDeg(heading_deg, 10.0);
        if (std::isfinite(heading_clear) && heading_clear < front_block_dist_ && cmd.linear.x > 0.0)
        {
            cmd.linear.x = 0.0;
            cmd.angular.z = (preferred_left_ ? 1.0 : -1.0) * turn_speed_soft_;
        }

        // if rear blocked, bias forward only
        if (std::isfinite(s.rear) && s.rear < rear_clear_dist_ && cmd.linear.x < 0.0)
            cmd.linear.x = 0.0;

        cmd = safetyGate(cmd, s);
        cmd_pub_.publish(cmd);
        state_ = DRIVE_TO_WAYPOINT;

        if (has_odom_ && progress_tracking_)
        {
            const double dist = odomDistanceFrom(progress_start_x_, progress_start_y_);
            if (dist >= stuck_min_progress_)
            {
                startProgressTracking();
                if (std::isfinite(s.front) && s.front > front_caution_dist_)
                    reverse_chain_count_ = 0;
                if (std::isfinite(s.front_wide) && s.front_wide > front_caution_dist_)
                    recovery_count_ = 0;
            }
        }

        ROS_INFO_THROTTLE(1.0,
                          "state=%d front=%.2f rear=%.2f left=%.2f right=%.2f corridor=%s goal_valid=%s goal_fail=%d waypoint=(%.2f,%.2f) v=%.2f w=%.2f",
                          static_cast<int>(state_),
                          finiteOr(s.front, -1.0),
                          finiteOr(s.rear, -1.0),
                          finiteOr(s.left, -1.0),
                          finiteOr(s.right, -1.0),
                          in_corridor_ ? "yes" : "no",
                          last_goal_valid_ ? "yes" : "no",
                          goal_fail_count_,
                          waypoint_x, waypoint_y,
                          cmd.linear.x, cmd.angular.z);
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    ros::Subscriber scan_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber map_sub_;
    ros::Subscriber done_sub_;
    ros::Publisher cmd_pub_;
    ros::Timer timer_;

    sensor_msgs::LaserScan latest_scan_;
    nav_msgs::Odometry latest_odom_;
    nav_msgs::OccupancyGrid latest_map_;

    bool has_scan_;
    bool has_odom_;
    bool has_map_;
    bool has_pose_in_map_;
    bool exploration_done_;
    bool preferred_left_;
    bool in_corridor_;
    bool progress_tracking_;
    bool last_goal_valid_;
    bool goal_progress_tracking_;
    bool recovery_turn_left_;

    State state_;
    ros::Time state_end_time_;
    ros::Time reverse_cooldown_until_;
    ros::Time last_visit_update_time_;
    ros::Time last_frontier_select_time_;
    ros::Time progress_start_time_;
    ros::Time goal_track_start_time_;

    double progress_start_x_;
    double progress_start_y_;
    double robot_map_x_;
    double robot_map_y_;
    double robot_map_yaw_;
    double last_goal_x_;
    double last_goal_y_;
    double last_heading_deg_;
    double goal_track_start_dist_;
    double tracked_goal_x_;
    double tracked_goal_y_;

    int recovery_count_;
    int reverse_chain_count_;
    int goal_fail_count_ = 0;

    std::vector<double> visit_counts_;
    std::vector<BlacklistedGoal> goal_blacklist_;

    double control_rate_hz_;

    double front_clear_dist_;
    double front_caution_dist_;
    double front_block_dist_;
    double front_emergency_dist_;

    double rear_clear_dist_;
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
    double recovery_forward_duration_sec_;

    double stuck_timeout_sec_;
    double stuck_min_progress_;
    int max_recovery_attempts_;
    int max_reverse_chain_;
    double reverse_cooldown_sec_;

    double front_sector_deg_;
    double front_wide_sector_deg_;
    double side_sector_min_deg_;
    double side_sector_max_deg_;
    double rear_sector_deg_;

    double steering_deadband_;
    double corridor_width_threshold_;
    double corridor_wall_presence_max_;
    double corridor_center_gain_;
    double corridor_heading_gain_;
    double corridor_front_slow_dist_;

    int frontier_min_cluster_size_;
    double frontier_robot_clearance_m_;
    double frontier_goal_pullback_m_;
    double frontier_reselect_period_sec_;
    double frontier_size_weight_;
    double frontier_distance_weight_;
    double frontier_heading_weight_;
    double frontier_visit_penalty_weight_;
    double frontier_unknown_density_weight_;
    double frontier_same_goal_bonus_;
    double frontier_narrow_penalty_weight_;

    double goal_reach_dist_m_;
    double goal_progress_timeout_sec_;
    double goal_progress_min_dist_m_;
    int goal_fail_limit_;
    double goal_blacklist_radius_m_;
    double goal_blacklist_duration_sec_;

    double waypoint_step_m_;
    double waypoint_max_lookahead_m_;

    double visit_update_radius_m_;
    double visit_penalty_cap_;

    double traj_sim_time_sec_;
    double traj_dt_sec_;
    double traj_goal_weight_;
    double traj_clearance_weight_;
    double traj_smoothness_weight_;
    double traj_visit_weight_;
    double traj_progress_weight_;

    double laser_sector_quantile_;
    double laser_neighbor_reject_jump_;

    std::string scan_topic_;
    std::string odom_topic_;
    std::string map_topic_;
    std::string cmd_topic_;
    std::string done_topic_;
    std::string map_frame_;
    std::string base_frame_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "auto_explorer");
    AutoExplorer node;
    ros::spin();
    return 0;
}