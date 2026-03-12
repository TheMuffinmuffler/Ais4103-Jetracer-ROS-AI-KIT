#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <cstdlib>
#include <cmath>
#include <ctime>
#include <string>

static const double MIN_RUNTIME_SECONDS = 20.0;
static const double STILL_SECONDS = 8.0;
static const double CHECK_PERIOD = 1.0;
static const double POS_EPS = 0.02;
static const double YAW_EPS = 0.05;
static const int STABLE_CELL_THRESHOLD = 20;
static const int BIG_CHANGE_THRESHOLD  = 80;
static const int REQUIRED_STABLE_COUNT = 8;

ros::Time start_time;
ros::Time last_move_time;
bool saved = false;
bool have_pose = false;
geometry_msgs::PoseStamped latest_pose;
geometry_msgs::PoseStamped prev_pose_for_motion;
bool have_map = false;
bool new_map_since_last_check = false;
nav_msgs::OccupancyGrid latest_map;
bool have_eval_map = false;
nav_msgs::OccupancyGrid eval_map;
int stable_count = 0;

double yawFromQuaternion(const geometry_msgs::Quaternion& q)
{
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

double normalizeAngle(double a)
{
    while (a > M_PI)  a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

int countChangedCells(const nav_msgs::OccupancyGrid& a,
                      const nav_msgs::OccupancyGrid& b)
{
    if (a.info.width != b.info.width || a.info.height != b.info.height) return 999999999;
    if (a.data.size() != b.data.size()) return 999999999;

    int changed = 0;
    for (size_t i = 0; i < a.data.size(); i++) {
        if (a.data[i] != b.data[i]) changed++;
    }
    return changed;
}


void saveMap()
{
    const char* home = std::getenv("HOME");
    std::string dir = std::string(home ? home : "/tmp") + "/maps";
    std::string file = dir + "/final_map_best";

    std::string cmd1 = "mkdir -p " + dir;
    std::string cmd2 = "rosrun map_server map_saver -f " + file;

    ROS_WARN("[smart_slam] Creating map directory: %s", dir.c_str());
    std::system(cmd1.c_str());

    ROS_WARN("[smart_slam] Running: %s", cmd2.c_str());
    int ret = std::system(cmd2.c_str());

    if (ret == 0) {
        ROS_WARN("[smart_slam] Map saved successfully: %s.(pgm/yaml)", file.c_str());
    } else {
        ROS_ERROR("[smart_slam] map_saver failed with code %d", ret);
    }
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    latest_pose = *msg;

    if (!have_pose) {
        prev_pose_for_motion = *msg;
        have_pose = true;
        last_move_time = ros::Time::now();
        ROS_INFO("[smart_slam] First /slam_out_pose received.");
        return;
    }

    double dx = latest_pose.pose.position.x - prev_pose_for_motion.pose.position.x;
    double dy = latest_pose.pose.position.y - prev_pose_for_motion.pose.position.y;
    double dist = std::sqrt(dx * dx + dy * dy);

    double yaw_now  = yawFromQuaternion(latest_pose.pose.orientation);
    double yaw_prev = yawFromQuaternion(prev_pose_for_motion.pose.orientation);
    double dyaw = std::fabs(normalizeAngle(yaw_now - yaw_prev));

    if (dist > POS_EPS || dyaw > YAW_EPS) {
        last_move_time = ros::Time::now();
        prev_pose_for_motion = latest_pose;
    }
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    latest_map = *msg;
    have_map = true;
    new_map_since_last_check = true;
}

void timerCallback(const ros::TimerEvent&)
{
    if (saved) return;

    ros::Time now = ros::Time::now();
    double runtime = (now - start_time).toSec();
    double still_time = (now - last_move_time).toSec();

    if (!have_pose) {
        ROS_INFO("[smart_slam] Waiting for /slam_out_pose ... runtime=%.1f", runtime);
        return;
    }

    if (!have_map) {
        ROS_INFO("[smart_slam] Waiting for /map ... runtime=%.1f", runtime);
        return;
    }

    if (!have_eval_map) {
        eval_map = latest_map;
        have_eval_map = true;
        new_map_since_last_check = false;
        ROS_INFO("[smart_slam] First map stored for evaluation.");
        return;
    }

    int changed_cells = 0;

    if (new_map_since_last_check) {
        changed_cells = countChangedCells(latest_map, eval_map);
        eval_map = latest_map;
        new_map_since_last_check = false;
    } else {
        changed_cells = 0;
    }

    if (changed_cells <= STABLE_CELL_THRESHOLD) {
        stable_count++;
    } else if (changed_cells <= BIG_CHANGE_THRESHOLD) {
        if (stable_count > 0) stable_count--;
    } else {
        stable_count = 0;
    }

    if (runtime < MIN_RUNTIME_SECONDS) return;
    if (still_time < STILL_SECONDS) return;
    if (stable_count < REQUIRED_STABLE_COUNT) return;

    ROS_WARN("[smart_slam] Stop condition met: runtime=%.1fs still=%.1fs stable_count=%d. Saving map...",
             runtime, still_time, stable_count);

    saveMap();
    saved = true;
    ros::shutdown();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "smart_slam_node");
    ros::NodeHandle nh;

    start_time = ros::Time::now();
    last_move_time = start_time;

    ROS_INFO("[smart_slam] Started: Hector auto-save using /slam_out_pose + /map");

    ros::Subscriber map_sub  = nh.subscribe("/map", 1, mapCallback);
    ros::Subscriber pose_sub = nh.subscribe("/slam_out_pose", 20, poseCallback);
    ros::Timer timer = nh.createTimer(ros::Duration(CHECK_PERIOD), timerCallback);

    ros::spin();
    return 0;
}
