#include <ros/ros.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Int32.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>
#include <vector>
#include <algorithm>
#include <string>

struct Waypoint
{
    int id;
    double x;
    double y;
    double yaw;
};

class MarkerGraphNavigator
{
public:
    MarkerGraphNavigator()
        : nh_(),
          pnh_("~"),
          ac_("move_base", true),
          has_pose_(false)
    {
        pnh_.param("map_frame", map_frame_, std::string("map"));
        pnh_.param("goal_tolerance_m", goal_tolerance_m_, 0.35);
        pnh_.param("wait_for_move_base_sec", wait_for_move_base_sec_, 20.0);
        pnh_.param("wait_each_goal_sec", wait_each_goal_sec_, 120.0);
        pnh_.param("use_graph", use_graph_, true);

        if (!loadGraphFromParams())
        {
            ROS_FATAL("Failed to load /graph/waypoints . Navigation node cannot continue.");
            ros::shutdown();
            return;
        }

        amcl_sub_ = nh_.subscribe("/amcl_pose", 1, &MarkerGraphNavigator::amclCallback, this);
        target_sub_ = nh_.subscribe("/target_marker_id", 1, &MarkerGraphNavigator::targetCallback, this);
        path_pub_ = nh_.advertise<nav_msgs::Path>("/planned_route", 1, true);

        ROS_INFO("Waiting for move_base action server...");
        if (!ac_.waitForServer(ros::Duration(wait_for_move_base_sec_)))
        {
            ROS_FATAL("move_base action server not available.");
            ros::shutdown();
            return;
        }

        ROS_INFO("marker_graph_navigator is ready.");
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber amcl_sub_;
    ros::Subscriber target_sub_;
    ros::Publisher path_pub_;

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_;

    std::unordered_map<int, Waypoint> waypoints_;
    std::unordered_map<int, std::vector<std::pair<int, double>>> adj_;

    bool has_pose_;
    double robot_x_;
    double robot_y_;
    double robot_yaw_;

    std::string map_frame_;
    double goal_tolerance_m_;
    double wait_for_move_base_sec_;
    double wait_each_goal_sec_;
    bool use_graph_;

    static double dist2d(double x1, double y1, double x2, double y2)
    {
        const double dx = x2 - x1;
        const double dy = y2 - y1;
        return std::sqrt(dx * dx + dy * dy);
    }

    bool loadGraphFromParams()
    {
        XmlRpc::XmlRpcValue wp_list;
        XmlRpc::XmlRpcValue edge_list;

        if (!nh_.getParam("/graph/waypoints", wp_list))
        {
            ROS_ERROR("Missing /graph/waypoints");
            return false;
        }

        if (wp_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
            ROS_ERROR("/graph/waypoints is not an array.");
            return false;
        }

        waypoints_.clear();
        adj_.clear();

        for (int i = 0; i < wp_list.size(); ++i)
        {
            Waypoint w;
            w.id = static_cast<int>(wp_list[i]["id"]);
            w.x = static_cast<double>(wp_list[i]["x"]);
            w.y = static_cast<double>(wp_list[i]["y"]);
            w.yaw = wp_list[i].hasMember("yaw") ? static_cast<double>(wp_list[i]["yaw"]) : 0.0;
            waypoints_[w.id] = w;
        }

        if (nh_.getParam("/graph/edges", edge_list))
        {
            if (edge_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
            {
                ROS_WARN("/graph/edges exists but is not an array. Falling back to direct-goal mode.");
                return true;
            }

            for (int i = 0; i < edge_list.size(); ++i)
            {
                int from = static_cast<int>(edge_list[i]["from"]);
                int to   = static_cast<int>(edge_list[i]["to"]);

                if (waypoints_.find(from) == waypoints_.end() || waypoints_.find(to) == waypoints_.end())
                {
                    ROS_WARN("Edge references unknown waypoint: %d -> %d", from, to);
                    continue;
                }

                double cost = 0.0;
                if (edge_list[i].hasMember("cost"))
                {
                    cost = static_cast<double>(edge_list[i]["cost"]);
                }
                else
                {
                    cost = dist2d(waypoints_[from].x, waypoints_[from].y,
                                  waypoints_[to].x, waypoints_[to].y);
                }

                adj_[from].push_back(std::make_pair(to, cost));
                adj_[to].push_back(std::make_pair(from, cost));
            }
        }
        else
        {
            ROS_WARN("No /graph/edges found. Navigator will send direct goal to target marker.");
        }

        ROS_INFO("Loaded %zu waypoints and %zu adjacency entries.",
                 waypoints_.size(), adj_.size());

        return !waypoints_.empty();
    }

    void amclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
    {
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;

        const double qx = msg->pose.pose.orientation.x;
        const double qy = msg->pose.pose.orientation.y;
        const double qz = msg->pose.pose.orientation.z;
        const double qw = msg->pose.pose.orientation.w;

        const double siny_cosp = 2.0 * (qw * qz + qx * qy);
        const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
        robot_yaw_ = std::atan2(siny_cosp, cosy_cosp);

        has_pose_ = true;
    }

    int nearestWaypointId(double x, double y) const
    {
        double best_d = std::numeric_limits<double>::infinity();
        int best_id = -1;

        for (std::unordered_map<int, Waypoint>::const_iterator it = waypoints_.begin();
             it != waypoints_.end(); ++it)
        {
            const double d = dist2d(x, y, it->second.x, it->second.y);
            if (d < best_d)
            {
                best_d = d;
                best_id = it->first;
            }
        }

        return best_id;
    }

    bool dijkstra(int start_id, int goal_id, std::vector<int>& path)
    {
        path.clear();

        if (waypoints_.find(start_id) == waypoints_.end() ||
            waypoints_.find(goal_id) == waypoints_.end())
        {
            return false;
        }

        if (adj_.empty())
        {
            path.push_back(goal_id);
            return true;
        }

        const double INF = std::numeric_limits<double>::infinity();
        std::unordered_map<int, double> dist;
        std::unordered_map<int, int> parent;

        for (std::unordered_map<int, Waypoint>::const_iterator it = waypoints_.begin();
             it != waypoints_.end(); ++it)
        {
            dist[it->first] = INF;
        }

        dist[start_id] = 0.0;
        parent[start_id] = start_id;

        typedef std::pair<double, int> QItem;
        std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem> > pq;
        pq.push(std::make_pair(0.0, start_id));

        while (!pq.empty())
        {
            double d = pq.top().first;
            int u = pq.top().second;
            pq.pop();

            if (d > dist[u])
                continue;

            if (u == goal_id)
                break;

            std::unordered_map<int, std::vector<std::pair<int, double>>>::const_iterator adj_it = adj_.find(u);
            if (adj_it == adj_.end())
                continue;

            for (size_t i = 0; i < adj_it->second.size(); ++i)
            {
                int v = adj_it->second[i].first;
                double w = adj_it->second[i].second;
                double nd = d + w;

                if (nd < dist[v])
                {
                    dist[v] = nd;
                    parent[v] = u;
                    pq.push(std::make_pair(nd, v));
                }
            }
        }

        if (parent.find(goal_id) == parent.end())
            return false;

        int cur = goal_id;
        while (cur != start_id)
        {
            path.push_back(cur);
            cur = parent[cur];
        }
        path.push_back(start_id);
        std::reverse(path.begin(), path.end());
        return true;
    }

    geometry_msgs::PoseStamped makeGoalPose(const Waypoint& w) const
    {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = map_frame_;
        pose.pose.position.x = w.x;
        pose.pose.position.y = w.y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = std::sin(w.yaw * 0.5);
        pose.pose.orientation.w = std::cos(w.yaw * 0.5);
        return pose;
    }

    void publishPath(const std::vector<int>& ids)
    {
        nav_msgs::Path path_msg;
        path_msg.header.stamp = ros::Time::now();
        path_msg.header.frame_id = map_frame_;

        for (size_t i = 0; i < ids.size(); ++i)
        {
            std::unordered_map<int, Waypoint>::const_iterator it = waypoints_.find(ids[i]);
            if (it == waypoints_.end())
                continue;

            path_msg.poses.push_back(makeGoalPose(it->second));
        }

        path_pub_.publish(path_msg);
    }

    bool sendGoalAndWait(const Waypoint& w)
    {
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose = makeGoalPose(w);

        ROS_INFO("Sending goal -> id=%d  (%.3f, %.3f, yaw=%.3f)",
                 w.id, w.x, w.y, w.yaw);

        ac_.sendGoal(goal);
        bool finished = ac_.waitForResult(ros::Duration(wait_each_goal_sec_));

        if (!finished)
        {
            ROS_WARN("Timeout while going to waypoint id=%d. Cancelling goal.", w.id);
            ac_.cancelGoal();
            return false;
        }

        actionlib::SimpleClientGoalState state = ac_.getState();
        ROS_INFO("move_base result for id=%d : %s", w.id, state.toString().c_str());

        return state == actionlib::SimpleClientGoalState::SUCCEEDED;
    }

    void targetCallback(const std_msgs::Int32::ConstPtr& msg)
    {
        if (!has_pose_)
        {
            ROS_WARN("No AMCL pose yet. Cannot navigate.");
            return;
        }

        const int target_id = msg->data;
        if (waypoints_.find(target_id) == waypoints_.end())
        {
            ROS_WARN("Target marker id %d not found in loaded waypoints.", target_id);
            return;
        }

        const int start_id = nearestWaypointId(robot_x_, robot_y_);
        if (start_id < 0)
        {
            ROS_WARN("Could not determine nearest start waypoint.");
            return;
        }

        ROS_INFO("Robot pose: (%.3f, %.3f). Nearest graph node = %d. Target = %d",
                 robot_x_, robot_y_, start_id, target_id);

        std::vector<int> route_ids;

        if (use_graph_ && !adj_.empty())
        {
            if (!dijkstra(start_id, target_id, route_ids))
            {
                ROS_WARN("No graph path found from %d to %d. Falling back to direct goal.", start_id, target_id);
                route_ids.clear();
                route_ids.push_back(target_id);
            }
        }
        else
        {
            route_ids.push_back(target_id);
        }

        publishPath(route_ids);

        for (size_t i = 0; i < route_ids.size(); ++i)
        {
            const Waypoint& w = waypoints_[route_ids[i]];
            if (!sendGoalAndWait(w))
            {
                ROS_WARN("Navigation aborted at waypoint id=%d", w.id);
                return;
            }
        }

        ROS_INFO("Route to target marker %d completed successfully.", target_id);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "marker_graph_navigator");
    MarkerGraphNavigator node;
    ros::spin();
    return 0;
}