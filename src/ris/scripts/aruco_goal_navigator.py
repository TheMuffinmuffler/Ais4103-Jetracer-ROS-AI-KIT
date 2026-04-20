#!/usr/bin/env python
import os
import yaml
import math
import rospy
import actionlib
import tf.transformations as tft

from std_msgs.msg import Int32
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class ArucoGoalNavigator(object):
    def __init__(self):
        self.yaml_path = rospy.get_param(
            "~yaml_path",
            os.path.expanduser("~/catkin_ws/src/ris/map/aruco_waypoints.yaml")
        )
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.move_base_action = rospy.get_param("~move_base_action", "/move_base")
        self.goal_topic = rospy.get_param("~goal_topic", "/target_marker_id")
        self.wait_for_server_sec = rospy.get_param("~wait_for_server_sec", 20.0)
        self.goal_timeout_sec = rospy.get_param("~goal_timeout_sec", 120.0)
        self.use_closest_yaw_if_missing = rospy.get_param("~use_closest_yaw_if_missing", True)

        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = None

        self.waypoints = {}
        self.load_waypoints()

        self.amcl_sub = rospy.Subscriber(
            "/amcl_pose",
            PoseWithCovarianceStamped,
            self.amcl_callback,
            queue_size=1
        )
        self.target_sub = rospy.Subscriber(
            self.goal_topic,
            Int32,
            self.target_callback,
            queue_size=1
        )
        self.goal_pose_pub = rospy.Publisher(
            "/aruco_goal_pose",
            PoseStamped,
            queue_size=1,
            latch=True
        )

        self.client = actionlib.SimpleActionClient(self.move_base_action, MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server: %s", self.move_base_action)
        if not self.client.wait_for_server(rospy.Duration(self.wait_for_server_sec)):
            rospy.logerr("move_base action server not available.")
            raise rospy.ROSException("move_base action server not available")

        rospy.loginfo("aruco_goal_navigator ready. Loaded %d waypoint(s).", len(self.waypoints))

    def load_waypoints(self):
        self.waypoints = {}

        if not os.path.exists(self.yaml_path):
            rospy.logwarn("Waypoints YAML does not exist yet: %s", self.yaml_path)
            return

        try:
            with open(self.yaml_path, "r") as f:
                data = yaml.safe_load(f) or {}

            items = data.get("waypoints", [])
            for wp in items:
                if "id" not in wp or "x" not in wp or "y" not in wp:
                    continue

                marker_id = int(wp["id"])
                self.waypoints[marker_id] = {
                    "x": float(wp["x"]),
                    "y": float(wp["y"]),
                    "yaw": float(wp.get("yaw", 0.0))
                }

            rospy.loginfo("Loaded %d marker waypoint(s) from %s", len(self.waypoints), self.yaml_path)

        except Exception as e:
            rospy.logerr("Failed to load waypoints YAML: %s", e)

    def amcl_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        quat = [q.x, q.y, q.z, q.w]
        _, _, self.robot_yaw = tft.euler_from_quaternion(quat)

    def build_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = self.map_frame
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        q = tft.quaternion_from_euler(0.0, 0.0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return pose

    def target_callback(self, msg):
        marker_id = int(msg.data)

        self.load_waypoints()

        if marker_id not in self.waypoints:
            rospy.logwarn("Marker ID %d not found in %s", marker_id, self.yaml_path)
            return

        wp = self.waypoints[marker_id]
        goal_x = wp["x"]
        goal_y = wp["y"]
        goal_yaw = wp.get("yaw", 0.0)

        if self.use_closest_yaw_if_missing and self.robot_x is not None and self.robot_y is not None:
            dx = goal_x - self.robot_x
            dy = goal_y - self.robot_y
            if abs(dx) > 1e-6 or abs(dy) > 1e-6:
                goal_yaw = math.atan2(dy, dx)

        goal_pose = self.build_pose(goal_x, goal_y, goal_yaw)
        self.goal_pose_pub.publish(goal_pose)

        goal = MoveBaseGoal()
        goal.target_pose = goal_pose

        rospy.loginfo(
            "Sending robot to marker %d -> x=%.3f y=%.3f yaw=%.3f",
            marker_id, goal_x, goal_y, goal_yaw
        )

        self.client.send_goal(goal)

        finished = self.client.wait_for_result(rospy.Duration(self.goal_timeout_sec))
        if not finished:
            rospy.logwarn("Timeout while navigating to marker %d. Cancelling goal.", marker_id)
            self.client.cancel_goal()
            return

        state = self.client.get_state()
        rospy.loginfo("move_base finished for marker %d with state=%d", marker_id, state)


if __name__ == "__main__":
    rospy.init_node("aruco_goal_navigator")
    try:
        navigator = ArucoGoalNavigator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass