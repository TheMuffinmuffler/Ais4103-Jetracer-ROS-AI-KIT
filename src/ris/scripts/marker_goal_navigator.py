#!/usr/bin/env python
import os
import math
import yaml
import rospy
import actionlib
import tf.transformations as tft
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class MarkerGoalNavigator(object):
    def __init__(self):
        self.yaml_path = rospy.get_param("~yaml_path", "")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.move_base_action = rospy.get_param("~move_base_action", "/move_base")
        self.goal_topic = rospy.get_param("~goal_topic", "/target_marker_id")
        self.goal_timeout_sec = rospy.get_param("~goal_timeout_sec", 180.0)
        self.use_heading_to_goal = rospy.get_param("~use_heading_to_goal", True)
        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = None
        self.busy = False
        self.waypoints = {}
        self.load_waypoints()
        self.client = actionlib.SimpleActionClient(self.move_base_action, MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server: %s", self.move_base_action)

        while not rospy.is_shutdown():
            if self.client.wait_for_server(rospy.Duration(2.0)):
                break
            rospy.logwarn_throttle(10.0, "Still waiting for move_base action server...")

        rospy.loginfo("Connected to move_base action server.")

        self.goal_pose_pub = rospy.Publisher("/marker_goal_pose", PoseStamped, queue_size=1, latch=True)

        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_callback, queue_size=1)
        rospy.Subscriber(self.goal_topic, Int32, self.target_callback, queue_size=1)
        rospy.loginfo("marker_goal_navigator ready. Loaded %d waypoint(s).", len(self.waypoints))

    def load_waypoints(self):
        self.waypoints = {}

        if not os.path.exists(self.yaml_path):
            rospy.logwarn("Waypoints YAML not found: %s", self.yaml_path)
            return

        try:
            with open(self.yaml_path, "r") as f:
                data = yaml.safe_load(f) or {}

            for wp in data.get("waypoints", []):
                if "id" not in wp or "x" not in wp or "y" not in wp:
                    continue

                self.waypoints[int(wp["id"])] = {
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
        if self.busy:
            rospy.logwarn("Navigation is already active. Ignoring new marker command.")
            return

        marker_id = int(msg.data)
        self.load_waypoints()

        if marker_id not in self.waypoints:
            rospy.logwarn("Marker ID %d not found in YAML.", marker_id)
            return

        wp = self.waypoints[marker_id]
        goal_x = wp["x"]
        goal_y = wp["y"]
        goal_yaw = wp["yaw"]

        if self.use_heading_to_goal and self.robot_x is not None and self.robot_y is not None:
            dx = goal_x - self.robot_x
            dy = goal_y - self.robot_y
            if abs(dx) > 1e-6 or abs(dy) > 1e-6:
                goal_yaw = math.atan2(dy, dx)

        goal_pose = self.build_pose(goal_x, goal_y, goal_yaw)
        self.goal_pose_pub.publish(goal_pose)
        goal = MoveBaseGoal()
        goal.target_pose = goal_pose
        rospy.loginfo("Sending robot to marker %d -> x=%.3f y=%.3f yaw=%.3f",
                      marker_id, goal_x, goal_y, goal_yaw)

        self.busy = True
        self.client.send_goal(goal)

        finished = self.client.wait_for_result(rospy.Duration(self.goal_timeout_sec))

        if not finished:
            rospy.logwarn("Timeout while navigating to marker %d. Cancelling goal.", marker_id)
            self.client.cancel_goal()
            self.busy = False
            return

        state = self.client.get_state()
        rospy.loginfo("move_base finished for marker %d with state=%d", marker_id, state)

        self.busy = False


if __name__ == "__main__":
    rospy.init_node("marker_goal_navigator")
    try:
        MarkerGoalNavigator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass