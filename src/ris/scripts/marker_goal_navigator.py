#!/usr/bin/env python



"""
I wrote this node to connect marker IDs to move_base navigation goals. The node loads marker waypoints from a YAML file, listens to /target_marker_id, finds the matching waypoint, builds a MoveBaseGoal, and sends it to /move_base.

I used this node to let the robot navigate from its current AMCL-localized pose to a selected marker waypoint. This supports the project goal of moving the robot from x_0 to x_n using saved marker/waypoint positions.

ChatGPT was used as programming assistance for structuring the ROS Python node, loading YAML waypoints, building MoveBaseGoal messages, handling actionlib, and adding simple busy/timeout logic. The node was adapted, tested, and integrated by me for the JetRacer navigation setup.

This file uses standard ROS Python libraries, actionlib, move_base_msgs, geometry_msgs, std_msgs, rospy, PyYAML, and tf.transformations.

Logic:
1. Load marker waypoint data from YAML.
2. Subscribe to /amcl_pose to know the robot pose.
3. Subscribe to /target_marker_id.
4. When a marker ID is received, reload the YAML file.
5. Find the waypoint with the matching ID.
6. Optionally compute yaw so the robot faces toward the goal.
7. Publish the selected goal pose for visualization.
8. Send the goal to move_base.
9. Wait for success or timeout, then clear the busy flag.
"""


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
       #Modern Robotics, Ch. 3, Eq. (3.1), p. 61 
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
      
         #Modern Robotics, App. B.3, Eq. (B.9), p. 583
        q = msg.pose.pose.orientation
        quat = [q.x, q.y, q.z, q.w]
        
        #Modern Robotics, App. B.1.1, Eq. (B.4), p. 579
        _, _, self.robot_yaw = tft.euler_from_quaternion(quat)

    def build_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = self.map_frame
        
         #Modern Robotics, Ch. 3, Eq. (3.1), p. 61
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # Modern Robotics, App. B.3, Eq. (B.9), p. 583
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
                goal_yaw = math.atan2(dy, dx) #Modern Robotics, App. B.1.1, Eq. (B.4), p. 579.

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
