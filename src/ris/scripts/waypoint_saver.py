#!/usr/bin/env python
import os
import yaml
import rospy
from geometry_msgs.msg import PoseStamped
import tf.transformations as tft

class WaypointSaver:
    def __init__(self):
        self.yaml_path = os.path.expanduser("~/catkin_ws/config/waypoints.yaml")
        self.seen_markers = set()

        self.load_existing_markers()

        self.waypoint_sub = rospy.Subscriber("/aruco_waypoints", PoseStamped, self.waypoint_callback)
        print("Waypoint Saver Node Started. Listening to /aruco_waypoints....")

    def load_existing_markers(self):
        if os.path.exists(self.yaml_path):
            try:
                with open(self.yaml_path, "r") as file:
                    data = yaml.safe_load(file) or {}
                    if "waypoints" in data:
                        for wp in data["waypoints"]:
                            self.seen_markers.add(wp["id"])
                print("Loaded {} existing markers from YAML.".format(len(self.seen_markers)))
            except Exception as e:
                print("Error loading YAML on startup {}".format(e))

    def waypoint_callback(self, msg):
        try:
            marker_id = int(msg.header.frame_id)
            map_x = msg.pose.position.x
            map_y = msg.pose.position.y
            
            # Convert Quaternion to Euler to get Yaw
            q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
            _, _, yaw = tft.euler_from_quaternion(q)

            # Always attempt to save
            self.save_to_yaml(marker_id, map_x, map_y, yaw)
        except Exception as e:
            rospy.logerr("Error in waypoint_callback: {}".format(e))

    def save_to_yaml(self, marker_id, map_x, map_y, yaw):
        try:
            # 1. Read existing data
            if os.path.exists(self.yaml_path):
                with open(self.yaml_path, "r") as file:
                    data = yaml.safe_load(file) or {}
            else:
                data = {}

            if "waypoints" not in data:
                data["waypoints"] = []

            # 2. Check if marker exists - if so, UPDATE it
            found = False
            for wp in data["waypoints"]:
                if wp["id"] == marker_id:
                    wp["x"] = round(map_x, 4)
                    wp["y"] = round(map_y, 4)
                    wp["yaw"] = round(yaw, 4)
                    found = True
                    break

            # 3. If not found, ADD it
            if not found:
                rospy.loginfo("New Marker [%s] detected! Saving to YAML.", marker_id)
                new_waypoint = {"id": marker_id, "x": round(map_x, 4), "y": round(map_y, 4), "yaw": round(yaw, 4)}
                data["waypoints"].append(new_waypoint)

            # 4. Save back to the file
            with open(self.yaml_path, "w") as file:
                yaml.dump(data, file, default_flow_style=None)

            self.seen_markers.add(marker_id)

        except Exception as e:
            rospy.logerr("Error saving to YAML: {}".format(e))

if __name__ == "__main__":
    rospy.init_node("waypoint_saver", anonymous=True)
    saver = WaypointSaver()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

