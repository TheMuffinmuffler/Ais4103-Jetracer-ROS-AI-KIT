#!/usr/bin/env python
import os
import yaml
import rospy
from geometry_msgs.msg import PointStamped

class WaypointSaver:
    def __init__(self):
        self.yaml_path = os.path.expanduser("~/catkin_ws/config/waypoints.yaml")
        self.seen_markers = set()

        self.load_existing_markers()

        self.waypoint_sub = rospy.Subscriber("/aruco_waypoints", PointStamped, self.waypoint_callback)
        print("Waypoint Saver Node Started. Listening to /aruco_waypoints....")

    def load_existing_markers(self):
        if os.path.exists(self.yaml_path):
            try:
                with open(self.yaml_path,"r") as file:
                    data = yaml.safe_load(file) or {}

                    if "waypoints" in data:
                        for wp in data["waypoints"]:
                            self.seen_markers.add(wp["id"])
                print("Loaded {} existing markers from YAML.".format(len(self.seen_markers)))
            except Exception as e:
                print("Error loading YAML on startup {}".format(e))
    def waypoint_callback(self, msg):
        # FIX: Corrected indentation and function signature (self, msg)
        marker_id = int(msg.header.frame_id)
        map_x = msg.point.x
        map_y = msg.point.y

        if marker_id not in self.seen_markers:
            self.seen_markers.add(marker_id)
            self.save_to_yaml(marker_id, map_x, map_y)

    def save_to_yaml(self, marker_id, map_x, map_y):
        print("New Marker [{}] detected at X: {:.2f}, Y: {:.2f}. Saving....".format(marker_id, map_x, map_y))

        try:
            # 1. Read the file if it exists, or create an empty dictionary if it doesn't
            if os.path.exists(self.yaml_path):
                with open(self.yaml_path, "r") as file:
                    data = yaml.safe_load(file) or {}
            else:
                data = {}

            # FIX: Ensure the 'waypoints' list exists before we try to loop through it
            if "waypoints" not in data:
                data["waypoints"] = []

            # FIX: Pulled this code OUT of the 'else' block so it runs every time
            # Check if marker already exists in the file
            for wp in data["waypoints"]:
                if wp["id"] == marker_id:
                    print("Marker [{}] already exists in YAML. Skipping".format(marker_id))
                    self.seen_markers.add(marker_id)
                    return

                    # Add the new waypoint
            new_waypoint = {"id": marker_id, "x": round(map_x, 4), "y": round(map_y, 4)}
            data["waypoints"].append(new_waypoint)

            # Save it back to the file
            with open(self.yaml_path, "w") as file:
                # FIX: Corrected spelling of default_flow_style
                yaml.dump(data, file, default_flow_style=None)

            self.seen_markers.add(marker_id)
            print("Successfully added to waypoints.yaml!")

        except Exception as e:
            print("Error saving to YAML: {}".format(e))
            self.seen_markers.remove(marker_id)

if __name__ == "__main__":
    rospy.init_node("waypoint_saver", anonymous=True)
    saver = WaypointSaver()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass