#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
import cv2 as cv
import numpy as np
import cv2.aruco as aruco
import tf2_ros
import tf2_geometry_msgs

from sensor_msgs.msg import CompressedImage, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge, CvBridgeError

class ArucoMapper:
    def __init__(self):
        self.camera_name = rospy.get_param("~camera_name","csi_cam_0")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.bridge = CvBridge()
        self.counter = 0
        self.process_every_n_frames = 5 #Skipps every 5 frames

        self.camera_matrix = None
        self.dist_coeffs = None

        # Your measured ArUco marker size in meters
        self.marker_size = 0.038


        self.info_sub = rospy.Subscriber(self.camera_name+"/camera_info", CameraInfo, self.info_callback)

        self.image_sub = rospy.Subscriber("/cv_video/compressed", CompressedImage, self.callback)
        self.image_pub = rospy.Publisher("/aruco_video/compressed", CompressedImage, queue_size=10)

        self.waypoint_pub = rospy.Publisher("/aruco_waypoints", PointStamped, queue_size=10)



        self.aruco_dicts = [
            aruco.Dictionary_get(aruco.DICT_4X4_50),
            aruco.Dictionary_get(aruco.DICT_5X5_100),
            aruco.Dictionary_get(aruco.DICT_6X6_250),
            aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
        ]
        self.aruco_params = aruco.DetectorParameters_create()

    def info_callback(self, msg):
        self.camera_matrix = np.array(msg.K).reshape(3, 3)
        self.dist_coeffs = np.array(msg.D).reshape(1, 5)


    def callback(self, data):
        # Increment frame counter
        self.counter += 1

        try:
            # Decode the compressed image
            img = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return

        # Only process ArUco logic every N frames
        if self.counter >= self.process_every_n_frames:
            self.counter = 0 # Reset counter

            # Ensure we have camera calibration before trying to find poses
            if self.camera_matrix is None or self.dist_coeffs is None:
                rospy.logwarn_throttle(5, "Waiting for camera_info...")
            else:
                # Convert to grayscale for detection
                if len(img.shape) == 3:
                    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
                else:
                    gray = img

                # Loop through dictionaries
                for current_dict in self.aruco_dicts:
                    corners, ids, rejectedImgPoints = aruco.detectMarkers(
                        gray, current_dict, parameters=self.aruco_params)

                    # CRITICAL FIX: Ensure ids actually contains markers before processing and breaking!
                    if ids is not None and len(ids) > 0:

                        # Draw visual markers on the color image
                        aruco.drawDetectedMarkers(img, corners, ids)

                        # Estimate pose (3D position/rotation)
                        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                            corners, self.marker_size, self.camera_matrix, self.dist_coeffs)

                        for i in range(len(ids)):
                            # Draw the 3D axes
                            aruco.drawAxis(img, self.camera_matrix, self.dist_coeffs,
                                           rvecs[i], tvecs[i], self.marker_size)

                            # Extract translation vectors
                            x_dist = tvecs[i][0][0]
                            y_dist = tvecs[i][0][1]
                            z_dist = tvecs[i][0][2]

                            # ROS-FRIENDLY PRINT: This forces the terminal to update immediately
                            rospy.loginfo("Marker [%s]: is %.2fm left/right, %.2fm up/down, and %.2fm straight ahead!",
                                          str(ids[i][0]), x_dist, y_dist, z_dist)

                            # Prepare point for TF transformation
                            local_point = PointStamped()
                            local_point.header.stamp = rospy.Time(0)
                            local_point.header.frame_id = self.camera_name

                            # Coordinate swap based on your lens math
                            local_point.point.x = z_dist  # Forward
                            local_point.point.y = -x_dist # Left
                            local_point.point.z = -y_dist # Up

                            try:
                                # Transform to global 'map' frame
                                global_point = self.tf_buffer.transform(local_point, "map", rospy.Duration(0.1))

                                # Use marker ID as the frame name for the saver
                                global_point.header.frame_id = str(ids[i][0])

                                # Publish the result
                                self.waypoint_pub.publish(global_point)

                            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                                # WARN INSTEAD OF PASSING
                                rospy.logwarn_throttle(5, "No map transform found! Publishing raw camera coordinates. Error: {}".format(e))

                                # Fallback: Publish raw camera coords so the saver still works
                                local_point.header.frame_id = str(ids[i][0])
                                self.waypoint_pub.publish(local_point)

                        # We ACTUALLY found a marker, so now it is safe to stop checking other dictionaries
                        break

                        # --- Visuals and Publishing ---
        cv.imshow("ArUco Scanner", img)
        cv.waitKey(1)

        try:
            # Publish the image
            self.image_pub.publish(self.bridge.cv2_to_compressed_imgmsg(img))
        except CvBridgeError as e:
            rospy.logerr("CvBridge Publish Error: {0}".format(e))

def main(args):
    rospy.init_node("aruco_mapper", anonymous=True)
    mapper = ArucoMapper()
    print("ArUco Mapper Node Started! Looking for Markers...")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")
    cv.destroyAllWindows()

if __name__ == "__main__":
    main(sys.argv)