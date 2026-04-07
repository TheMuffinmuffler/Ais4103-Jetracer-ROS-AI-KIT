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
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge, CvBridgeError
import tf.transformations as tft

class ArucoMapper:
    def __init__(self):
        self.camera_name = rospy.get_param("~camera_name","csi_cam_0")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.bridge = CvBridge()
        self.counter = 0
        self.process_every_n_frames = 2 # Process more frequently for averaging

        self.camera_matrix = None
        self.dist_coeffs = None

        # Your measured ArUco marker size in meters
        self.marker_size = 0.038

        # --- MAPPING LOGIC ---
        self.mapping_ids = [1, 2, 3, 4] # The IDs we want to place on the map
        self.buffers = {mid: [] for mid in self.mapping_ids}
        self.buffer_size = 15 # Number of frames to average
        # ---------------------

        self.info_sub = rospy.Subscriber(self.camera_name+"/camera_info", CameraInfo, self.info_callback)
        self.image_sub = rospy.Subscriber("/cv_video/compressed", CompressedImage, self.callback)
        
        self.image_pub = rospy.Publisher("/aruco_video/compressed", CompressedImage, queue_size=10)
        self.waypoint_pub = rospy.Publisher("/aruco_waypoints", PoseStamped, queue_size=10)
        self.rviz_pub = rospy.Publisher("/aruco_markers_rviz", Marker, queue_size=10)



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
                            marker_id = int(ids[i][0])
                            
                            # A: FILTER - Only process IDs in our whitelist
                            if marker_id not in self.mapping_ids:
                                continue

                            # Draw the 3D axes
                            aruco.drawAxis(img, self.camera_matrix, self.dist_coeffs,
                                           rvecs[i], tvecs[i], self.marker_size)

                            # Extract translation vectors
                            x_dist = tvecs[i][0][0]
                            y_dist = tvecs[i][0][1]
                            z_dist = tvecs[i][0][2]

                            # Prepare point for TF transformation
                            local_point = PointStamped()
                            local_point.header.stamp = rospy.Time(0)
                            local_point.header.frame_id = self.camera_name
                            local_point.point.x = z_dist  # Forward
                            local_point.point.y = -x_dist # Left
                            local_point.point.z = -y_dist # Up

                            try:
                                # Transform to global 'map' frame
                                # Note: We need the full transform (position + orientation)
                                local_pose = PoseStamped()
                                local_pose.header.stamp = rospy.Time(0)
                                local_pose.header.frame_id = self.camera_name
                                local_pose.pose.position.x = z_dist
                                local_pose.pose.position.y = -x_dist
                                local_pose.pose.position.z = -y_dist
                                
                                # Convert rvec to quaternion for the transform
                                R, _ = cv.Rodrigues(rvecs[i])
                                T_4x4 = np.eye(4)
                                T_4x4[:3, :3] = R
                                quat = tft.quaternion_from_matrix(T_4x4)
                                
                                # ArUco axes are weird; swap them to match ROS (Z forward, X right, Y up)
                                # But let's just use the position for now and calculate yaw from the map transform
                                local_pose.pose.orientation.x = quat[0]
                                local_pose.pose.orientation.y = quat[1]
                                local_pose.pose.orientation.z = quat[2]
                                local_pose.pose.orientation.w = quat[3]

                                global_pose = self.tf_buffer.transform(local_pose, "map", rospy.Duration(0.1))
                                
                                # B: BUFFER - Add to the smoothing buffer
                                self.buffers[marker_id].append(global_pose.pose)

                                # C: AVERAGE & PUBLISH
                                if len(self.buffers[marker_id]) >= self.buffer_size:
                                    avg_x = sum(p.position.x for p in self.buffers[marker_id]) / self.buffer_size
                                    avg_y = sum(p.position.y for p in self.buffers[marker_id]) / self.buffer_size
                                    
                                    # For orientation, we'll just take the last one or average the yaw
                                    # (Simplification: Use the most recent orientation for the mapped marker)
                                    final_quat = self.buffers[marker_id][-1].orientation
                                    
                                    # Finalized Pose
                                    final_pose = PoseStamped()
                                    final_pose.header.frame_id = str(marker_id)
                                    final_pose.pose.position.x = avg_x
                                    final_pose.pose.position.y = avg_y
                                    final_pose.pose.orientation = final_quat
                                    
                                    # 1. Send to the Waypoint Saver
                                    self.waypoint_pub.publish(final_pose)
                                    
                                    # 2. Publish to RViz for visual confirmation
                                    self.publish_rviz_marker(marker_id, avg_x, avg_y, final_quat)

                                    # Reset buffer to prevent spamming
                                    self.buffers[marker_id] = []
                                    rospy.loginfo("PLACED: Marker [%s] at X: %.2f, Y: %.2f", marker_id, avg_x, avg_y)

                            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                                rospy.logwarn_throttle(10, "No map transform! Marker detected but map not ready.")

                        # We found a marker, stop checking other dictionaries
                        break

    def publish_rviz_marker(self, marker_id, x, y, quat):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "aruco_mapping"
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.05 # Slightly above the floor
        marker.pose.orientation = quat
        
        marker.scale.x = 0.08
        marker.scale.y = 0.08
        marker.scale.z = 0.08
        
        marker.color.r = 0.0
        marker.color.g = 1.0 # Bright green
        marker.color.b = 0.0
        marker.color.a = 1.0 # Opaque
        
        self.rviz_pub.publish(marker)

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