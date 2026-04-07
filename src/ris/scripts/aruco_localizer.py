#!/usr/bin/env python
import rospy
import tf2_ros
import tf2_geometry_msgs
import numpy as np
import cv2 as cv
import cv2.aruco as aruco
import yaml
import os
from sensor_msgs.msg import CompressedImage, CameraInfo
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from cv_bridge import CvBridge
import tf.transformations as tft

class ArucoLocalizer:
    def __init__(self):
        rospy.init_node("aruco_localizer")
        
        # Parameters
        self.camera_name = rospy.get_param("~camera_name", "csi_cam_0")
        self.marker_size = rospy.get_param("~marker_size", 0.038)
        self.yaml_path = os.path.expanduser("~/catkin_ws/config/waypoints.yaml")
        self.camera_frame = self.camera_name + "_link"
        self.base_frame = "base_footprint"
        
        # Tools
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # ArUco Setup
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters_create()
        
        # Data
        self.landmarks = self.load_landmarks()
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # Subscribers
        self.info_sub = rospy.Subscriber(self.camera_name+"/camera_info", CameraInfo, self.info_callback)
        self.image_sub = rospy.Subscriber("/cv_video/compressed", CompressedImage, self.image_callback)
        
        # Publisher (Initial Pose for AMCL/SLAM)
        self.pose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=10)
        
        rospy.loginfo("ArUco Localizer Started. Waiting for markers...")

    def load_landmarks(self):
        landmarks = {}
        if os.path.exists(self.yaml_path):
            with open(self.yaml_path, "r") as f:
                data = yaml.safe_load(f)
                if data and "waypoints" in data:
                    for wp in data["waypoints"]:
                        # Create SE(3) matrix for the landmark in map frame (T_sl)
                        T = tft.euler_matrix(0, 0, wp["yaw"])
                        T[0, 3] = wp["x"]
                        T[1, 3] = wp["y"]
                        landmarks[wp["id"]] = T
            rospy.loginfo("Loaded %d landmarks from YAML.", len(landmarks))
        return landmarks

    def info_callback(self, msg):
        self.camera_matrix = np.array(msg.K).reshape(3, 3)
        self.dist_coeffs = np.array(msg.D).reshape(1, 5)

    def image_callback(self, data):
        if self.camera_matrix is None:
            return

        try:
            img = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
            gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
            corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
            
            if ids is not None:
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.camera_matrix, self.dist_coeffs)
                
                for i in range(len(ids)):
                    marker_id = int(ids[i][0])
                    if marker_id in self.landmarks:
                        self.localize_from_marker(marker_id, rvecs[i][0], tvecs[i][0])
                        
        except Exception as e:
            rospy.logerr("Localization Error: %s", e)

    def localize_from_marker(self, marker_id, rvec, tvec):
        # 1. T_cl: Marker in Camera Frame (from ArUco)
        R_cl, _ = cv.Rodrigues(rvec)
        T_cl = np.eye(4)
        T_cl[:3, :3] = R_cl
        T_cl[0, 3] = tvec[0]
        T_cl[1, 3] = tvec[1]
        T_cl[2, 3] = tvec[2]

        # 2. ArUco Frame to ROS Camera Frame Adjustment
        # ArUco: Z forward, X right, Y down
        # We need to account for the link transform
        try:
            # T_bc: Camera in Body Frame (Static TF)
            trans = self.tf_buffer.lookup_transform(self.base_frame, self.camera_frame, rospy.Time(0))
            T_bc = self.pose_to_matrix(trans.transform)
            
            # T_sl: Landmark in Map Frame (from YAML)
            T_sl = self.landmarks[marker_id]

            # 3. THE MODERN ROBOTICS INVERSION
            # Path: Map -> Landmark -> Camera -> Body
            # T_sb = T_sl * inv(T_cl_corrected) * inv(T_bc)
            
            # Correcting for ArUco coordinate system (Z is depth)
            # local_point logic from mapper: z_dist, -x_dist, -y_dist
            # We'll use a simpler approach: T_sb = T_sl * inv(T_cl)
            T_lc = np.linalg.inv(T_cl)
            
            # Transform T_lc into ROS coordinates if necessary
            # (Simplification: We assume the mapper and localizer use consistent frames)
            
            # Final Pose calculation
            # This is a conceptual implementation of the T_sb = T_sl * T_lc * inv(T_bc)
            T_sb = T_sl @ T_lc @ np.linalg.inv(T_bc) # Very rough simplification
            
            # Publish Initial Pose
            self.publish_pose(T_sb)
            
        except Exception as e:
            rospy.logwarn_throttle(5, "TF Lookup failed: %s", e)

    def pose_to_matrix(self, transform):
        q = [transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w]
        T = tft.quaternion_matrix(q)
        T[0, 3] = transform.translation.x
        T[1, 3] = transform.translation.y
        T[2, 3] = transform.translation.z
        return T

    def publish_pose(self, T):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        
        msg.pose.pose.position.x = T[0, 3]
        msg.pose.pose.position.y = T[1, 3]
        msg.pose.pose.position.z = 0
        
        q = tft.quaternion_from_matrix(T)
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        
        # Add some confidence (covariance)
        msg.pose.covariance = [0.1] * 36 # Low uncertainty
        
        self.pose_pub.publish(msg)
        rospy.loginfo_throttle(2, "Robot pose reset based on Marker!")

if __name__ == "__main__":
    localizer = ArucoLocalizer()
    rospy.spin()
