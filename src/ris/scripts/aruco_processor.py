#!/usr/bin/env python
import rospy
import tf2_ros
import tf2_geometry_msgs
import numpy as np
import cv2 as cv
import cv2.aruco as aruco
import yaml
import os
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, PointStamped
from visualization_msgs.msg import Marker
from std_srvs.srv import Trigger, TriggerResponse
from cv_bridge import CvBridge
import tf.transformations as tft

class ArucoProcessor:
    def __init__(self):
        rospy.init_node("aruco_processor")
        
        # Parameters
        self.camera_name = rospy.get_param("~camera_name", "/csi_cam_0")
        self.marker_size = rospy.get_param("~marker_size", 0.038)
        self.yaml_path = os.path.expanduser("~/catkin_ws/config/waypoints.yaml")
        # Ensure frame names don't have leading slash for TF2 compatibility
        self.camera_frame = self.camera_name.lstrip("/") + "_link"
        self.base_frame = rospy.get_param("~base_frame", "base_footprint").lstrip("/")
        
        # Mode Parameters
        self.enable_mapping = rospy.get_param("~enable_mapping", True) #place markers and saves them
        self.enable_localization = rospy.get_param("~enable_localization", False) #locate the car using the aruco codes
        self.process_every_n_frames = rospy.get_param("~process_every_n_frames", 2)
        
        # Tools
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.counter = 0
        
        # ArUco Setup
        self.aruco_dicts = [
            aruco.Dictionary_get(aruco.DICT_4X4_50),
            aruco.Dictionary_get(aruco.DICT_5X5_100),
            aruco.Dictionary_get(aruco.DICT_6X6_250),
            aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
            ]
        self.aruco_params = aruco.DetectorParameters_create()
        
        # Mapping Data
        self.mapping_ids = [1, 2, 3, 4]
        self.buffers = {mid: [] for mid in self.mapping_ids}
        self.buffer_size = 10
        
        # Localization Data
        self.landmarks = self.load_landmarks()
        # Initialize to None; wait for info_callback to populate from ROS CameraInfo topic
        self.camera_matrix = None
        self.dist_coeffs = None
        self.has_info = False
        
        # Subscribers
        self.info_sub = rospy.Subscriber(self.camera_name+"/camera_info", CameraInfo, self.info_callback)
        self.image_sub = rospy.Subscriber(self.camera_name+"/image_raw", Image, self.image_callback)
        
        # Publishers
        self.pose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=10)
        self.waypoint_pub = rospy.Publisher("/aruco_waypoints", PoseStamped, queue_size=10)
        self.rviz_pub = rospy.Publisher("/aruco_markers_rviz", Marker, queue_size=10)
        self.debug_image_pub = rospy.Publisher("/csi_cam_0/image_raw", Image, queue_size=5)
        self.debug_compressed_pub = rospy.Publisher("/csi_cam_0/image_raw/compressed", CompressedImage, queue_size=5)
        
        # Services
        self.toggle_srv = rospy.Service("~toggle_mode", Trigger, self.handle_toggle_mode)
        self.toggle_mapping_srv = rospy.Service("~toggle_mapping", Trigger, self.handle_toggle_mapping)
        self.toggle_localization_srv = rospy.Service("~toggle_localization", Trigger, self.handle_toggle_localization)
        
        rospy.loginfo("ArUco Processor Started (Map: %s, Localize: %s)",
                      self.enable_mapping, self.enable_localization)

    def handle_toggle_mode(self, req):
        # Toggle between mapping and localization
        if self.enable_mapping:
            self.enable_mapping = False
            self.enable_localization = True
            msg = "Switched to Localization Mode (Mapping: OFF, Localization: ON)"
        else:
            self.enable_mapping = True
            self.enable_localization = False
            msg = "Switched to Mapping Mode (Mapping: ON, Localization: OFF)"
        
        rospy.loginfo(msg)
        return TriggerResponse(success=True, message=msg)

    def handle_toggle_mapping(self, req):
        self.enable_mapping = not self.enable_mapping
        msg = "Mapping is now " + ("ON" if self.enable_mapping else "OFF")
        rospy.loginfo(msg)
        return TriggerResponse(success=True, message=msg)

    def handle_toggle_localization(self, req):
        self.enable_localization = not self.enable_localization
        msg = "Localization is now " + ("ON" if self.enable_localization else "OFF")
        rospy.loginfo(msg)
        return TriggerResponse(success=True, message=msg)

    def load_landmarks(self):
        landmarks = {}
        if os.path.exists(self.yaml_path):
            try:
                with open(self.yaml_path, "r") as f:
                    data = yaml.safe_load(f)
                    if data and "waypoints" in data:
                        for wp in data["waypoints"]:
                            yaw = wp.get("yaw", 0.0)
                            T = tft.euler_matrix(0, 0, yaw)
                            T[0, 3] = wp["x"]
                            T[1, 3] = wp["y"]
                            landmarks[wp["id"]] = T
                rospy.loginfo("Loaded %d landmarks from YAML.", len(landmarks))
            except Exception as e:
                rospy.logwarn("Failed to load landmarks: %s", e)
        return landmarks

    def info_callback(self, msg):
        if not self.has_info:
            rospy.loginfo("Received Camera Info!")
        self.camera_matrix = np.array(msg.K).reshape(3, 3)
        self.dist_coeffs = np.array(msg.D).reshape(1, 5)
        self.has_info = True

    def image_callback(self, data):
        # Even without info, we want to publish the debug image for RViz
        self.counter += 1
        if self.counter < self.process_every_n_frames:
            return
        self.counter = 0

        try:
            # Use imgmsg_to_cv2 since we subscribed to raw Image
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
            
            # Only detect and estimate pose if we have valid camera info
            if not self.has_info:
                # Optionally publish raw image for debugging anyway
                if self.debug_image_pub.get_num_connections() > 0:
                    self.debug_image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
                return

            found_any = False
            
            # Search across all specified dictionaries
            for adict in self.aruco_dicts:
                corners, ids, _ = aruco.detectMarkers(gray, adict, parameters=self.aruco_params)
                
                if ids is not None:
                    found_any = True
                    rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.camera_matrix, self.dist_coeffs)
                    
                    # Draw ALL detected markers on the image for the presentation
                    aruco.drawDetectedMarkers(img, corners, ids)
                    for i in range(len(ids)):
                         marker_id = int(ids[i][0])
                         aruco.drawAxis(img, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.05)

                    for i in range(len(ids)):
                        marker_id = int(ids[i][0])
                        
                        # 1. LOCALIZATION LOGIC
                        if self.enable_localization and marker_id in self.landmarks:
                            self.localize_from_marker(marker_id, rvecs[i][0], tvecs[i][0])
                        
                        # 2. MAPPING LOGIC
                        if self.enable_mapping and marker_id in self.mapping_ids:
                            self.process_mapping(marker_id, rvecs[i][0], tvecs[i][0])

            # Publish debug images
            if self.debug_image_pub.get_num_connections() > 0:
                self.debug_image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
            if self.debug_compressed_pub.get_num_connections() > 0:
                self.debug_compressed_pub.publish(self.bridge.cv2_to_compressed_imgmsg(img))
                        
        except Exception as e:
            rospy.logerr_throttle(10, "ArUco Error: %s", e)

    def process_mapping(self, marker_id, rvec, tvec):
        try:
            # ArUco coords -> ROS coords (Z forward, X right, Y up)
            x_dist, y_dist, z_dist = tvec[0], tvec[1], tvec[2]
            
            local_pose = PoseStamped()
            local_pose.header.stamp = rospy.Time(0)
            local_pose.header.frame_id = self.camera_frame
            local_pose.pose.position.x = z_dist
            local_pose.pose.position.y = -x_dist
            local_pose.pose.position.z = -y_dist
            
            R, _ = cv.Rodrigues(rvec)
            T_4x4 = np.eye(4)
            T_4x4[:3, :3] = R
            quat = tft.quaternion_from_matrix(T_4x4)
            local_pose.pose.orientation.x = quat[0]
            local_pose.pose.orientation.y = quat[1]
            local_pose.pose.orientation.z = quat[2]
            local_pose.pose.orientation.w = quat[3]

            global_pose = self.tf_buffer.transform(local_pose, "map", rospy.Duration(0.3))
            self.buffers[marker_id].append(global_pose.pose)

            if len(self.buffers[marker_id]) >= self.buffer_size:
                # Weighted Average Logic: Give more weight to recent frames
                weights = range(1, self.buffer_size + 1)
                total_weight = sum(weights)
                
                avg_x = sum(p.position.x * w for p, w in zip(self.buffers[marker_id], weights)) / total_weight
                avg_y = sum(p.position.y * w for p, w in zip(self.buffers[marker_id], weights)) / total_weight
                
                final_quat = self.buffers[marker_id][-1].orientation
                
                final_pose = PoseStamped()
                final_pose.header.frame_id = str(marker_id).lstrip("/")
                final_pose.pose.position.x = avg_x
                final_pose.pose.position.y = avg_y
                final_pose.pose.orientation = final_quat
                
                self.waypoint_pub.publish(final_pose)
                self.publish_rviz_marker(marker_id, avg_x, avg_y, final_quat)
                self.buffers[marker_id] = []
                rospy.loginfo("MAPPED: Marker [%s] at X: %.2f, Y: %.2f", marker_id, avg_x, avg_y)

        except Exception as e:
            rospy.logwarn_throttle(10, "Mapping TF Error: %s", e)

    def localize_from_marker(self, marker_id, rvec, tvec):
        try:
            R_cl, _ = cv.Rodrigues(rvec)
            T_cl = np.eye(4)
            T_cl[:3, :3] = R_cl
            T_cl[0, 3] = tvec[0]
            T_cl[1, 3] = tvec[1]
            T_cl[2, 3] = tvec[2]

            trans = self.tf_buffer.lookup_transform(self.base_frame, self.camera_frame, rospy.Time(0), rospy.Duration(0.3))
            T_bc = self.pose_to_matrix(trans.transform)
            T_sl = self.landmarks[marker_id]
            T_lc = np.linalg.inv(T_cl)
            T_sb = np.dot(T_sl, np.dot(T_lc, np.linalg.inv(T_bc))) 
            self.publish_pose(T_sb, marker_id)
        except Exception as e:
            rospy.logwarn_throttle(5, "Localization TF Error: %s", e)

    def pose_to_matrix(self, transform):
        q = [transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w]
        T = tft.quaternion_matrix(q)
        T[0, 3] = transform.translation.x
        T[1, 3] = transform.translation.y
        T[2, 3] = transform.translation.z
        return T

    def publish_pose(self, T, marker_id):
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
        msg.pose.covariance = [0.1] * 36
        self.pose_pub.publish(msg)
        rospy.loginfo_throttle(5, "Pose reset by Marker %s", marker_id)

    def publish_rviz_marker(self, marker_id, x, y, quat):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "aruco"
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.05
        marker.pose.orientation = quat
        marker.scale.x = 0.08
        marker.scale.y = 0.08
        marker.scale.z = 0.08
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.rviz_pub.publish(marker)
1
if __name__ == "__main__":
    ArucoProcessor()
    rospy.spin()
