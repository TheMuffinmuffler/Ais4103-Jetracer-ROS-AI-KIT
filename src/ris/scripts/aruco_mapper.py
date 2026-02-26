#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
import cv2 as cv
import numpy as np
import cv2.aruco as aruco
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

class ArucoMapper:
    def __init__(self):
        self.camera_name = rospy.get_param("~camera_name","csi_cam_0")
        self.bridge = CvBridge()

        self.camera_matrix = None
        self.dist_coeffs = None

        # Your measured ArUco marker size in meters
        self.marker_size = 0.038


        self.info_sub = rospy.Subscriber(self.camera_name+"/camera_info", CameraInfo, self.info_callback)

        self.image_sub = rospy.Subscriber(self.camera_name+"/image_raw/compressed", CompressedImage, self.callback)
        self.image_pub = rospy.Publisher("/aruco_video/compressed", CompressedImage, queue_size=10)

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
        if self.camera_matrix is None or self.dist_coeffs is None:
            return
        try:
            img = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # Loop through every dictionary in our list ---
        for current_dict in self.aruco_dicts:
        # Notice we use 'current_dict' here instead of 'self.aruco_dict'
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, current_dict, parameters=self.aruco_params)

            if ids is not None:
                aruco.drawDetectedMarkers(img, corners, ids)

            # Calculate the 3D distance using your custom lens math!
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                    corners, self.marker_size, self.camera_matrix, self.dist_coeffs)

                for i in range(len(ids)):
                    aruco.drawAxis(img, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], self.marker_size)

                    x_dist = tvecs[i][0][0]
                    y_dist = tvecs[i][0][1]
                    z_dist = tvecs[i][0][2]

                    print("Marker [{}]: is {:.2f}m left/right, {:.2f}m up/down, and {:.2f}m straight ahead!".format(
                    ids[i][0], x_dist, y_dist, z_dist))

        cv.imshow("ArUco Scanner", img)
        cv.waitKey(3)

        try:
            self.image_pub.publish(self.bridge.cv2_to_compressed_imgmsg(img))
        except CvBridgeError as e:
            print(e)

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