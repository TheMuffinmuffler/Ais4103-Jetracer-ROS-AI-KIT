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

        # --- YOUR CUSTOM CALIBRATION MATH ---
        # Matrix 'K' from your calibration
        self.camera_matrix = np.array([
            [399.340134, 0.000000, 324.141394],
            [0.000000, 532.852065, 217.404681],
            [0.000000, 0.000000, 1.000000]
        ])

        # Distortion 'D' from your calibration
        self.dist_coeffs = np.array([[-0.322724, 0.097739, -0.000331, -0.001090, 0.0]])

        # Your measured ArUco marker size in meters
        self.marker_size = 0.038

        # Only listen to the video feed now, we already have the math!
        self.image_sub = rospy.Subscriber(self.camera_name+"/image_raw/compressed", CompressedImage, self.callback)
        self.image_pub = rospy.Publisher("/aruco_video/compressed", CompressedImage, queue_size=10)

        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters_create()

    def callback(self, data):
        try:
            img = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

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