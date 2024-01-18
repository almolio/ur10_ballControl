#!/usr/bin/env python

import rospy
import sys
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from math import pi
from cv_bridge import CvBridge, CvBridgeError
import numpy as np 
from pathlib import Path
import matplotlib.pyplot as plt
import tf

# TNOTES: 
# /kinect2/hd/camera_info
# /kinect2/hd/image_color
# /kinect2/hd/image_color/compressed
# /kinect2/hd/image_color_rect
# /kinect2/hd/image_color_rect/compressed
# /kinect2/hd/image_depth_rect
# /kinect2/hd/image_depth_rect/compressed
# /kinect2/hd/image_mono
# /kinect2/hd/image_mono/compressed
# /kinect2/hd/image_mono_rect
# /kinect2/hd/image_mono_rect/compressed
# /kinect2/hd/points

CALIBRATION_STEPS = 150

### KINECT
# COLOR_TOPIC = "/kinect2/hd/image_color"
# CAMERA_INFO_TOPIC = "/kinect2/hd/camera_info"
# CAMERA_FRAME = "kinect2_rgb_optical_frame"

### PRIMESENSE
COLOR_TOPIC = "/camera/rgb/image_raw"
CAMERA_INFO_TOPIC = "/camera/rgb/camera_info"
CAMERA_FRAME = "camera_rgb_optical_frame"

class arucodetection:
    # adapted from https://pyimagesearch.com/2020/12/21/detecting-aruco-markers-with-opencv-and-python/
    # pose estimation adapted from https://github.com/GSNCodes/ArUCo-Markers-Pose-Estimation-Generation-Python/blob/main/pose_estimation.py

    def __init__(self):
        self.troubleshoot = False
        
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(COLOR_TOPIC,
                                        Image,self.imgcallback)
        self.cam_info_sub = rospy.Subscriber(CAMERA_INFO_TOPIC,
                                        CameraInfo,self.infocallback)

        self.pub = rospy.Publisher('aruco_coordinate', String, queue_size=10)
        self.tfbroadcaster = tf.TransformBroadcaster()
        self.camera_matrix  = np.asarray([ 1.0599465578241038e+03, 0., 9.5488326677588441e+02, 0.,
                                            1.0539326808799726e+03, 5.2373858291060583e+02, 0., 0., 1. ]).reshape([3,3])
        self.distortion_coefficients   = np.asarray([ 5.6268441170930321e-02, -7.4199141308694802e-02,
                                                        1.4250797540545752e-03, -1.6951722389720336e-03,
                                                                            2.4107681263086548e-02 ])

        self.cx = 0
        self.cy = 0
        self.tvec = 0
        self.rvec = 0
        # DICT_5X5_1000=7
        self.arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
        self.collected_poses = []
        self.calibrated = False
        self.calibrated_pose = None

    def infocallback(self,info):
        # print(info)
        self.camera_matrix = np.asarray(info.K).reshape([3,3])
        self.distortion_coefficients = info.D
        if self.troubleshoot:
            #print(self.camera_matrix)
            #print(self.distortion_coefficients)
            pass

    def preprocess_image(self, image):
        def adjust_gamma(image, gamma=1.0):
            invGamma = 1.0 / gamma
            table = np.array([((i / 255.0) ** invGamma) * 255
                for i in np.arange(0, 256)]).astype("uint8")
            return cv2.LUT(image, table)
        return adjust_gamma(image, gamma=3.0)    # 2.0 is a good value for the asus sensor. lower values make the image darker, higher values make it brighter

    def sendimag(self):
        if len(self.collected_poses) >= CALIBRATION_STEPS:
            if not self.calibrated:
                self.collected_poses = np.array(self.collected_poses)
                self.calibrated_pose = self.collected_poses.mean(axis=0)
                print("Finished calibration")
                self.calibrated = True

            # Just publish mean pose after calibration is done
            x, y, z = self.calibrated_pose[:3]

            # Converting rotation vector 
            rvec = self.calibrated_pose[3:]

            # Converting rvec to Rodrigues
            rmat, _ = cv2.Rodrigues(rvec)
            rmat_se3 = np.eye(4)
            rmat_se3[:3, :3] = rmat
            self.tfbroadcaster.sendTransform(
                translation=[x+0.02, y-0.05, z - 0.2], 
                rotation=tf.transformations.quaternion_from_matrix(rmat_se3),
                time = rospy.get_rostime(),
                child = 'ARUCOFRAME',
                parent=CAMERA_FRAME
            )
            
            return

    def imgcallback (self, image):
        if len(self.collected_poses) < CALIBRATION_STEPS:

            try: # if there is an image
            # Acquire the image, and convert to single channel gray image
                raw_img = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")

                raw_img = self.preprocess_image(raw_img)

                arucoParams = cv2.aruco.DetectorParameters_create()
                (corners, ids, rejected) = cv2.aruco.detectMarkers(raw_img, self.arucoDict,
                    parameters=arucoParams)

                # print(corners)

                image_aruco = raw_img.copy()
                
                if len(corners) > 0:
                    # flatten the ArUco IDs list
                    ids = ids.flatten()
                    
                    # Draw the corners and ids
                    image_aruco = self.markerboundary(image_aruco, corners, ids)

                    # Call this function to take in the coordinate frame on top of the Aruco
                    image_aruco = self.pose_estimation(image_aruco,corners,ids, self.camera_matrix, self.distortion_coefficients)

                    x, y, z = self.tvec.squeeze()
                    self.collected_poses.append([x, y, z, *self.rvec.squeeze()])
                print('Calibration step : {} / {}'.format(len(self.collected_poses), CALIBRATION_STEPS))

            except CvBridgeError as e:
                print (e)

    def markerboundary(self,image_aruco, corners, ids):
        for (markerCorner, markerID) in zip(corners, ids):
            # extract the marker corners (which are always returned in
            # top-left, top-right, bottom-right, and bottom-left order)
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            # convert each of the (x, y)-coordinate pairs to integers
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))

            cv2.line(image_aruco, topLeft, topRight, (255, 0, 0), 2)
            cv2.line(image_aruco, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(image_aruco, bottomRight, bottomLeft, (0, 0, 255), 2)
            cv2.line(image_aruco, bottomLeft, topLeft, (0, 255, 0), 2)

            # compute and draw the center (x, y)-coordinates of the ArUco
            # marker
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            cv2.circle(image_aruco, (cX, cY), 4, (0, 0, 255), -1)
            # draw the ArUco marker ID on the image
            # cv2.putText(image_aruco, 'id = {}'.format(markerID),
            #     (cX, cY), cv2.FONT_HERSHEY_SIMPLEX,
            #     1, (0, 255, 0), 2)
            self.cx = cX
            self.cy = cY
        return image_aruco
         

    def pose_estimation(self, frame, corners,ids, camera_matrix, distortion_coefficients):
        # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
        for i in range(len(ids)):
            self.rvec, self.tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.150, camera_matrix,
                                                                        distortion_coefficients)

            # Draw Axis
            cv2.aruco.drawAxis(frame, camera_matrix, distortion_coefficients, self.rvec, self.tvec, 0.1)  

        # Show the image
        if self.troubleshoot:
            cv2.imshow('frame', frame)
            cv2.waitKey(1)

        return frame

    def broadcast_aruco_to_base(self):
        # up y 15cm , and z 12cm back. rotate -90 along x 
        # http://docs.ros.org/en/jade/api/tf/html/python/transformations.html
        self.tfbroadcaster.sendTransform(
                    translation=[0, 0.15, -0.12], 
                    rotation= tf.transformations.quaternion_from_euler(-pi/2, 0, 0, axes='sxyz'), 
                    time = rospy.get_rostime(),
                    child = 'world',
                    parent= 'ARUCOFRAME')

def shutdown_hook(): 
    print("Shutting down")
    cv2.destroyAllWindows()

def main(args):

    rospy.init_node('ArucoBroadcast', anonymous=True)
    ic = arucodetection()

    while not rospy.is_shutdown():
        ic.broadcast_aruco_to_base()
        ic.sendimag()
    rospy.on_shutdown(shutdown_hook)


if __name__ == '__main__':
	main(sys.argv)