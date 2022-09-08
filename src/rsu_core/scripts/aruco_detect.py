#! /usr/bin/env python

import os

import numpy as np
import cv2
from cv2 import aruco

import rospy
import tf2_ros
import tf_conversions
import message_filters as mf
from sensor_msgs.msg import Image, CameraInfo
from aruco_msgs.msg import Marker
from geometry_msgs.msg import TransformStamped, Point, Quaternion
from cv_bridge import CvBridge

from rosonic import Node, Parameter, Name


class aruco_detect(Node):

    ## Topics

    SUB_IMAGE = Parameter('~sub_image')
    PUB_ARUCO_POSE = Parameter('~pub_aruco_pose', 'aruco_pose')

    SUB_CAMERA_INFO = (
        SUB_IMAGE.copy()
        .then(Name('camera_info').next_to)
        .then(str)
    )

    ## Auxiliary

    ARUCO_DICT_NAME = Parameter('~aruco_dict', 'DICT_4X4_250')
    ARUCO_SIZE = Parameter('~aruco_size', '0.05')
    ARUCO_TF_NAME = Parameter('~aruco_tf_name', 'aruco')


    def __init__(self):

        ## Aruco

        self.aruco_size = float(self.ARUCO_SIZE)

        dict_name = getattr(aruco, self.ARUCO_DICT_NAME)
        self.aruco_dict = aruco.Dictionary_get(dict_name)

        ## TF2

        self.br = tf2_ros.TransformBroadcaster()

        ## Publishers

        self.pub_aruco_pose = rospy.Publisher(self.PUB_ARUCO_POSE, Marker, queue_size=5)
        self.log_event(self.PUB_ARUCO_POSE)


        ## Subscribers

        ts = mf.TimeSynchronizer([
            mf.Subscriber(self.SUB_IMAGE, Image),
            mf.Subscriber(self.SUB_CAMERA_INFO, CameraInfo),
        ], queue_size=1)
        ts.registerCallback(self.callback)
        self.log_event(self.SUB_IMAGE)

    def callback(self, image, camera_info):

        now = rospy.Time.now()

        # convert to grayscale
        gray = bridge.imgmsg_to_cv2(image, 'mono8')

        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict)

        if ids is None:
            return

        rvecs, tvecs = aruco.estimatePoseSingleMarkers(
            corners,
            self.aruco_size,
            np.array(camera_info.K).reshape((3, 3)),          # camera matrix
            np.array(camera_info.D).reshape((1, 5)),          # camera distortion
        )[:2] # [:2] due to python2/python3 compatibility

        for aruco_id, rvec, tvec in zip(ids, rvecs, tvecs):

            mtx = np.zeros((4, 4))
            mtx[:3, :3] = cv2.Rodrigues(rvec)[0]
            mtx[:3, 3] = tvec
            mtx[3, 3] = 1
            translation = tf_conversions.transformations.translation_from_matrix(mtx)
            rotation = tf_conversions.transformations.quaternion_from_matrix(mtx)

            ## Broadcast

            t = TransformStamped()
            t.header = image.header
            t.child_frame_id = self.ARUCO_TF_NAME + str(aruco_id)
            t.transform.translation = Point(*translation)
            t.transform.rotation = Quaternion(*rotation)

            self.br.sendTransform(t)

            ## Publish

            marker = Marker()
            marker.header = image.header
            marker.id = int(aruco_id)
            marker.pose.pose.position = Point(*translation)
            marker.pose.pose.orientation = Quaternion(*rotation)
            marker.confidence = 1 # NOTE: Set this to something more relevant?

            self.pub_aruco_pose.publish(marker)

if __name__ == '__main__':

    ##  Global resources  ##

    bridge = CvBridge()

    ##  Start node  ##

    aruco_detect.run()

