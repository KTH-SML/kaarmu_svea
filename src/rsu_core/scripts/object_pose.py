#! /usr/bin/env python

import numpy as np

import rospy
import message_filters as mf
from sensor_msgs.msg import Image, CameraInfo
from image_geometry import PinholeCameraModel

from rosonic import Node, Parameter, Name
from rsu_msgs.msg import StampedObjectArray, StampedObjectPoseArray, ObjectPose

class object_pose(Node):

    ## Topics

    SUB_DEPTH_IMAGE = Parameter('~sub_depth_image', 'depth_image')
    SUB_OBJECTS = Parameter('~sub_objects', 'objects')
    PUB_OBJECTPOSES = Parameter('~pub_objectposes', 'objectposes')

    SUB_CAMERA_INFO = (
        SUB_DEPTH_IMAGE.copy()
        .then(Name('camera_info').next_to)
        .then(str)
    )


    def __init__(self):

        ## Camera model

        self.camera_model = PinholeCameraModel()

        ## Publishers

        self.pub_objectposes = rospy.Publisher(str(self.PUB_OBJECTPOSES), StampedObjectPoseArray, queue_size=10)
        self.log_event(self.PUB_OBJECTPOSES)

        ## Subscribers

        self.ts = mf.TimeSynchronizer([
            mf.Subscriber(self.SUB_OBJECTS, StampedObjectArray),
            mf.Subscriber(self.SUB_DEPTH_IMAGE, Image),
            mf.Subscriber(self.SUB_CAMERA_INFO, CameraInfo),
        ], queue_size=10)
        self.ts.registerCallback(self.callback)

        self.log_event(self.SUB_OBJECTS)
        self.log_event(self.SUB_DEPTH_IMAGE)

    def callback(self, object_array, image, camera_info):

        ## Load camera info

        self.camera_model.fromCameraInfo(camera_info)

        ## Prepare depth map

        depth_map = np.frombuffer(image.data, dtype=np.float32).reshape(image.height, image.width)
        H, W = depth_map.shape[:2]

        ## Project pixel to 3D coordinate for each object

        objects = []

        for obj in object_array.objects:

            ## Get depth of object
            # 1. create a mask for the region of interest
            # 2. Segment by thresholding (pick out the foreground)
            # 3. Save mean of foreground as distance

            u1 = obj.roi.x_offset
            v1 = obj.roi.y_offset
            u2 = u1 + obj.roi.width
            v2 = v1 + obj.roi.height

            roi_mask = np.zeros((H, W), dtype=np.bool)
            roi_mask[v1:v2, u1:u2] = True

            # Get only usuable depths of the roi
            roi_mask[np.isnan(depth_map)] = False
            roi_mask[np.isinf(depth_map)] = False

            if not roi_mask.sum():
                continue

            # threshold = mean of masked area
            segm_mask = depth_map[roi_mask] < depth_map[roi_mask].mean()

            if not segm_mask.sum():
                continue

            # take mean of the segment as distance
            d = depth_map[roi_mask][segm_mask].mean()

            ## Projection
            # 1. take middle pixel of region of interest
            # 2. get unit vector of projection by `camera_model.projectPixelTo3dRay()`
            # 3. multiply unit vec by distance to get real world coordinates in camera's frame

            u = obj.roi.x_offset + obj.roi.width // 2
            v = obj.roi.y_offset + obj.roi.height // 2

            ray = self.camera_model.projectPixelTo3dRay((u, v))
            x, y, z = np.array(ray) * d

            objpose = ObjectPose()
            objpose.object = obj
            ## NOTE: Message supports these
            # objpose.pose.covariance = ... # float64
            # objpose.pose.pose.orientation = ... # geometry_msgs/Quaternion
            objpose.pose.pose.position.x = x
            objpose.pose.pose.position.y = y
            objpose.pose.pose.position.z = z

            objects.append(objpose)

        ## Publish

        if objects:

            objectpose_array = StampedObjectPoseArray()
            objectpose_array.header = object_array.header
            objectpose_array.objects = objects

            self.pub_objectposes.publish(objectpose_array)


if __name__ == '__main__':

    ##  Start node  ##

    object_pose()
