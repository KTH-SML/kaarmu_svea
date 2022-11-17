#! /usr/bin/env python3

from rosonic import Node, Parameter, OnShutdown

import rospy
import tf2_ros
from tf2_geometry_msgs import do_transform_pose

from rsu_msgs.msg import StampedObjectPoseArray

class rsu_node(Node):

    def __init__(self):

        self.tf_buf = tf2_ros.Buffer() 
        tf2_ros.TransformListener(self.tf_buf)

        self.sub_objectposes = rospy.Subscriber(
            '/rsu/objectposes',
            StampedObjectPoseArray,
            self.objectposes_cb,
            # queue_size=4,
            # buff_size=4*100,
        )

        self.pub_objectposes = rospy.Subscriber(
            '/rsu/objectposes_out',
            StampedObjectPoseArray,
            queue_size=10,
        )   

    def objectposes_cb(self, objectposes):

        origin_frame = objectposes.header.frame_id 

        if not self.tf_buf.can_transform(origin_frame, 'map', rospy.Time()):
            print('CANNOT TRANSFORM')
            return

        for objpose in objectposes.objects:

            transform = self.tf_buf.lookup_transform(origin_frame, 'map', rospy.Time())
            objpose.pose = do_transform_pose(objpose.pose, transform)

        self.pub_objectposes.publish(objectposes)

if __name__ == '__main__':

    ## Start node ##

    rsu_demo()
