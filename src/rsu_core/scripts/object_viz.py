#! /usr/bin/env python3

import numpy as np

import rospy
from visualization_msgs.msg import Marker

from rsu_msgs.msg import StampedObjectPoseArray


def load_param(name, value=None):
    if value is None:
        assert rospy.has_param(name), f'Missing parameter "{name}"'
    return rospy.get_param(name, value)

def replace_base(old, new):
    split_last = lambda xs: (xs[:-1], xs[-1])
    is_private = new.startswith('~')
    is_global = new.startswith('/')
    assert not (is_private or is_global)
    ns, _ = split_last(old.split('/'))
    ns += new.split('/')
    return '/'.join(ns)


class object_viz:

    def __init__(self):

        ## Initialize node

        rospy.init_node('object_viz')

        ## Parameters

        self.TOPIC_OBJECTPOSES = load_param('~objectposes')
        self.TOPIC_VIZ_OBJECTS = load_param('~object_markers', 'object_markers')

        self.initial = True

        ## Publishers

        self.pub = rospy.Publisher(self.TOPIC_VIZ_OBJECTS, Marker, queue_size=10)
        rospy.loginfo(self.TOPIC_VIZ_OBJECTS)

        ## Subscribers

        rospy.Subscriber(self.TOPIC_OBJECTPOSES, StampedObjectPoseArray, self.callback)
        rospy.loginfo(self.TOPIC_OBJECTPOSES)

        self.pub.publish(Marker(ns='object_detect', id=0, type=Marker.SPHERE_LIST, action=Marker.ADD))

    def run(self):
        rospy.spin()

    def callback(self, objectposes_array):

        marker = Marker()
        marker.header = objectposes_array.header
        marker.ns = 'object_detect'
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.MODIFY
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.r = 1
        marker.color.b = 1
        marker.color.a = 1
        marker.lifetime = rospy.Duration(0, 500 * 10**6)

        self.initial = False

        points = []

        for obj in objectposes_array.objects:
            points.append(obj.pose.pose.position)

        marker.points = points
        marker.colors = [marker.color] * len(points)
        self.pub.publish(marker)

if __name__ == '__main__':

    ##  Start node  ##

    object_viz().run()

