#! /usr/bin/env python

import numpy as np

import rospy
from visualization_msgs.msg import Marker

from rosonic import Node, Parameter
from rsu_msgs.msg import StampedObjectPoseArray


class object_viz(Node):

    ## Topics

    TOPIC_OBJECTPOSES = Parameter('~objectposes')
    TOPIC_VIZ_OBJECTS = Parameter('~object_markers', 'object_markers')


    def __init__(self):

        self.initial = True

        ## Publishers

        self.pub = rospy.Publisher(self.TOPIC_VIZ_OBJECTS, Marker, queue_size=10)
        self.log_event(self.TOPIC_VIZ_OBJECTS)

        ## Subscribers

        rospy.Subscriber(self.TOPIC_OBJECTPOSES, StampedObjectPoseArray, self.callback)
        self.log_event(self.TOPIC_OBJECTPOSES)

        self.pub.publish(Marker(ns='object_detect', id=0, type=Marker.SPHERE_LIST, action=Marker.ADD))

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

    object_viz()

