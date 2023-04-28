#! /usr/bin/env python3

import json
from math import pi

import rospy
from std_msgs.msg import String

from svea import ActuationInterface

def load_param(name, value=None):
    if value is None:
        assert rospy.has_param(name), f'Missing parameter "{name}"'
    return rospy.get_param(name, value)

class SenseHatInterface(ActuationInterface):

    velocity: int   # [rad]
    steering: int   # [m/s]

    def __init__(self):

        self.velocity = 0
        self.steering = 0

        ## Parameters

        self.SENSEHAT_TOP = load_param('~sensehat_top', 'sensehat')
        self.UPDATE_FREQ = load_param('~update_freq', 10)

        ## Subscriber

        rospy.Subscriber(self.SENSEHAT_TOP, String, self.callback)

    def callback(self, msg):
        d = json.loads(msg.data)
        forward, right = d['forward'], d['right']
        self.steering = right * pi/5
        self.velocity = forward * 0.75

    def update(self):
        self.send_control(steering=-self.steering,
                          velocity=self.velocity)

