#! /usr/bin/env python3

import numpy as np

import rospy
from wp3_tests.msg import Packet
import message_filters as mf

def load_param(name, value=None):
    if value is None:
        assert rospy.has_param(name), f'Missing parameter "{name}"'
    return rospy.get_param(name, value)


class vehx:

    NAME = 'vehx'
    START_DIST = 0 # [m]
    TRAVEL_VEL = 0 # [m/s]
    STATE_FREQ = 0 # [Hz]
    STATE_SIZE = 0 # [num of bytes]

    def __init__(self):

        ## Initialize node

        rospy.init_node('vehx')

        ## Parameters

        self.NAME = load_param('name')
        self.START_DIST = load_param('start_dist')
        self.TRAVEL_VEL = load_param('travel_vel')
        self.STATE_SIZE = load_param('state_size')
        self.STATE_FREQ = load_param('state_freq')

        xs = np.random.rand(self.STATE_SIZE) * 256
        self.state_content = bytes(map(int, xs))

        self.rate = rospy.Rate(self.STATE_FREQ)

        ## Publishers

        self.pub = rospy.Publisher(f'state', Packet, queue_size=1)

        ## Subscribers

        rospy.Subscriber(f'server/safe_set', Packet, self.safe_set_cb)

    def safe_set_cb(self, msg):
        pass

    def run(self):
        while self.keep_alive():
            msg = Packet()
            msg.header.stamp = rospy.Time.now()
            msg.data = self.state_content
            self.pub.publish(msg)
            self.rate.sleep()

    def keep_alive(self):
        return not rospy.is_shutdown()

if __name__ == '__main__':

    ## Start node ##

    srvx().run()

