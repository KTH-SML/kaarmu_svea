#! /usr/bin/env python3

import numpy as np

import rospy
from wp3_tests.msg import Packet
import message_filters as mf

def load_param(name, value=None):
    if value is None:
        assert rospy.has_param(name), f'Missing parameter "{name}"'
    return rospy.get_param(name, value)


class srvx:

    ALG_OUT_SIZE = 0 # [num of bytes]
    ALG_COMP_TIME = 0 # [s]

    def __init__(self):

        ## Initialize node

        rospy.init_node('srvx')

        ## Parameters

        self.CLIENTS = load_param('clients')
        self.ALG_OUT_SIZE = load_param('alg_out_size')
        self.ALG_COMP_TIME = load_param('alg_comp_time')

        self.packet_data = b'A' * self.ALG_OUT_SIZE


        # GENERATE RANDOM BYTEARRAY :)
        xs = np.random.rand(self.ALG_OUT_SIZE) * 255
        self.packet_data = bytes(map(int, xs))

        ## Publishers

        self.pubs = [
            rospy.Publisher(f'{client}/safe_set', Packet, queue_size=1)
            for client in self.CLIENTS
        ]

        ## Subscribers

        self.ts = mf.TimeSynchronizer([
            mf.Subscriber(f'{client}/state', Packet)
            for client in self.CLIENTS
        ], queue_size=10)
        self.ts.registerCallback(lambda *states: self.compute_alg(states))

    def compute_alg(self, _):

        rospy.sleep(self.ALG_COMP_TIME)

        msg = Packet()
        msg.header.stamp = rospy.Time.now()
        msg.data = self.packet_data

        for pub in self.pubs:
            pub.publish(msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':

    ## Start node ##

    srvx().run()

