#!/usr/bin/env python

import math
import rospy
from datetime import datetime

from geometry_msgs.msg import Twist

from svea.svea_managers.svea_archetypes import SVEAManager
from svea.localizers import LocalizationInterface
from svea.teleop import RCInterface


class RCForwarder(object):
    def __init__(self, vehicle_name=''):
        self.steering = 0
        self.velocity = 0
        self.transmission = 0

    def update_rc(self, steering, velocity, transmission):
        self.steering = steering
        self.velocity = velocity
        self.transmission = transmission

    def compute_control(self, state):
        return self.steering, self.velocity, self.transmission


def main():
    rospy.init_node('SVEA_rc')

    # start manager of SVEA and RC remote
    rc = RCInterface().start()
    svea = SVEAManager(LocalizationInterface,
                       RCForwarder)
    svea.start(wait=True)

    # control loop
    rate = rospy.Rate(80) # 80Hz
    while not rospy.is_shutdown():

        # read latest rc inputs
        rc_steering = rc.steering
        rc_velocity = rc.velocity
        rc_transmission = rc.gear

        # pass rc ctrl message, matching pattern from other scripts
        # TODO: can be easily simplified by implementing new manager
        svea.controller.update_rc(rc_steering, rc_velocity, rc_transmission)
        steering, velocity, transmission = svea.compute_control()
        svea.send_control(steering, velocity, transmission)

        # print current rc input (in human-readable format)
        rospy.loginfo_throttle(1, rc)

        rate.sleep()

    # now = datetime.now()
    # time_str = now.strftime("_%B%d_%H:%M:%S")
    # svea.data_handler.save_data("rc"+time_str)

    rospy.spin()

if __name__ == '__main__':
    main()
