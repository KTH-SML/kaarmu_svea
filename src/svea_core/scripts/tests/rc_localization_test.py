#!/usr/bin/env python

import math
import rospy
from datetime import datetime

from geometry_msgs.msg import Twist

from svea.svea_managers.svea_archetypes import SVEAManager
from svea.localizers import LocalizationInterface as IndoorLocalizationInterface
from svea.localizers import MotionCaptureInterface
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

def param_init():
    # grab parameters from launch-file
    localization_method_param = rospy.search_param('localization_method')
    localization_method = rospy.get_param(localization_method_param, "")
    mocap_name_param = rospy.search_param('mocap_name')
    mocap_name = rospy.get_param(mocap_name_param, "")
    return localization_method, mocap_name

def main():
    rospy.init_node('SVEA_rc_localization_test')
    localization_method, mocap_name = param_init()

    if localization_method == "mocap":
        svea = SVEAManager(MotionCaptureInterface,
                           RCForwarder,
                           mocap_name = mocap_name)
    else:
        # currently default to indoor localization
        svea = SVEAManager(IndoorLocalizationInterface,
                           RCForwarder)

    # start manager of SVEA and RC remote
    rc = RCInterface().start()
    svea.start(wait=True)

    # control loop
    while not rospy.is_shutdown():
        state = svea.wait_for_state() # loop dictated by when state data comes in

        # read latest rc inputs
        rc_steering = rc.steering
        rc_velocity = rc.velocity
        rc_transmission = rc.gear

        # pass rc ctrl message, matching pattern from other scripts
        svea.controller.update_rc(rc_steering, rc_velocity, rc_transmission)
        steering, velocity, transmission = svea.compute_control()
        svea.send_control(steering, velocity, transmission)

        rospy.loginfo_throttle(1, state)

    rospy.spin()

    # save data logs
    now = datetime.now()
    time_str = now.strftime("_%B%d_%H:%M:%S")
    svea.data_handler.save_data("rc_localize_test"+time_str)

    # visualize what the trajectory looked like in the end
    svea.visualize_data()

if __name__ == '__main__':
    main()
