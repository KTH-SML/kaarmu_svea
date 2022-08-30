#!/usr/bin/env python

import sys
import os
import rospy

dirname = os.path.dirname(__file__)
svea = os.path.join(dirname, '../../')
sys.path.append(os.path.abspath(svea))

from svea.actuation import ActuationInterface
from svea.teleop import LogitechInterface
from math import pi


def main():
    rospy.init_node('SVEA_remote_teleop')

    sampling_hz = 50
    r = rospy.Rate(sampling_hz)

    ctrl_interface = ActuationInterface().start(wait = True)
    teleop_interface = LogitechInterface().start()

    while not rospy.is_shutdown():
        curr_teleop = teleop_interface.teleop_cmd

        hum_steer = float(curr_teleop.steer_percent)/100.0
        hum_gas = float(curr_teleop.gas_percent)/100.0
        hum_steering = pi/4 * hum_steer
        hum_vel = 1.5 * hum_gas
        if curr_teleop.gear_name == "reverse":
            hum_vel = -hum_vel

        ctrl_interface.send_control(steering = hum_steering,
                                    velocity = hum_vel,
                                    transmission = 0)

        r.sleep()

    while not rospy.is_shutdown():
        ctrl_interface.send_control(0, 0)

    rospy.spin()

if __name__ == '__main__':
    main()
