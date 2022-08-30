#!/usr/bin/env python

import rospy
import numpy as np

from svea.states import VehicleState
from svea.svea_managers.svea_archetypes import SVEAManager
from svea.localizers import LocalizationInterface
from svea.teleop import SafeTeleopInterface
from svea.data import TrajDataHandler, RVIZPathHandler
from svea.models.bicycle import SimpleBicycleModel
from svea.simulators.sim_SVEA import SimSVEA

default_init_pt = [0.0, 0.0, 0.0, 0.0] # [x, y, yaw, v], units: [m, m, rad, m/s]

class SpeedController(object):

    DT = 1.0/80.0
    K_p = 1.0  # speed control propotional gain
    K_i = 0.2  # speed control integral gain
    MAX_VEL = 0.3

    def __init__(self, vehicle_name=''):
        self.target_velocity = 0.0
        self.error_sum = 0.0

        self.steer = 0.0
        self.accel = 0.0

    def update_input(self, steer, accel):
        self.steer = steer
        self.accel = accel

    def compute_control(self, state):
        # update target velocity
        new_vel = self.target_velocity + self.accel*self.DT
        sign = np.sign(new_vel)
        self.target_velocity = sign * min(self.MAX_VEL, abs(new_vel))

        # speed control
        error = self.target_velocity - state.v
        self.error_sum += error * self.DT
        P = error * self.K_p
        I = self.error_sum * self.K_i
        correction = P + I
        return self.steer, self.target_velocity + correction


def param_init():
    """Initialization handles use with just python or in a launch file
    """
    # grab parameters from launch-file
    start_pt_param = rospy.search_param('start_pt')
    is_sim_param = rospy.search_param('is_sim')
    use_rviz_param = rospy.search_param('use_rviz')

    start_pt = rospy.get_param(start_pt_param, default_init_pt)
    if isinstance(start_pt, str):
        start_pt = start_pt.split(',')
        start_pt = [float(curr) for curr in start_pt]
        start_pt = VehicleState(*start_pt)

    is_sim = rospy.get_param(is_sim_param, True)
    use_rviz = rospy.get_param(use_rviz_param, False)

    return start_pt, is_sim, use_rviz


def main():
    rospy.init_node('svea_node')
    start_pt, is_sim, use_rviz = param_init()

    if is_sim:
        model_for_sim = SimpleBicycleModel(start_pt)
        SimSVEA(model_for_sim, dt=0.01,
                run_lidar=False, start_paused=False).start()

    if use_rviz:
        DataHandler = RVIZPathHandler
    else:
        DataHandler = TrajDataHandler

    # start manager of SVEA and RC remote
    svea = SVEAManager(LocalizationInterface,
                       SpeedController,
                       data_handler = DataHandler)
    svea.start(wait=True)
    safe_teleop_interface = SafeTeleopInterface().start()

    # control loop
    rate = rospy.Rate(80) # 80Hz
    while not rospy.is_shutdown():
        svea.wait_for_state()

        steer = safe_teleop_interface.drive_input.steering
        accel = safe_teleop_interface.drive_input.acceleration
        svea.controller.update_input(steer, accel)
        steering, velocity = svea.compute_control()
        svea.send_control(steering, velocity)

        if use_rviz:
            svea.visualize_data()

        rate.sleep()

    rospy.spin()

if __name__ == '__main__':
    main()
