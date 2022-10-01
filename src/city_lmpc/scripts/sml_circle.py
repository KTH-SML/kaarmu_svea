#! /usr/bin/env python3

from math import hypot

from rosonic import Node, Rate

import rospy
from geometry_msgs.msg import PointStamped

from svea_msgs.msg import VehicleState as VehicleStateMsg
from svea.states import VehicleState
from svea.data import RVIZPathHandler
from svea.actuation import ActuationInterface
from svea.controllers.pure_pursuit import PurePursuitSpeedController


class sml_circle(Node):

    VEL_REF = 0.5
    TGT_THRESH = 0.1
    TRACK = [
        (-2.28, -1.73),
        (+2.52, -1.65),
        (-2.37, +1.74),
        (+2.28, +2.00),
    ]

    def __init__(self):

        self.rviz = RVIZPathHandler()

        self.state = VehicleState()
        self.state.state_msg = rospy.wait_for_message('state', VehicleStateMsg)
        rospy.Subscriber("state",
                         VehicleStateMsg,
                         lambda msg: setattr(self.state, 'state_msg', msg))

        self.actuation = ActuationInterface().start(wait=True)

        self.controller = PurePursuitSpeedController()
        self.controller.target_velocity = self.VEL_REF

        self._target_index = 0

        self.loginfo("init done")

    def spin(self):
        if self.is_close():
            self.update_target()
        else:
            self.loginfo(f'distance: {self.dist_to_target()}')

        steering, velocity = self.controller.compute_control(self.state, self.target)

        self.actuation.send_control(
            steering=steering,
            velocity=velocity,
        )

        self.rviz.log_ctrl(steering, velocity, rospy.get_time())
        self.rviz.log_state(self.state)
        self.rviz.visualize_data()

    def dist_to_target(self):
        x, y = self.state.x, self.state.y
        xt, yt = self.target
        dx, dy = map(abs, [xt-x, yt-y])
        return hypot(dx, dy) 

    def is_close(self):
        return self.dist_to_target() < self.TGT_THRESH

    def update_target(self):
        self._target_index += 1
        self._target_index %= len(self.TRACK)

        self.rviz.update_target(self.target)

    @property
    def target(self):
        return self.TRACK[self._target_index]
        


if __name__ == '__main__':

    sml_circle()
