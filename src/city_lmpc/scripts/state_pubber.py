#! /usr/bin/env python3

from rosonic import Node, Rate

import rospy
from geometry_msgs.msg import PointStamped

from svea_msgs.msg import VehicleState as VehicleStateMsg
from svea.states import VehicleState
from svea.data import RVIZPathHandler


class state_pubber(Node):

    def __init__(self):

        self.rviz = RVIZPathHandler()
        self.state = VehicleState()
        self.state.state_msg = rospy.wait_for_message('state', VehicleStateMsg)
        rospy.Subscriber("state",
                         VehicleStateMsg,
                         lambda msg: setattr(self.state, 'state_msg', msg))

        self.sub_clicked_point = rospy.Subscriber('/clicked_point',
                                                  PointStamped,
                                                  lambda msg: self.log('Clikcked point: [%f, %f]', msg.x, msg.y))
        self.loginfo("init done")

    @Rate(10)
    def spin(self):
        self.log('[%f, %f, %f]', self.state.x, self.state.y, self.state.yaw)
        self.rviz.log_state(self.state)
        self.rviz.visualize_data()


if __name__ == '__main__':

    state_pubber()
