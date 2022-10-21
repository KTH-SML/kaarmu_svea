#! /usr/bin/env python3

import numpy as np

from rosonic import Node, Parameter, Rate
import ros_abconnect as abconn

import rospy
from geometry_msgs.msg import Point
from std_srvs.srv import Trigger, SetBool
from geometry_msgs.msg import PoseWithCovarianceStamped

from city_lmpc.vehicle import DECA
from city_lmpc.pure_pursuit import PurePursuitSpeedController

from svea_msgs.msg import VehicleState as VehicleStateMsg
from svea_msgs.msg import lli_ctrl
from svea.actuation import ActuationInterface
from svea.states import VehicleState
from svea.data import RVIZPathHandler
from svea.simulators.sim_SVEA import SimSVEA
from svea.models.bicycle import SimpleBicycleModel


def euler_to_quaternion(yaw, pitch, roll):
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - \
        np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + \
        np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - \
        np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + \
        np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    return [qx, qy, qz, qw]


class svea(Node):

    NAME = Parameter("~name")
    X0 = Parameter("~x0")
    IS_SIM = Parameter("~is_sim")

    V_REF = 0.5 # [m/s]
    V_REF_STRT = 0.5
    V_REF_CURV = 0.2

    LOOK_AHEAD_BASE = 0.32
    LOOK_AHEAD_FACT = 0

    NUM_STATES = 4
    WINDOW_LEN = 5
    TIME_STEP = 0.1

    def start_sim(self):
        self.state.x = self.X0[0]
        self.state.y = self.X0[1]
        self.state.v = self.X0[2]
        self.state.yaw = self.X0[3]
        self.sim_model = SimpleBicycleModel(self.state)
        self.simulator = SimSVEA(self.sim_model,
                                 dt=self.TIME_STEP,
                                 start_paused=False).start()

    def __init__(self):

        self.state = VehicleState()

        self.rviz = RVIZPathHandler()

        self.controller = PurePursuitSpeedController()

        self.actuation = ActuationInterface().start()

        if self.IS_SIM:
            self.start_sim()

        # Make sure we have a good state before continuing
        self.state.state_msg = rospy.wait_for_message('state', VehicleStateMsg)

        self.log("subscribe to %s", "state")
        rospy.Subscriber("state",
                         VehicleStateMsg,
                         lambda msg: setattr(self.state, 'state_msg', msg))

        self.pub_initialpose = rospy.Publisher(
            'initialpose',
            PoseWithCovarianceStamped,
            queue_size=1,
        )

        rospy.Subscriber('lli/remote',
                         lli_ctrl,
                         self.lli_remote_cb)

        rospy.wait_for_message('lli/remote', lli_ctrl)
        rospy.Timer(rospy.Duration(2), self.publish_initialpose)

        abconn.Publisher(
            'rsu',
            DECA[self.NAME] + 0,
            '/{veh}/state'.format(veh=self.NAME),
            VehicleStateMsg,
            queue_size=1,
        )

        self.sub_ctrl = abconn.Subscriber(
            'rsu',
            DECA[self.NAME] + 3,
            '/abconnect/rsu/{veh}/ctrl'.format(veh=self.NAME),
            Point,
            self.svea_ctrl_cb,
        )
        self.log('Created subscriber to %s (%d)',
                 '/abconnect/rsu/{veh}/ctrl'.format(veh=self.NAME),
                 DECA[self.NAME] + 3)

        rospy.wait_for_message(
            '/abconnect/rsu/{veh}/ctrl'.format(veh=self.NAME),
            Point,
        )

        self.log('Starting run')

    def lli_remote_cb(self, msg):
        self.ctrl = msg.ctrl
        if self.ctrl > 3:
            self.controller.reset()

    def publish_initialpose(self, _):
        if self.ctrl > 3:
            x, y, _, yaw = self.X0
            qx, qy, qz, qw = euler_to_quaternion(yaw, 0, 0)

            msg = PoseWithCovarianceStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = 'map'
            msg.pose.pose.position.x = x
            msg.pose.pose.position.y = y
            msg.pose.pose.position.z = 0
            msg.pose.pose.orientation.x = qx
            msg.pose.pose.orientation.y = qy
            msg.pose.pose.orientation.z = qz
            msg.pose.pose.orientation.w = qw
            self.pub_initialpose.publish(msg)

    def keep_alive(self):
        return not self.is_shutdown()

    def svea_ctrl_cb(self, msg: Point):

        # Encoding:
        steering = msg.x
        velocity = msg.y

        # Build control signal to SVEA
        self.actuation.send_control(
            steering=steering,
            velocity=velocity,
            brake_force=0,
            transmission=0,
            differential_front=0,
            differential_rear=0,
        )

        # Handle the publishing of RVIZ topics
        self.rviz.log_ctrl(steering, velocity, rospy.get_time())
        self.rviz.log_state(self.state)
        self.rviz.visualize_data()

if __name__ == '__main__':

    ## Start node ##

    svea()
