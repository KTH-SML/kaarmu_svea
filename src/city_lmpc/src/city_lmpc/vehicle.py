import numpy as np

import rospy
from geometry_msgs.msg import Point

from rosonic import SubProgram

import ros_abconnect as abconn

from svea_msgs.msg import VehicleState as VehicleStateMsg
from svea.states import VehicleState
from svea.data import RVIZPathHandler

from city_lmpc.pure_pursuit import PurePursuitSpeedController
from city_lmpc.arcs import Track, get_track, adjust_for_lane, TRACK_CHOOSER

DECA = {
    'obs': 10,
    'ego': 20,
    'onc': 30,
}

class VehicleInterface(SubProgram):

    name: str
    state: VehicleState

    sub_veh_state: abconn.Subscriber
    pub_veh_command: abconn.Publisher

    rviz: RVIZPathHandler

    pub_ctrl: abconn.Publisher
    track: Track

    controller: PurePursuitSpeedController

    V_REF = 0.6

    LOOK_AHEAD_BASE = 0.4
    LOOK_AHEAD_FACT = 0

    NUM_STATES = 4
    WINDOW_LEN = 5
    TIME_STEP = 0.1

    def __init__(self, name: str, downstream: bool, master_track: Track):

        self.timer = rospy.Rate(10)

        self.name = name
        self.state = VehicleState()

        self.rviz = RVIZPathHandler(self.name)

        self.lane = 'right' if downstream else 'left'
        self.downstream = downstream
        self.track = get_track(
            master_track.arcs,
            'center',
            (master_track.x_s0, master_track.y_s0, master_track.phi_s0),
            not downstream,
        )

        self.controller = PurePursuitSpeedController()

        self.sub_veh_state = abconn.Subscriber(
            self.name,
            DECA[self.name] + 0,
            '/abconnect/{veh}/state'.format(veh=self.name),
            VehicleStateMsg,
            self.state_cb,
        )
        rospy.loginfo('Created subscriber to %s (%d)',
                      '/abconnect/{veh}/state'.format(veh=self.name),
                      DECA[self.name] + 0)

        # Make sure we have a received at least one state
        self.state.state_msg = rospy.wait_for_message(
            '/abconnect/{veh}/state'.format(veh=self.name),
            VehicleStateMsg,
        )

        self.pub_ctrl = abconn.Publisher(
            self.name,
            DECA[self.name] + 3,
            '/{veh}/ctrl'.format(veh=self.name),
            Point,
            queue_size=1,
        )
        rospy.loginfo('Created publisher to %s (%d)',
                      '/{veh}/ctrl'.format(veh=self.name),
                      DECA[self.name] + 3)

        traj_x, traj_y = self.track.cartesian
        self.rviz.update_traj(traj_x, traj_y)
        self.rviz.log_state(self.state)
        self.rviz.visualize_data()

        # Empty spin
        self.spin = lambda _: None

        rospy.loginfo('VEHICLE INTERFACE CREATED (%s)', self.name)

    def send_ctrl(self, steering, velocity):
        msg = Point()
        msg.x = steering
        msg.y = velocity
        self.pub_ctrl.publish(msg)

    def state_cb(self, msg):
        self.state.state_msg = msg
        self.state_ctrl = msg.ctrl
        self.rviz.log_state(self.state)
        self.rviz.visualize_data()

    def main(self):
        while self.keep_alive():
            self.spin(self)
            self.timer.sleep()

