#! /usr/bin/env python3

import numpy as np

import rospy
from rosonic import Node, Parameter
from geometry_msgs.msg import Point, PoseStamped
from std_srvs.srv import Trigger, SetBool

from svea_msgs.msg import VehicleState as VehicleStateMsg
from svea.actuation import ActuationInterface
from svea.controllers.pure_pursuit import PurePursuitSpeedController
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


class track_svea(Node):

    # initial state
    NAME = Parameter("~name")
    X0 = Parameter("~x0")

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

    def publish_initialpose(self):
        from geometry_msgs.msg import PoseWithCovarianceStamped

        pub = rospy.Publisher(f'initialpose', PoseWithCovarianceStamped, queue_size=5)
        rate = rospy.Rate(5)

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

        for _ in range(10):
            pub.publish(msg)
            rate.sleep()

        rospy.loginfo('initialpose at [%f, %f, %f]', x, y, yaw)


    def __init__(self):

        self.state = VehicleState()
        self.enabled = False

        self.rviz = RVIZPathHandler()

        self.x = [self.X0[0], self.X0[1], self.X0[3]]
        self.controller = PurePursuitSpeedController()
        self.controller.target_velocity = self.V_REF

        self.actuation = ActuationInterface().start()
        self.start_sim()

        # Make sure we have a good state before continuing
        self.state.state_msg = rospy.wait_for_message('state', VehicleStateMsg)

        self.loginfo("subscribe to %s", "state")
        rospy.Subscriber("state",
                         VehicleStateMsg,
                         self.state_cb)

        self.publish_initialpose()

        self.loginfo("creating service %s", "enable_vehicle")
        rospy.Service(
            'enable_vehicle',
            SetBool,
            self.enable_vehicle_cb,
        )

        # Wait so we don't start before having something to follow
        self.loginfo("creating subscriber %s", "target")
        rospy.Subscriber(
            f'/rsu/{self.NAME}_target',
            Point,
            self.svea_target_cb
        )

        self.loginfo("waiting for %s", "target")
        self.svea_target_cb(rospy.wait_for_message(f'/rsu/{self.NAME}_target', Point))
        self.loginfo("all connected###")

        self.rviz.log_state(self.state)
        self.rviz.update_target(self.target)
        self.rviz.visualize_data()
        self.loginfo("init done")

    def state_cb(self, msg):
        self.state.state_msg = msg

    def svea_target_cb(self, msg):
        self.target = (msg.x, msg.y)
        self.controller.target_velocity = msg.z

    def enable_vehicle_cb(self, req):
        self.enabled = req.data
        return True, ''

    def keep_alive(self):
        return not self.is_shutdown()

    def spin(self):

        if not self.enabled:
            return

        # Compute control using path tracking MPC
        steering, velocity = self.controller.compute_control(self.state, self.target)
        transmission = 0

        # Build control signal to SVEA
        self.actuation.send_control(
            steering=steering, 
            velocity=velocity, 
            brake_force=0,
            transmission=transmission,
            differential_front=0, 
            differential_rear=0,
        )

        # Handle the publishing of RVIZ topics
        self.rviz.log_ctrl(steering, velocity, transmission, rospy.get_time())
        self.rviz.log_state(self.state)
        self.rviz.update_target(self.target)
        self.rviz.visualize_data()


if __name__ == '__main__':

    ## Start node ##

    track_svea(anonymous=True)
