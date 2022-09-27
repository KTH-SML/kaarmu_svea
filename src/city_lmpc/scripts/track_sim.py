#! /usr/bin/env python3

import math

import numpy as np
from casadi import DM, vertcat

import rospy
from rosonic import Node, Parameter
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path

from city_lmpc.arcs import get_track, init_adjust, ARCS, ARCS_BENDY
from city_lmpc.models.bicycle import BicycleModel
from city_lmpc.controllers.bicycle import BicycleMPC

from svea_msgs.msg import VehicleState as VehicleStateMsg
from svea.actuation import ActuationInterface
from svea.states import VehicleState
from svea.data import RVIZPathHandler
from svea.simulators.sim_SVEA import SimSVEA
from svea.models.bicycle import SimpleBicycleModel


class track_sim(Node):

    # [x_s0, y_s0, phi_s0]
    S0 = Parameter("~s0")
    LANE = Parameter("~lane", "center")
    FLIPPED_TRACK = Parameter("~flipped_track", False)

    PUB_TO_RVIZ = True

    V_REF_STRT = 0.3
    V_REF_CURV = 0.2

    LOOK_AHEAD_BASE = 0.32
    LOOK_AHEAD_FACT = 0

    NUM_STATES = 4
    WINDOW_LEN = 5
    TIME_STEP = 0.1

    def start_sim(self):
        self.state = VehicleState()
        self.state.x = self.x[0, 0]
        self.state.y = self.x[1, 0]
        self.state.v = self.x[2, 0]
        self.state.yaw = self.x[3, 0]
        self.sim_model = SimpleBicycleModel(self.state)
        self.simulator = SimSVEA(self.sim_model,
                                 dt=self.TIME_STEP,
                                 run_lidar=True,
                                 start_paused=False).start()

    def __init__(self):

        self.clicked_point = None
        self.initialpose = None

        rospy.Subscriber("state", 
                         VehicleStateMsg, 
                         lambda msg: setattr(self.state, 'state_msg', msg))
        rospy.Subscriber("clicked_point", 
                         PointStamped, 
                         lambda msg: setattr(self, 'clicked_point', msg))
        rospy.Subscriber('initialpose',
                         PoseWithCovarianceStamped,
                         lambda msg: setattr(self, 'initialpose', msg))
        self.pred_path_pub = rospy.Publisher("pred_path", Path, queue_size=1, latch=True)

        if self.PUB_TO_RVIZ:
            self.rviz_handler = RVIZPathHandler()

        # Track
        
        rate = rospy.Rate(1)
        while self.clicked_point is None:

            rate.sleep()

            if self.initialpose:
                self.S0 = (
                    self.initialpose.pose.pose.position.x,
                    self.initialpose.pose.pose.position.y,
                    self.quaternion_to_euler((
                        self.initialpose.pose.pose.orientation.x,
                        self.initialpose.pose.pose.orientation.y,
                        self.initialpose.pose.pose.orientation.z,
                        self.initialpose.pose.pose.orientation.w,
                    ))[0],
                )
                self.loginfo(str(self.S0))

            self.x_s0, self.y_s0, self.phi_s0 = init_adjust(self.LANE, *self.S0)

            self.track = get_track(
                ARCS_BENDY,
                self.LANE,
                self.S0,
                self.FLIPPED_TRACK,
            ) 

            if self.PUB_TO_RVIZ:
                traj_x, traj_y = self.track.cartesian
                self.rviz_handler.update_traj(traj_x, traj_y)

        # Model Parameters
        point = self.track.points[0]
        self.model = BicycleModel(
            x0=[point.x_s, point.y_s, 0, self.phi_s0],
            dt=self.TIME_STEP
        )
        self.x = self.model.x0

        # MPC Parameters
        self.controller = BicycleMPC(
            self.model,
            N=self.WINDOW_LEN,
            Q=[5, 5, 50, 7],
            R=[1, 2],
            xlb=[-100, -100, -0.5, -2*np.pi],
            xub=[100, 100, 0.6, 2*np.inf],
            ulb=[-1, -np.pi / 5],
            uub=[1.5, np.pi / 5],
        )

        self.actuation = ActuationInterface().start()
        self.start_sim()

    def keep_alive(self):
        diff = self.x[:2, 0] - self.track.points_array[:2, -1]
        return not self.is_shutdown() and np.linalg.norm(diff) > 0.5

    def spin(self):
        # Get local frenet coodinates
        self.x = vertcat(self.state.x, self.state.y,
                         self.state.v, self.state.yaw)
        # Compute reference trajectory and track parameters over the reference
        ref = self.get_trajectory(self.x)

        # Compute control using path tracking MPC
        u, pred = self.controller.get_ctrl(self.x, ref)
        # Log velocity information
        # rospy.loginfo_throttle(0.5, "v_pred {} v {}, V_REF_STRT {}, v_err {}".format(pred[2, 0], self.state.v,
        #                        ref[2, 0], ref[2, 0] - self.state.v))

        # Build control signal to SVEA
        velocity = u[0, 0]
        steering = u[1, 0]
        brake_force = 0
        transmission = 0
        differential_front = 0
        differential_rear = 0
        self.actuation.send_control(
            steering, velocity, brake_force, transmission, differential_front, differential_rear)

        # Handle the publishing of RVIZ topics
        if self.PUB_TO_RVIZ:
            # Get MPC prediction as planned path
            x_pred = pred[0, :]
            y_pred = pred[1, :]

            new_pred = self.lists_to_pose_stampeds(
                list(x_pred), list(y_pred))
            path = Path()
            path.header.stamp = rospy.Time.now()
            path.header.frame_id = "map"
            path.poses = new_pred
            self.pred_path_pub.publish(path)
            self.rviz_handler.log_ctrl(
                steering, velocity, transmission, rospy.get_time())

        if self.PUB_TO_RVIZ:
            self.rviz_handler.log_state(self.state)
            self.rviz_handler.visualize_data()
            self.rviz_handler.update_target((ref[0, 0], ref[1, 0]))

    def window(self, s, v):
        look_ahead = (
            self.LOOK_AHEAD_BASE +
            self.LOOK_AHEAD_FACT * self.WINDOW_LEN * self.TIME_STEP * v
        )
        i = np.argmin(np.abs(
            self.track.points_array[2, :] - s - look_ahead
        ))
        x_ref = DM.zeros(self.NUM_STATES, self.WINDOW_LEN + 1)
        x_ref[0, :] = self.track.points_array[0, i:i + self.WINDOW_LEN + 1]
        x_ref[1, :] = self.track.points_array[1, i:i + self.WINDOW_LEN + 1]
        for j in range(self.WINDOW_LEN + 1):
            if i + j >= len(self.track.points):
                curv = self.track.points_array[3, i]
            else:
                curv = self.track.points_array[3, i + j]
            if curv > 0 or curv < 0:
                x_ref[2, j] = self.V_REF_CURV
            else:
                x_ref[2, j] = self.V_REF_STRT

        if self.FLIPPED_TRACK:
            x_ref[-1, :] = self.track.points_array[4, i:i + self.WINDOW_LEN + 1] - np.pi
        else:
            x_ref[-1, :] = self.track.points_array[4, i:i + self.WINDOW_LEN + 1]

        return x_ref

    def get_trajectory(self, x):
        N = self.controller.N
        dt = self.model.dt
        v = x[2, 0]
        s, e = self.track.to_local(x[0, 0], x[1, 0])

        x_ref = self.window(s, v)
        return x_ref

    def lists_to_pose_stampeds(self, x_list, y_list, yaw_list=None, t_list=None):
        poses = []
        for i in range(len(x_list)):
            x = x_list[i]
            y = y_list[i]

            curr_pose = PoseStamped()
            curr_pose.header.frame_id = 'map'
            curr_pose.pose.position.x = x
            curr_pose.pose.position.y = y

            if not yaw_list is None:
                yaw = yaw_list[i]
                quat = self.euler_to_quaternion(0.0, 0.0, yaw)
                curr_pose.pose.orientation.x = quat[0]
                curr_pose.pose.orientation.y = quat[1]
                curr_pose.pose.orientation.z = quat[2]
                curr_pose.pose.orientation.w = quat[3]

            if not t_list is None:
                t = t_list[i]
                curr_pose.header.stamp = rospy.Time(secs=t)
            else:
                curr_pose.header.stamp = rospy.Time.now()

            poses.append(curr_pose)
        return poses

    def euler_to_quaternion(self, r):
        (yaw, pitch, roll) = (r[0], r[1], r[2])
        qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - \
            np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
        qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + \
            np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
        qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - \
            np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
        qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + \
            np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
        return [qx, qy, qz, qw]

    def quaternion_to_euler(self, q):
        (x, y, z, w) = (q[0], q[1], q[2], q[3])
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return [yaw, pitch, roll]


if __name__ == '__main__':

    ## Start node ##

    track_sim()
