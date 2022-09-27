#! /usr/bin/env python3
import numpy as np
import math
from numpy import pi
from casadi import DM, vertcat

import rospy
from rosonic import Node, Parameter
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path

from city_lmpc.models.frenet import FrenetModel
from city_lmpc.models.track import ArcByAngle, ArcByLength, Track
from city_lmpc.controllers.frenet import FrenetMPC

from svea_msgs.msg import VehicleState as VehicleStateMsg
from svea.actuation import ActuationInterface
from svea.states import VehicleState
from svea.data import RVIZPathHandler


class TrackFrenetBase(Node):
    name = "track_frenet_base"
    IS_SIM = Parameter("~is_sim", False)
    PUB_TO_RVIZ = Parameter("~pub_to_rviz", False)

    def __init__(self):
        self.start_flag = False
        self.state = VehicleState()
        self.actuation = ActuationInterface().start()
        rospy.Subscriber("state", VehicleStateMsg, self.state_cb)
        rospy.Subscriber("clicked_point", PointStamped, self.start_flag_cb)
        self.pred_path_pub = rospy.Publisher(
            "pred_path", Path, queue_size=1, latch=True)

        self.l_lane = 0.5
        self.e_ref = -self.l_lane / 2
        self.v_ref = 0.5
        # Track Arcs
        arcs = [ArcByLength(0, 1.35),
                ArcByLength(0, .50),
                ArcByAngle(-1 / np.sqrt(2) / .65, 90),
                ArcByLength(0, 4.0),
                ArcByAngle(1 / np.sqrt(2) / .65, 90),
                ArcByLength(0, .50),
                ArcByLength(0, 1.35),
                ]

        # Track Parameters
        self.look_ahead_base = 0.1
        self.look_ahead_factor = 0.1
        self.x_s0 = -0.848089039326
        self.y_s0 = -4.9686331749
        self.phi_s0 = 0.97 - pi
        self.track = Track(arcs, x_s0=self.x_s0,
                           y_s0=self.y_s0, phi_s0=self.phi_s0)

        # Model Parameters
        self.dt = 0.1
        self.model = FrenetModel(
            x0=[0.1, self.e_ref, 0, self.phi_s0], dt=self.dt)
        self.x = self.model.x0

        # MPC Parameters
        xlb = [0, -self.l_lane, -0.5, -np.inf]
        xub = [self.track.length, self.l_lane, 1, np.inf]
        ulb = [-1, -pi / 6]
        uub = [1.5, pi / 6]
        Q = [1, 80, 5, 40]
        R = [2, 1]
        self.controller = FrenetMPC(self.model, N=5, Q=Q, R=R,
                                    xlb=xlb, xub=xub, ulb=ulb, uub=uub)

        if self.PUB_TO_RVIZ:
            self.rviz_handler = RVIZPathHandler()
            traj_x, traj_y = self.track.cartesian
            self.rviz_handler.update_traj(traj_x, traj_y)

    def keep_alive(self):
        return not self.is_shutdown() and (self.x[0, 0] + 0.1 < self.track.length)

    def spin(self):
        # Get local frenet coodinates
        x_s, e = self.track.to_local(self.state.x, self.state.y)
        # Compute reference trajectory and track parameters over the reference
        xs_ref, curvature, s_0_arc, phi_0_arc = self.get_trajectory(self.x)
        # Build local frenet state
        self.x = vertcat(x_s, e, self.state.v, self.state.yaw)

        if self.start_flag:
            # Compute control using path tracking MPC
            u, xs_pred = self.controller.get_ctrl(
                self.x, xs_ref, curvature, s_0_arc, phi_0_arc)
            # Log velocity information
            rospy.loginfo_throttle(0.5, "v_pred {} v {}, v_ref {}, v_err {}".format(xs_pred[2, 0], self.state.v,
                                   xs_ref[2, 0], xs_ref[2, 0] - self.state.v))

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
                x_pred = np.zeros(xs_pred.shape[1])
                y_pred = np.zeros(xs_pred.shape[1])
                for j in range(xs_pred.shape[1]):
                    x_pred[j], y_pred[j] = self.track.to_global(
                        xs_pred[0, j], xs_pred[1, j])
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
            self.rviz_handler.update_target(
                self.track.to_global(xs_ref[0, 0], xs_ref[1, 0]))

    def state_cb(self, msg):
        self.state.state_msg = msg

    def start_flag_cb(self, msg):
        self.start_flag = True

    def get_trajectory(self, x):
        N = self.controller.N
        dt = self.dt
        v = x[2, 0]
        look_ahead_base = self.look_ahead_base
        look_ahead_factor = self.look_ahead_factor
        look_ahead_dist = look_ahead_base + look_ahead_factor * v * N * dt

        i = np.argmin(
            np.abs(self.track.points_array[2, :] - x[0, 0] - look_ahead_dist))  # i_with_look_ahead

        x_ref = DM.zeros(self.controller.n_states, N + 1)
        x_ref[0, :] = self.track.points_array[2, i:i + N + 1]
        x_ref[1, :] = self.e_ref
        x_ref[2, :] = self.v_ref
        x_ref[-1, :] = self.track.points_array[4, i:i + N + 1]
        i = np.argmin(
            np.abs(self.track.points_array[2, :] - x[0, 0]))
        curvature = self.track.points_array[3, i:i + N]
        s_0_arc = self.track.points_array[5, i:i + N]
        phi_0_arc = self.track.points_array[6, i:i + N]

        return x_ref, curvature, s_0_arc, phi_0_arc

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

    TrackFrenetBase.run()
