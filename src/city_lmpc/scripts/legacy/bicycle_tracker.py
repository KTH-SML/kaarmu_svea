#! /usr/bin/env python3
# system path management (to import from adjacent directories)

from ast import BitXor
import numpy as np
from numpy import pi
from casadi import DM, vertcat

import rospy
from rosonic import Node, Parameter
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path

from city_lmpc.models.frenet import FrenetModel
from city_lmpc.models.bicycle import BicycleModel
from city_lmpc.models.track import ArcByAngle, ArcByLength, Track
from city_lmpc.controllers.frenet import FrenetMPC
from city_lmpc.controllers.bicycle import BicycleMPC


from svea_msgs.msg import VehicleState as VehicleStateMsg
from svea.actuation import ActuationInterface
from svea.models.bicycle import SimpleBicycleModel
from svea.states import VehicleState
from svea.simulators.sim_SVEA import SimSVEA
from svea.localizers import LocalizationInterface
from svea.controllers.pure_pursuit import PurePursuitController
from svea.svea_managers.path_following_sveas import SVEAPurePursuit
from svea.data import TrajDataHandler, RVIZPathHandler
from svea.simulators.viz_utils import lists_to_pose_stampeds


class BicycleTracker(Node):
    name = "bicycle_tracker_sim"
    IS_SIM = Parameter("~is_sim", False)
    PUB_TO_RVIZ = Parameter("~pub_to_rviz", False)
    X_S0 = Parameter("~x_s0", 0)
    Y_S0 = Parameter("~y_s0", 0)
    PHI_S0 = Parameter("~phi_s0", 0)

    rate = 10

    def __init__(self):
        self.start_time = rospy.get_time()
        self.start_flag = False
        self.state = VehicleState()
        self.actuation = ActuationInterface().start()
        rospy.Subscriber("state", VehicleStateMsg, self.state_cb)
        rospy.Subscriber("clicked_point", PointStamped, self.start_flag_cb)
        self.pred_path_pub = rospy.Publisher(
            "pred_path", Path, queue_size=1, latch=True)

        self.l_lane = 0.5
        self.e_ref = -self.l_lane / 2
        self.v_ref = 0.75  # .75  # 0.34803149606299205
        # Track Arcs
        arcs = [ArcByLength(0, 1.35),
                ArcByLength(0, .50),
                ArcByAngle(-1 / np.sqrt(2) / .65, 90),
                ArcByLength(0, 4.0),
                ArcByAngle(1 / np.sqrt(2) / .65, 90),
                ArcByLength(0, .50),
                ArcByLength(0, 1.35),
                ]
        # arcs = [ArcByLength(0, 10)]

        # Track Parameters
        self.x_s0 = -0.848089039326
        self.y_s0 = -4.9686331749
        self.phi_s0 = 0.97 - pi
        self.track = Track(arcs, x_s0=self.x_s0,
                           y_s0=self.y_s0, phi_s0=self.phi_s0)

        # Model Parameters
        self.dt = 0.1
        self.model = BicycleModel(
            x0=[self.x_s0, self.y_s0, 0, self.phi_s0], dt=self.dt)
        self.x = self.model.x0

        # MPC Parameters
        xlb = [-np.inf, -np.inf, -0.5, -np.inf]
        xub = [np.inf, np.inf, 1, np.inf]
        # xlb = [-np.inf, -np.inf, -np.inf, -np.inf]
        # xub = [np.inf, np.inf, np.inf, np.inf]
        ulb = [-1, -pi / 6]
        uub = [1.5, pi / 6]

        Q = [10, 10, 5, 8]
        R = [2, 1]
        self.controller = BicycleMPC(self.model, N=5, Q=Q, R=R,
                                     xlb=xlb, xub=xub, ulb=ulb, uub=uub)

        if self.PUB_TO_RVIZ:
            self.rviz_handler = RVIZPathHandler()
            traj_x, traj_y = self.track.cartesian
            self.rviz_handler.update_traj(traj_x, traj_y)

        if self.IS_SIM:
            self.state.x = self.x[0, 0]
            self.state.y = self.x[1, 0]
            self.state.v = self.x[2, 0]
            self.state.yaw = self.x[3, 0]
            self.sim_model = SimpleBicycleModel(self.state)
            self.simulator = SimSVEA(self.sim_model,
                                     dt=self.dt,
                                     run_lidar=True,
                                     start_paused=True).start()

        if self.IS_SIM:
            self.simulator.toggle_pause_simulation()

    def keep_alive(self):
        return not self.is_shutdown() and (np.linalg.norm(self.x[:2, 0] - self.track.points_array[:2, -1]) > 0.1)

    def spin(self):
        self.x = vertcat(self.state.x, self.state.y,
                         self.state.v, self.state.yaw)

        ref = self.get_trajectory(self.x)

        if self.start_flag:
            u, pred = self.controller.get_ctrl(
                self.x, ref)
            rospy.loginfo_throttle(0.5, "v_pred {} v {} v_err {}".format(
                pred[2, 0], self.state.v, self.state.v - ref[2, 0]))

            # u = vertcat(u, curvature[0], s_0_arc[0], phi_0_arc[0])
            # Simulate one step
            # self.model.sim(time_steps=1, u=u,
            #    state_noise=False, input_noise=True)

            velocity = u[0, 0]
            steering = u[1, 0]
            brake_force = 0
            transmission = 0
            differential_front = 0
            differential_rear = 0
            self.actuation.send_control(
                steering, velocity, brake_force, transmission, differential_front, differential_rear)

            if self.PUB_TO_RVIZ:
                # Get MPC prediction as planned path
                x_pred = pred[0, :]
                y_pred = pred[1, :]

                new_pred = lists_to_pose_stampeds(list(x_pred), list(y_pred))
                path = Path()
                path.header.stamp = rospy.Time.now()
                path.header.frame_id = "map"
                path.poses = new_pred
                self.pred_path_pub.publish(path)

                # self.rviz_handler.log_ctrl(
                #     steering, velocity, transmission, rospy.get_time())
            self.start_time = rospy.get_time()

        if self.PUB_TO_RVIZ:
            self.rviz_handler.log_state(self.state)
            self.rviz_handler.visualize_data()
            self.rviz_handler.update_target((ref[0, 0], ref[1, 0]))

    def state_cb(self, msg):
        self.state.state_msg = msg
        # x_traj, _ = self.model.get_trajectory()
        # xs = x_traj[:, -1]
        # x, y = self.track.to_global(xs[0], xs[1])
        # self.state.x = x
        # self.state.y = y
        # self.state.yaw = xs[-1]
        # self.state.v = xs[2]

    def start_flag_cb(self, msg):
        self.start_time = rospy.get_time()
        self.start_flag = True

    def get_trajectory(self, x):
        N = self.controller.N
        dt = self.model.dt
        v = x[2, 0]
        s, e = self.track.to_local(x[0, 0], x[1, 0])
        look_ahead_base = 0.0
        look_ahead_factor = .5
        look_ahead_dist = look_ahead_base + look_ahead_factor * v * N * dt

        i = np.argmin(
            np.abs(self.track.points_array[2, :] - s - look_ahead_dist))  # i_with_look_ahead

        x_ref = DM.zeros(self.controller.n_states, N + 1)
        x_ref[0, :] = self.track.points_array[0, i:i + N + 1]
        x_ref[1, :] = self.track.points_array[1, i:i + N + 1]
        x_ref[2, :] = self.v_ref
        x_ref[-1, :] = self.track.points_array[4, i:i + N + 1]

        return x_ref


if __name__ == '__main__':

    ## Start node ##

    BicycleTracker.run()
