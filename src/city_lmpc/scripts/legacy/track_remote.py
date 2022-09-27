#! /usr/bin/env python3
import numpy as np
import math
from numpy import pi
from casadi import DM, vertcat

import rospy
from rosonic import Node, Parameter
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String

from city_lmpc.models.frenet import FrenetModel
from city_lmpc.models.bicycle import BicycleModel
from city_lmpc.models.track import ArcByAngle, ArcByLength, Track
from city_lmpc.controllers.frenet import FrenetMPC
from city_lmpc.controllers.bicycle import BicycleMPC

from svea_msgs.msg import VehicleState as VehicleStateMsg
from svea.actuation import ActuationInterface
from svea.states import VehicleState
from svea.data import RVIZPathHandler


class TrackBicycleBase(Node):
    name = "track_bicycle_remote"
    IS_SIM = Parameter("~is_sim", False)
    PUB_TO_RVIZ = Parameter("~pub_to_rviz", False)
    LANE_TO_TRACK = "right"  # "left" | "center" | "right"
    FLIPPED_REF = False

    def __init__(self):
        self.start_flag = False
        self.state = VehicleState()
        self.actuation = ActuationInterface().start()
        rospy.Subscriber("clicked_point", PointStamped, self.start_flag_cb)
        self.pred_path_pub = rospy.Publisher("pred_path", Path, queue_size=1, latch=True)

        self.v_ref = 0.3
        self.v_ref_curv = 0.2
        self.phi_s0 = 0.97 - pi
        self.l_lane = 0.25

        # Track Center Lane by default
        # Coordinates for starting next to the bend closes to project course rooms
        # self.x_s0 = -0.848089039326
        # self.y_s0 = -4.9686331749
        length_of_straight = 4

        # Coordinates for starting from the middle of floor2
        self.x_s0 = 6.6403172533607675
        self.y_s0 = 6.21655082703
        # length_of_straight = 3.5

        if self.LANE_TO_TRACK == "left":  # Track Left Lane
            diff = self.l_lane
        elif self.LANE_TO_TRACK == "right":  # Track Right Lane
            diff = -self.l_lane
        else:  # Track road center
            diff = 0
        self.x_s0 -= diff * np.sin(self.phi_s0)
        self.y_s0 += diff * np.cos(self.phi_s0)

        arcs = [ArcByLength(0, 1.35 + diff),
                ArcByLength(0, .50),
                ArcByAngle(-1 / np.sqrt(2) / .65, 90),
                ArcByLength(0, length_of_straight),
                ArcByAngle(1 / np.sqrt(2) / .65, 90),
                ArcByLength(0, .50),
                ArcByLength(0, 1.35 - diff),
                ]

        arcs_bendy = [ArcByLength(0, 1.35 + diff),
                      ArcByLength(0, .50),
                      # turn right
                      ArcByAngle(-1 / np.sqrt(2) / .65, 90),
                      # drive straight
                      ArcByLength(0, length_of_straight / 2 - 6 * self.l_lane),
                      #   begin left bend
                      ArcByAngle(1 / self.l_lane, 90),
                      #   end left bend
                      ArcByAngle(-1 / self.l_lane, 90),
                      # drive straight on left lane
                      ArcByLength(0, 5 * self.l_lane),
                      # begin bend right
                      ArcByAngle(-1 / self.l_lane, 90),
                      # end bend right
                      ArcByAngle(1 / self.l_lane, 90),
                      # drive on right lane
                      ArcByLength(0, 3 * self.l_lane),
                      # turn left
                      ArcByAngle(1 / np.sqrt(2) / .65, 90),
                      ArcByLength(0, .50),
                      ArcByLength(0, 1.35 - diff)]

        self.track = Track(arcs, self.x_s0, self.y_s0, self.phi_s0, self.FLIPPED_REF)
        # Set self.ref to the incoming ref trajectory
        rospy.Subscriber('/rsu/track', String, lambda msg: setattr(self, 'ref', DM.deserialize(msg)))

        # Model Parameters
        self.dt = 0.1
        point = self.track.points[0]
        phi_s0 = (point.phi_s - pi) if self.FLIPPED_REF else point.phi_s
        x0 = [point.x_s, point.y_s, 0, phi_s0]
        self.model = BicycleModel(
            x0=x0, dt=self.dt)
        self.x = self.model.x0

        # MPC Parameters
        xlb = [-np.inf, -np.inf, -0.5, -np.inf]
        xub = [np.inf, np.inf, 1, np.inf]
        ulb = [-1, -pi / 6]
        uub = [1.5, pi / 6]
        Q = [1, 1, 100, 5]
        R = [1, 2]
        self.controller = BicycleMPC(self.model, N=5, Q=Q, R=R,
                                     xlb=xlb, xub=xub, ulb=ulb, uub=uub)

        self.ref = DM.zeros(self.controller.n_states, self.controller.N+1)

        if self.PUB_TO_RVIZ:
            self.rviz_handler = RVIZPathHandler()
            traj_x, traj_y = self.track.cartesian
            self.rviz_handler.update_traj(traj_x, traj_y)

    def keep_alive(self):
        return not self.is_shutdown() and (np.linalg.norm(self.x[:2, 0] - self.track.points_array[:2, -1])) > 0.5

    def spin(self):
        # Get local frenet coodinates
        self.x = vertcat(self.state.x, self.state.y,
                         self.state.v, self.state.yaw)

        if self.start_flag:
            # Compute control using path tracking MPC
            u, pred = self.controller.get_ctrl(self.x, self.ref)
            # Log velocity information
            # rospy.loginfo_throttle(0.5, "v_pred {} v {}, v_ref {}, v_err {}".format(pred[2, 0], self.state.v,
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
            self.rviz_handler.update_target((self.ref[0, 0], self.ref[1, 0]))

    def state_cb(self, msg):
        self.state.state_msg = msg

    def start_flag_cb(self, msg):
        self.start_flag = True

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

    TrackBicycleBase.run()
