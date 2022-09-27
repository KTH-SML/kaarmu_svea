#! /usr/bin/env python

import numpy as np

from rosonic import Node, Parameter
from svea.models.bicycle import SimpleBicycleModel
from svea.states import VehicleState
from svea.simulators.sim_SVEA import SimSVEA
from svea.localizers import LocalizationInterface
from svea.controllers.pure_pursuit import PurePursuitController
from svea.svea_managers.path_following_sveas import SVEAPurePursuit
from svea.data import TrajDataHandler, RVIZPathHandler

def assert_points(pts):
    """
    Helper function for validation of points list.

    Functions like this one can be nice for support to the node.

    Args
    ----
        pts: whatever was given by the `points` parameter
    """

    assert isinstance(pts, (list, tuple)), 'points is of wrong type, expected list'
    for xy in pts:
        assert isinstance(xy, (list, tuple)), 'points contain an element of wrong type, expected list of two values (x, y)'
        assert len(xy), 'points contain an element of wrong type, expected list of two values (x, y)'
        x, y = xy
        assert isinstance(x, (int, float)), 'points contain a coordinate pair wherein one value is not a number'
        assert isinstance(y, (int, float)), 'points contain a coordinate pair wherein one value is not a number'


def set_state(state, n=10):

    from rospy import Publisher, Rate
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from tf.transformations import quaternion_from_euler

    p = PoseWithCovarianceStamped()
    p.header.frame_id = 'map'
    p.pose.pose.position.x = state.x
    p.pose.pose.position.y = state.y
    
    q = quaternion_from_euler(0, 0, state.yaw)
    p.pose.pose.orientation.z = q[2]
    p.pose.pose.orientation.w = q[3]

    pub = Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
    rate = Rate(10)

    for _ in range(n):
        pub.publish(p)
        rate.sleep()


class pure_pursuit(Node):

    POINTS = Parameter('~points')
    IS_SIM = Parameter('~is_sim', False)
    USE_RVIZ = Parameter('~use_rviz', False)

    # A rosonic Parameter work like "futures", you can specify what should
    # happen to them once they are loaded
    INIT_STATE = Parameter('~state', [0, 0]).then(lambda xy: VehicleState(*xy))

    # A class field doesn't have to be a rosonic Parameter
    DELTA_TIME = 0.01
    TRAJ_LEN = 10
    GOAL_THRESH = 0.2
    TARGET_VELOCITY = 1.0

    def __init__(self):

        assert_points(self.POINTS)

        # Set initial state
        self.state = self.INIT_STATE

        set_state(self.state)

        # create initial goal state
        self.curr = 0
        self.goal = self.POINTS[self.curr]
        xs, ys = self.compute_traj()

        if self.IS_SIM:
            self.sim_model = SimpleBicycleModel(self.state)
            self.simulator = SimSVEA(self.sim_model,
                                     dt=self.DELTA_TIME,
                                     run_lidar=True,
                                     start_paused=True).start()

        # start the SVEA manager
        self.svea = SVEAPurePursuit(LocalizationInterface,
                                    PurePursuitController,
                                    xs, ys,
                                    data_handler=RVIZPathHandler if self.USE_RVIZ else TrajDataHandler)

        self.svea.controller.target_velocity = self.TARGET_VELOCITY
        self.svea.start(wait=True)

        if self.IS_SIM:
            self.simulator.toggle_pause_simulation()

    def keep_alive(self):
        return not (self.svea.is_finished or self.is_shutdown())

    def spin(self):
        state = self.svea.wait_for_state()

        steering, velocity = self.svea.compute_control(state)
        self.svea.send_control(steering, velocity)

        if np.hypot(state.x - self.goal[0], state.y - self.goal[1]) < self.GOAL_THRESH:
            self.update_goal()
            xs, ys = self.compute_traj()
            self.svea.update_traj(xs, ys)

        if self.USE_RVIZ:
            self.svea.visualize_data()

    def update_goal(self):
        """
        Update the goal state to next in POINTS list

        Methods like this one can be nice as well.
        """

        self.curr += 1
        self.curr %= len(self.POINTS)
        self.goal = self.POINTS[self.curr]

    def compute_traj(self):
        xs = np.linspace(self.state.x, self.goal[0], self.TRAJ_LEN)
        ys = np.linspace(self.state.y, self.goal[1], self.TRAJ_LEN)
        return xs, ys


if __name__ == '__main__':

    ## Start node ##

    pure_pursuit.run()

