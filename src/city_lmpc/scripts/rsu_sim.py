#! /usr/bin/env python3

import math

import numpy as np

import rospy
from rosonic import Node, Parameter
from geometry_msgs.msg import Point
from std_srvs.srv import Trigger, SetBool

from city_lmpc.models.track import Track
from city_lmpc.arcs import get_track, adjust_for_lane, TRACK_CHOOSER

from svea_msgs.msg import VehicleState as VehicleStateMsg
from svea.states import VehicleState
from svea.data import RVIZPathHandler

from city_lmpc.ref_generator import RefGenerator

class Vehicle:

    name: str           # name of the svea vehicle (ROS namespace)
    state: VehicleState # state of vehicle
    track: Track        # track to follow
    lane: str
    rviz: RVIZPathHandler
    pub_target: rospy.Publisher
    enable_vehicle: rospy.ServiceProxy

    def __init__(self, name: str, master_track: Track, downstream: bool):

        self.name = name 
        self.state = VehicleState(
            child_frame=f'{self.name}_base_link',
        )
        self.lane = 'right' if downstream else 'left'
        self.downstream = downstream
        self.track = get_track(
            master_track.arcs,
            'center',
            (master_track.x_s0, master_track.y_s0, master_track.phi_s0),
            not downstream,
        )
        self.rviz = RVIZPathHandler(name)

        rospy.loginfo('waiting for %s', self.name)

        rospy.wait_for_service(f'/{self.name}/enable_vehicle')
        self.enable_vehicle=rospy.ServiceProxy(
            f'/{self.name}/enable_vehicle',
            SetBool,
        )

        rospy.loginfo('connected to %s', self.name)
        
        rospy.Subscriber(
            f'/{self.name}/state',
            VehicleStateMsg, 
            lambda msg: setattr(self.state, 'state_msg', msg)
        )
        self.state.state_msg = rospy.wait_for_message(f'/{self.name}/state', VehicleStateMsg)

        self.pub_target = rospy.Publisher(
            f'/rsu/{self.name}_target',
            Point,
            queue_size=1,
        )

        rospy.loginfo('target publisher has been created for %s', self.name)

        traj_x, traj_y = self.track.cartesian
        self.rviz.update_traj(traj_x, traj_y)
        self.rviz.visualize_data()

        rospy.loginfo('vehicle %s has been created', self.name)


class rsu_sim(Node):

    S0 = Parameter('~s0')
    TRACK = Parameter('~track')
    VEHICLES = Parameter('~vehicles', [])

    V_REF_STRT = 0.3
    V_REF_CURV = 0.2

    LOOK_AHEAD_BASE = 0.4
    LOOK_AHEAD_FACT = 0

    NUM_STATES = 4
    WINDOW_LEN = 5
    TIME_STEP = 0.1

    def __init__(self):

        self.master_track = get_track(
            TRACK_CHOOSER[self.TRACK],
            'center',
            self.S0,
            True,
        )

        # This will wait for the vehicle to provde "enable_vehicle" service
        self.vehicles = [Vehicle(name, self.master_track, downstream) 
                         for (name, downstream) in self.VEHICLES]

        self.srv_geofence = rospy.Service(
            'geofence',
            Trigger,
            self.geofence_cb,
        )

        self.demo_init()

    def geofence_cb(self, req):
        obs, ego, onc = self.vehicles
        obs.enable_vehicle(False)
        self.demo_crash()
        return True, ''

    def keep_alive(self):
        rsu_cond = False
        for vehicle in self.vehicles:
            x, y = vehicle.state.x, vehicle.state.y

            s, e = vehicle.track.to_local(x, y)

            veh_cond = s < vehicle.track.length - .5

            if not veh_cond:
                vehicle.enable_vehicle(False) 

            rsu_cond = rsu_cond or veh_cond 

        return rsu_cond and not self.is_shutdown() 


    def spin(self):
        for vehicle in self.vehicles:
            msg = self.vehicle_spin(vehicle)
            vehicle.pub_target.publish(msg)


    #      #      #      #      #      #      #      #      #      #      #      # 
    # DEMO # DEMO # DEMO # DEMO # DEMO # DEMO # DEMO # DEMO # DEMO # DEMO # DEMO # 
    #      #      #      #      #      #      #      #      #      #      #      # 

    SCENARIO = Parameter('~demo_scenario', 1)

    def demo_init(self):

        def spinner(vehicle):

            # Get coodinates
            x = vehicle.state.x
            y = vehicle.state.y
            v = vehicle.state.v

            # Compute reference trajectory and track parameters over the reference
            s, e = vehicle.track.to_local(x, y)
            look_ahead = (
                self.LOOK_AHEAD_BASE +
                self.LOOK_AHEAD_FACT * self.WINDOW_LEN * self.TIME_STEP * v
            )
            i = np.argmin(np.abs(
                vehicle.track.points_array[2, :] - s - look_ahead
            ))

            # publish
            msg = Point()
            msg.x, msg.y, _ = adjust_for_lane(
                vehicle.lane,
                vehicle.track.points_array[0, i],
                vehicle.track.points_array[1, i],
                vehicle.track.points_array[4, i],
            )
            msg.z = 0.5 # [m/s]
            return msg

        self.vehicle_spin = spinner

        obs, ego, onc = self.vehicles

        self.gen = RefGenerator(
            get_track(
                self.master_track.arcs,
                'center',
                (self.master_track.x_s0, self.master_track.y_s0, self.master_track.phi_s0),
                not self.master_track.flip,
            ),
            self.TIME_STEP,
        )
        
        obs.enable_vehicle(True)
        ego.enable_vehicle(True)

        rospy.Timer(rospy.Duration(8),
                    lambda _: onc.enable_vehicle(True))

    def demo_crash(self):

        obs, ego, onc = self.vehicles

        rospy.loginfo(obs.state)
        obs_local = self.gen.state_to_local(obs.state)
        obs_local[0, 0] -= 0.5
        obs_state = self.gen.local_to_state(obs_local)
        ref = self.gen.get_ref(ego.state, onc.state, obs_state=obs_state)

        traj_x = [ref[0, i] for i in range(ref.shape[1])]
        traj_y = [ref[1, i] for i in range(ref.shape[1])]
        ego.rviz.update_traj(traj_x, traj_y)
        ego.rviz.visualize_data()

        old_spin = self.vehicle_spin

        def spinner(vehicle):

            if vehicle.name == 'ego':

                obs, ego, onc = self.vehicles

                vehicle.enable_vehicle(True)

                if ref.shape[1] < 5:
                    k = -1
                    self.vehicle_spin = old_spin
                else:
                    k = 5

                msg = Point()
                msg.x = ref[0, k]
                msg.y = ref[1, k]
                msg.z = ref[2, k]

            else:
                msg = old_spin(vehicle)

            return msg

        self.vehicle_spin = spinner


if __name__ == '__main__':

    ## Start node ##

    rsu_sim()
