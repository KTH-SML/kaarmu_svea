#! /usr/bin/env python3

import numpy as np

from rosonic import Node, Parameter, OnShutdown

import rospy
from geometry_msgs.msg import Point
from std_srvs.srv import Trigger

from city_lmpc.arcs import get_track, adjust_for_lane, TRACK_CHOOSER
from city_lmpc.vehicle import VehicleInterface
from city_lmpc.ref_generator import RefGenerator


class rsu_demo(Node):

    S0 = Parameter('~s0')
    TRACK = Parameter('~track')
    VEHICLE_LIST = Parameter('~vehicle_list')
    SCENARIO = Parameter('~demo_scenario', 1)

    TIME_STEP = 0.1

    def __init__(self):

        self.master_track = get_track(
            TRACK_CHOOSER[self.TRACK],
            'center',
            self.S0,
            True,
        )

        # "lmpc" reference generator
        self.ref_gen = RefGenerator(
            get_track(
                self.master_track.arcs,
                'center',
                (self.master_track.x_s0, self.master_track.y_s0, self.master_track.phi_s0),
                not self.master_track.flip,
            ),
            self.TIME_STEP,
        )

        # Contains the VehicleInterface programs
        self.vehicle_subprogs = {}
        for name, downstream in self.VEHICLE_LIST:
            self.vehicle_subprogs[name] = VehicleInterface(
                name,
                downstream,
                self.master_track,
            ).start()

        self.srv_geofence_start = rospy.Service(
            '/geofence_start/trigger',
            Trigger,
            self.geofence_start_cb,
        )

        self.srv_geofence_crash = rospy.Service(
            '/geofence_crash/trigger',
            Trigger,
            self.geofence_crash_cb,
        )

    @OnShutdown()
    def on_shutdown(self):
        for name, _ in self.VEHICLE_LIST:
            veh = self.vehicle_subprogs.get(name)
            if veh is not None:
                veh.stop_and_join()

    def geofence_start_cb(self, _):

        self.log('GEOFENCE TRIGGER: start')

        def track_spin(veh):

            # Get coodinates
            x = veh.state.x
            y = veh.state.y
            v = veh.state.v

            # Compute reference trajectory and track parameters over the reference
            s, e = veh.track.to_local(x, y)
            look_ahead = (
                veh.LOOK_AHEAD_BASE +
                veh.LOOK_AHEAD_FACT * veh.WINDOW_LEN * veh.TIME_STEP * v
            )
            i = np.argmin(np.abs(
                veh.track.points_array[2, :] - s - look_ahead
            ))

            msg = Point()
            msg.x, msg.y, _ = adjust_for_lane(
                veh.lane,
                veh.track.points_array[0, i],
                veh.track.points_array[1, i],
                veh.track.points_array[4, i],
            )
            msg.z = veh.V_REF if s < veh.track.length - .5 else 0
            veh.pub_target.publish(msg)
            veh.rviz.update_target((msg.x, msg.y))

        for veh in self.vehicle_subprogs.values():
            if veh.spin.__name__ != 'track_spin':
                veh.spin = track_spin

        return True, ''

    def geofence_crash_cb(self, _):

        self.log('GEOFENCE TRIGGER: crash')

        # In case some vehicles are not in use
        obs = self.vehicle_subprogs.get('obs')
        ego = self.vehicle_subprogs.get('ego')
        onc = self.vehicle_subprogs.get('onc')

        ## OBS behaviour

        if not obs:
            return False, 'OBS not available'

        # if obs.spin.__name__ != 'stop_spin':

        def stop_spin(veh):

            msg = Point()
            msg.x = veh.state.x
            msg.y = veh.state.y
            msg.z = 0
            veh.pub_target.publish(msg)
            veh.rviz.update_target((msg.x, msg.y))

        obs.spin = stop_spin

        ## EGO behaviour

        if not (ego and onc):
            return False, 'EGO and/or ONC not available'

        ego_local = self.ref_gen.state_to_local(ego.state)
        # ego_local[0, 0] += 0.2
        ego_state = self.ref_gen.local_to_state(ego_local)
        ref = self.ref_gen.get_ref(ego_state, onc.state, obs_state=obs.state)

        traj_x = [ref[0, i] for i in range(ref.shape[1])]
        traj_y = [ref[1, i] for i in range(ref.shape[1])]
        ego.rviz.update_traj(traj_x, traj_y)
        ego.rviz.visualize_data()

        def lmpc_spin(veh):

            s, e = veh.track.to_local(veh.state.x, veh.state.y)

            # i = np.argmin(np.abs(np.linalg.norm(
            #     ref[:2, :] - np.array([[veh.state.x], [veh.state.y]]),
            #     axis=0,
            # )))
            i = 7

            msg = Point()
            msg.x = ref[0, i]
            msg.y = ref[1, i]
            msg.z = ref[2, i] if s < veh.track.length - .5 else 0
            veh.pub_target.publish(msg)
            veh.rviz.update_target((msg.x, msg.y))

        ego.spin = lmpc_spin

        return True, ''

    #      #      #      #      #      #      #      #      #      #      #      #
    # DEMO # DEMO # DEMO # DEMO # DEMO # DEMO # DEMO # DEMO # DEMO # DEMO # DEMO #
    #      #      #      #      #      #      #      #      #      #      #      #


    def demo_crash(self):

        if self.SCENARIO == 1:

            obs, ego, onc = self.vehicles

            rospy.loginfo(obs.state)
            obs_local = self.gen.state_to_local(obs.state)
            obs_local[0, 0] -= 0.6
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

    rsu_demo()
