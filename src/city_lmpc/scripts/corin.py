#! /usr/bin/env python3

import numpy as np

from rosonic import Node, Parameter, OnShutdown

import rospy
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import Point
from std_srvs.srv import Trigger

from city_lmpc.arcs import get_track, adjust_for_lane, TRACK_CHOOSER
from city_lmpc.vehicle import VehicleInterface
from city_lmpc.ref_generator import RefGenerator

from rsu_msgs.msg import StampedObjectPoseArray

def isinside(pt, box):
    xl, yl, xu, yu = box
    x, y = pt
    return (xl < x < xu) and (yl < y < yu)

class rsu_demo(Node):

    S0 = Parameter('~s0')
    TRACK = Parameter('~track')
    VEHICLE_LIST = Parameter('~vehicle_list')
    SCENARIO = Parameter('~demo_scenario', 1)

    TIME_STEP = 0.1

    SAFE_BOX = [-1.26, -7.15, +0.74, -5.15]

    unsafe_since = None 

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

        self.tf_buf = tf2_ros.Buffer() 
        tf2_ros.TransformListener(self.tf_buf)

        self.sub_objectposes = rospy.Subscriber(
            '/rsu/objectposes',
            StampedObjectPoseArray,
            self.objectposes_cb,
            # queue_size=4,
            # buff_size=4*100,
        )

        self.srv_geofence_crash = rospy.Service(
            '/geofence_crash/trigger',
            Trigger,
            self.geofence_crash_cb,
        )

        self.srv_geofence_start = rospy.Service(
            '/geofence_start/trigger',
            Trigger,
            self.geofence_start_cb,
        )

    @OnShutdown()
    def on_shutdown(self):
        for name, _ in self.VEHICLE_LIST:
            veh = self.vehicle_subprogs.get(name)
            if veh is not None:
                veh.stop_and_join()

    def objectposes_cb(self, objectposes):

        origin_frame = objectposes.header.frame_id 

        if not self.tf_buf.can_transform(origin_frame, 'map', rospy.Time()):
            print('CANNOT TRANSFORM')
            return

        for objpose in filter(self.person_filter, objectposes.objects):

            transform = self.tf_buf.lookup_transform(origin_frame, 'map', rospy.Time())
            pose = do_transform_pose(objpose.pose, transform)

            pt = pose.pose.position.x, pose.pose.position.y
            ## print("PERSON!!!", pt)

            if isinside(pt, self.SAFE_BOX):
                self.geofence_safe_cb(None)
                return

    @staticmethod 
    def person_filter(objpose):
        return objpose.object.label == 'person'

    def geofence_start_cb(self, _):

        if not self.is_safe:
            return True, ''

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

        obs = self.vehicle_subprogs.get('obs')
        ego = self.vehicle_subprogs.get('ego')
        onc = self.vehicle_subprogs.get('onc')
        self.log('GEOFENCE TRIGGER: start  (%s, %s, %s)', obs.spin.__name__, ego.spin.__name__, onc.spin.__name__)

        return True, ''

    @property
    def is_safe(self):
        t = rospy.Time.now()
        if self.unsafe_since is None:
            # Unsafe hasn't happened
            return True
        elif (t - self.unsafe_since).to_sec() > 3:
            # Must have been safe for at least 3 sec
            ##  print("SAFE AGAIN")
            self.unsafe_since = None 
            return True
        else:
            """COMMENT
            print("time since:", (t - self.unsafe_since).to_sec(), 
                  "current time:", t, 
                  "unsafe time:", self.unsafe_since)
            """
            return False

    def geofence_safe_cb(self, _):

        self.unsafe_since = rospy.Time.now()

        all_spins = [veh.spin for veh in self.vehicle_subprogs.values()]

        def safe_spin(veh):

            msg = Point()
            msg.x = veh.state.x
            msg.y = veh.state.y
            msg.z = veh.state.v * 0.5   # decrease with 50 %
            veh.pub_target.publish(msg)
            veh.rviz.update_target((msg.x, msg.y))

            if self.is_safe:
                # return to old spin
                vehs = self.vehicle_subprogs.values()
                for veh, spin in zip(vehs, all_spins):
                    if veh.spin.__name__ != spin.__name__:
                        veh.spin = spin

        for veh in self.vehicle_subprogs.values():
            if veh.spin.__name__ != 'safe_spin':
                veh.spin = safe_spin

        self.log('GEOFENCE TRIGGER: safe')

    def geofence_crash_cb(self, _):

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

        # ego_local[0, 0] += ego.state.v * ego.TIME_STEP * 5
        ego_state = self.ref_gen.local_to_state(ego_local)
        ref = self.ref_gen.get_ref(ego_state, onc.state, obs_state=obs.state)

        traj_x = [ref[0, i] for i in range(ref.shape[1])]
        traj_y = [ref[1, i] for i in range(ref.shape[1])]
        ego.rviz.update_traj(traj_x, traj_y)
        ego.rviz.visualize_data()

        def lmpc_spin(veh):

            s, e = veh.track.to_local(veh.state.x, veh.state.y)

            if not s < veh.track.length - .5:
                veh.spin = stop_spin
                return

            # i = np.argmin(np.abs(np.linalg.norm(
            #     ref[:2, :] - np.array([[veh.state.x], [veh.state.y]]),
            #     axis=0,
            # )))
            i = 7

            msg = Point()
            msg.x = ref[0, i]
            msg.y = ref[1, i]
            msg.z = ref[2, i]
            veh.pub_target.publish(msg)
            veh.rviz.update_target((msg.x, msg.y))

        ego.spin = lmpc_spin
        self.log(f'GEOFENCE TRIGGER: crash  (%s, %s, %s)  {ref[:3, 7]}', obs.spin.__name__, ego.spin.__name__, onc.spin.__name__)
        return True, ''


if __name__ == '__main__':

    ## Start node ##

    rsu_demo()
