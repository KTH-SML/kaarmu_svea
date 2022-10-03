from math import ceil
from pathlib import Path
import pickle
import numpy as np
import casadi as ca

from city_lmpc.models.track import ArcByAngle, ArcByLength, Track
from svea_msgs.msg import VehicleState
from geometry_msgs.msg import PoseStamped

# SVEA Vehicle parameters
LENGTH = 0.586  # [m]
WIDTH = 0.2485  # [m]
BACKTOWHEEL = 0.16  # [m]

# Experiment parameters (not used in code, left as a reference)
S_OBS = 4.867936954901469
S_EGO_0 = 3.881936954901469


class RefGenerator():

    ROOT = Path(__file__)

    # TUNABLE PARAMETERS
    # used to calculate number of time-steps until onc passed ego (if ego stands-still), the closer to 0 the longer we will wait even after onc has passed
    VELOCITY_MARGIN = 0.9  # \in (0,1]
    N_SAFETY_LENGTHS = 2
    # [m] bumper-to-bumper distance between ego and obs vehicles
    DIST_OBS_TO_EGO = 0.4
    # don't update trajectories based on obs_state if new_obs_state-old_obs_state is within this value
    S_OBS_SENSITIVITY = 0.2

    # Computed bumper-to-bumper safety distance
    L_SAFE = 2 * (LENGTH - BACKTOWHEEL) + N_SAFETY_LENGTHS * LENGTH

    def __init__(self, track: Track, dt=0.1):
        self.dt = dt
        self.track = track
        self.traj_local = self.load_trajectory()
        self.n_states = self.traj_local.shape[0]
        self.n_steps = self.traj_local.shape[1]
        self.traj_global = self._traj_to_global()
        # Time-steps at which ego must have returned to main lane
        self.n_r = self.traj_local.shape[1] - 1 - \
            np.argmin(np.absolute(self.traj_local[1, ::-1]))
        self.s_obs_old = S_OBS

    def get_ref(self, ego_state: VehicleState, onc_state: VehicleState, obs_state: VehicleState = None, obs_pose: PoseStamped = None, frame="map"):
        self.frame = frame
        ego_local = self.state_to_local(ego_state)
        onc_local = self.state_to_local(onc_state)

        # position of ego along s
        s_ego_0 = ego_local[0, 0]
        # position of onc along s
        s_onc_0 = onc_local[0, 0]
        # velocity of onc
        v_onc = onc_local[2, 0]

        # Update trajectories if obs_state is given and it's value deviates from the old value by S_OBS_SENSITIVITY
        if obs_state is not None:
            obs_local = self.state_to_local(obs_state)
            # position of obs along s
            s_obs_0 = obs_local[0, 0]
            # check that s_obs moved beyond sensitivity limit and that we aren't already past the old s_obs_0
            if abs(self.s_obs_old - s_obs_0) > self.S_OBS_SENSITIVITY and s_obs_0 > s_ego_0:
                print("From RefGenerator: recalculating states because OBS vehicle has moved more than",
                      self.S_OBS_SENSITIVITY, "meters")
                obs_local = self.state_to_local(obs_state)
                self.traj_local[0, :] = self.traj_local[0, :] - \
                    self.traj_local[0, 0] + obs_local[0, 0] - \
                    LENGTH - self.DIST_OBS_TO_EGO
                self.traj_global = self._traj_to_global()
                self.s_obs_old = s_obs_0
                self.n_r = self.traj_local.shape[1] - 1 - \
                    np.argmin(np.absolute(self.traj_local[1, ::-1]))

        # position of ego in time-steps
        n_ego = np.argmin(np.absolute(self.traj_local[0, :] - s_ego_0))

        if n_ego > self.n_r:  # ego has already returned to main lane
            return self._check_for_obstacle(obs_pose, n_ego, self.traj_local, self.traj_global)

        # number of time-steps until ego returns to main lane
        n_delta = self.n_r - n_ego
        # time until ego returns to main lane
        t_ego_r = self.dt * n_delta
        # projected position of onc along s when ego returns to main lane
        s_onc_r = s_onc_0 - v_onc * t_ego_r
        # position of ego along s when ego returns to main lane (assuming perfect path tracking)
        s_ego_r = self.traj_local[0, self.n_r]

        # onc has drive past ego or bumper-to-bumper distance between ego and onc is large enough
        # when the time ego returns to main lane
        if (s_onc_0 < s_ego_0) or ((s_onc_r - s_ego_r) > self.L_SAFE):
            return self._check_for_obstacle(obs_pose, n_ego, self.traj_local, self.traj_global)
        else:  # onc is too close to ego, we have to wait until onc passed ego
            # time until onc passes ego
            # we set traj_local[0,0] as an initial reference in case s_ego_0 > self.traj_local[0,0]
            # if s_ego_0 happens to be before the first point in trajectory, take the minimum of the two values.
            s_ego_0 = min(s_ego_0, float(self.traj_local[0, 0]))
            if v_onc != 0:
                t_onc_p = (s_onc_0 - s_ego_0) / (self.VELOCITY_MARGIN *
                                                 v_onc)  # assuming v_onc is non-zero
                # time-steps until onc passes ego
                n_p = int(ceil(abs(t_onc_p) / self.dt))
                # cap n_p to be at most equal to the length of the local trajectory_filename
                # n_p = min(n_p, self.traj_local.shape[1])
            else:
                n_p = self.traj_local.shape[1]

            print(f'n_p={n_p}, s_ego_0={s_ego_0}, s_onc_0={s_onc_0}')

            # self.n_r += n_p
            # get ref extension buffer (stand still reference)
            ref_buffer_local = ca.DM.zeros(self.n_states, n_p)
            ref_buffer_global = ca.DM.zeros(self.n_states, n_p)

            # ref_buffer_local[0, :] = ego_local[0, 0]
            # ref_buffer_local[1, :] = ego_local[1, 0]
            ref_buffer_local[0, :] = self.traj_local[0, 0]
            ref_buffer_local[1, :] = self.traj_local[1, 0]

            # ref_buffer_global[0, :] = ego_state.x
            # ref_buffer_global[1, :] = ego_state.y
            ref_buffer_global[0, :] = self.traj_global[0, 0]
            ref_buffer_global[1, :] = self.traj_global[1, 0]
            ref_buffer_local[-1, :] = ref_buffer_global[-1, :] = ego_state.yaw

            # extend local and global trajectories
            traj_local = ca.horzcat(ref_buffer_local, self.traj_local)
            traj_global = ca.horzcat(ref_buffer_global, self.traj_global)
            n_steps = self.traj_local.shape[1]
            return self._check_for_obstacle(obs_pose, n_ego, traj_local, traj_global)

    def _check_for_obstacle(self, obs_pose, n_ego, traj_local, traj_global):
        if obs_pose is not None:
            obs_local = self.pose_to_local(obs_pose)
            # position of obstacle along s
            s_obs = obs_local[0, 0]
            # position of obstacle in time-steps
            n_obs = np.argmin(np.absolute(traj_local[0, :] - s_obs))

            if n_ego < n_obs:  # ego hasn't passed obstacle
                if self.frame == "map":
                    return traj_global[:, n_ego:n_obs - 1]
                else:
                    return traj_local[:, n_ego:n_obs - 1]

        if self.frame == "map":
            return traj_global[:, n_ego:]
        else:
            return traj_local[:, n_ego:]

    def load_trajectory(self, trajectory_filename="J_ref.npx"):
        try:
            with open(self.ROOT.parent / f'trajectories/{trajectory_filename}', "rb") as file:
                states, _ = pickle.load(file).values()
                return ca.DM(states)
        except IOError as error:
            raise(error)

    def _traj_to_global(self):
        traj_global = ca.DM.zeros(self.n_states, self.n_steps)
        traj_global[2, :] = self.traj_local[2, :]
        traj_global[3, :] = self.traj_local[3, :]

        for i in range(self.n_steps):
            x, y = self.track.to_global(
                self.traj_local[0, i], self.traj_local[1, i])
            traj_global[0, i] = x
            traj_global[1, i] = y

        return traj_global

    def state_to_local(self, state: VehicleState):
        s, e = self.track.to_local(x=state.x, y=state.y)
        return ca.vertcat(s, e, state.v, state.yaw)

    def pose_to_local(self, pose: PoseStamped):
        x = pose.pose.position.x
        y = pose.pose.position.y
        s, e = self.track.to_local(x, y)
        return ca.vertcat(s, e, 0, 0)

    def local_to_state(self, local_state):
        x, y = self.track.to_global(local_state[0, 0], local_state[1, 0])
        state = VehicleState()
        state.x = x
        state.y = y
        state.yaw = local_state[-1, 0]
        state.v = local_state[2, 0]
        return state

    def global_to_state(self, global_state):
        state = VehicleState()
        state.x = global_state[0, 0]
        state.y = global_state[1, 0]
        state.yaw = global_state[-1, 0]
        state.v = global_state[2, 0]
        return state


def test(ego_start, onc_start):
    arcs = [
        # ArcByLength(0, 1.35),
        ArcByLength(0, .50),
        ArcByAngle(-1 / np.sqrt(2) / .65, 90),
        ArcByLength(0, 4),
        ArcByAngle(1 / np.sqrt(2) / .65, 90),
        ArcByLength(0, .50),
        # ArcByLength(0, 1.35)
    ]

    track = Track(arcs, x_s0=0, y_s0=0, phi_s0=0)
    ref = RefGenerator(track)

    import matplotlib.pyplot as plt
    from copy import deepcopy
    xt, yt = track.cartesian

    onc_locals = ref.load_trajectory("onc.npx")
    onc_locals[0, :] = track.length - onc_locals[0, :]

    t_ego = ego_start
    ego_local = deepcopy(ref.traj_local[:, t_ego])
    ego_state = ref.local_to_state(ego_local)

    t_onc = onc_start  # 20
    onc_state = ref.local_to_state(onc_locals[:, t_onc])

    obs_pose = PoseStamped()
    obs_pose.pose.position.x = 6
    obs_pose.pose.position.y = -6

    obs_state = ref.local_to_state(ca.vertcat(S_OBS, -0.25, 0, 0))
    ref_traj = ref.get_ref(ego_state, onc_state,
                           obs_state, obs_pose=obs_pose)
    ref_traj_0 = deepcopy(ref_traj)

    for t in range(max(onc_locals.shape[1], ref_traj_0.shape[1])):
        # TODO FIX what happens when s_ego_0 is at the point of no return and cannot reverse when chagneing s_obs eg. if if statement uses new s_obs_0
        ref_traj = ref.get_ref(ego_state, onc_state,
                               obs_state=obs_state, obs_pose=obs_pose)
        k_onc = -1 if t + t_onc + 1 > onc_locals.shape[1] else t + t_onc
        onc_state = ref.local_to_state(onc_locals[:, k_onc])

        k_ego = -1 if t + 1 > ref_traj_0.shape[1] else t
        ego_state = ref.global_to_state(ref_traj_0[:, k_ego])

        x = np.array(ref_traj[0, :]).flatten()
        y = np.array(ref_traj[1, :]).flatten()

        plt.cla()
        plt.plot(xt, yt, label="track")
        plt.plot(ego_state.x, ego_state.y, "*", label="ego")
        plt.plot(onc_state.x, onc_state.y, "*", label="onc")
        plt.plot(obs_state.x, obs_state.y, "*", label="obs")
        plt.plot(obs_pose.pose.position.x,
                 obs_pose.pose.position.y, "*", label="obs_pose")
        plt.plot(x, y, label="ref")

        xr, yr = float(ref.traj_global[0, ref.n_r]), float(
            ref.traj_global[1, ref.n_r])
        plt.plot(xr, yr, "*", label="n_r")
        plt.legend()
        plt.axis("equal")
        plt.pause(0.01)
        if t > 20:
            obs_state = ref.local_to_state(
                ca.vertcat(S_OBS + 0.5, -0.25, 0, 0))


if __name__ == "__main__":
    test(0, 0)
    pass
