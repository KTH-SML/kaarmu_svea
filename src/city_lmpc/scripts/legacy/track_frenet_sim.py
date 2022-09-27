#! /usr/bin/env python3
from track_frenet_base import TrackFrenetBase
from svea.simulators.sim_SVEA import SimSVEA
from svea.models.bicycle import SimpleBicycleModel


class TrackFrenetSim(TrackFrenetBase):
    name = "track_frenet_sim"

    def __init__(self):
        super(TrackFrenetSim, self).__init__()
        if self.IS_SIM:
            x0, y0 = self.track.to_global(0.1, self.e_ref)
            self.state.x = x0
            self.state.y = y0
            self.state.yaw = self.phi_s0
            self.sim_model = SimpleBicycleModel(self.state)
            self.simulator = SimSVEA(self.sim_model,
                                     dt=self.dt,
                                     run_lidar=True,
                                     start_paused=False).start()


if __name__ == '__main__':

    ## Start node ##

    TrackFrenetSim.run()
