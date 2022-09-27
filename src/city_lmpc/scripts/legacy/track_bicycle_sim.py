#! /usr/bin/env python3
from track_bicycle_base import TrackBicycleBase
from svea.simulators.sim_SVEA import SimSVEA
from svea.models.bicycle import SimpleBicycleModel


class TrackBicycleSim(TrackBicycleBase):
    name = "track_bicycle_sim"

    def __init__(self):
        super(TrackBicycleSim, self).__init__()
        if self.IS_SIM:
            self.state.x = self.x[0, 0]
            self.state.y = self.x[1, 0]
            self.state.v = self.x[2, 0]
            self.state.yaw = self.x[3, 0]
            self.sim_model = SimpleBicycleModel(self.state)
            self.simulator = SimSVEA(self.sim_model,
                                     dt=self.dt,
                                     run_lidar=True,
                                     start_paused=False).start()


if __name__ == '__main__':

    ## Start node ##

    TrackBicycleSim.run()
