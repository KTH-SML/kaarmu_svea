#! /usr/bin/env python3
from track_bicycle_base import TrackBicycleBase


class TrackBicycleReal(TrackBicycleBase):
    name = "track_bicycle_real"

    def __init__(self):
        super(TrackBicycleReal, self).__init__()


if __name__ == '__main__':

    ## Start node ##

    TrackBicycleBase.run()
