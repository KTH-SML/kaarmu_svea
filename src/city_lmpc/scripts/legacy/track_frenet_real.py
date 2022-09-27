#! /usr/bin/env python3
from track_frenet_base import TrackFrenetBase


class TrackFrenetReal(TrackFrenetBase):
    name = "track_frenet_real"

    def __init__(self):
        super(TrackFrenetReal, self).__init__()


if __name__ == '__main__':

    ## Start node ##

    TrackFrenetReal.run()
