
import numpy as np

from city_lmpc.models.track import ArcByAngle, ArcByLength, Track

ARCS = [
    ArcByLength(0, 1.21),
    ArcByAngle(-1 / 1.15, 90),
    ArcByLength(0, 2.15),
    ArcByAngle(1 / 1.15, 90),
    ArcByLength(0, 1.21),
]

LEN_STRT = 3.5
LEN_LANE = 0.25
ARCS_BENDY = [
    ArcByLength(0, .50),
    # turn right
    ArcByAngle(-1 / np.sqrt(2) / .65, 90),
    # drive straight
    ArcByLength(0, LEN_STRT / 2 - 6 * LEN_LANE),
    #   begin left bend
    ArcByAngle(1 / LEN_LANE, 90),
    #   end left bend
    ArcByAngle(-1 / LEN_LANE, 90),
    # drive straight on left lane
    ArcByLength(0, 5 * LEN_LANE),
    # begin bend right
    ArcByAngle(-1 / LEN_LANE, 90),
    # end bend right
    ArcByAngle(1 / LEN_LANE, 90),
    # drive on right lane
    ArcByLength(0, 3 * LEN_LANE),
    # turn left
    ArcByAngle(1 / np.sqrt(2) / .65, 90),
    ArcByLength(0, .50),
]

TRACK_CHOOSER = {
    'arcs': ARCS,
    'arcs_bendy': ARCS_BENDY,
}

LANE_WIDTH = 0.25
LANE2DIFF = {
    'left': LANE_WIDTH,
    'right': -LANE_WIDTH,
    'center': 0,
}

def adjust_for_lane(lane, x_s0, y_s0, phi_s0):
    diff = LANE2DIFF.get(lane, 0)
    x_s0 -= diff * np.sin(phi_s0)
    y_s0 += diff * np.cos(phi_s0)
    return x_s0, y_s0, phi_s0

def get_track(arcs, lane, s0, flip):
    x_s0, y_s0, phi_s0 = adjust_for_lane(lane, *s0)
    diff = LANE2DIFF.get(lane, 0)
    return Track(
        arcs,
        x_s0=x_s0,
        y_s0=y_s0,
        phi_s0=phi_s0,
        diff=diff,
        flip=flip,
    )
