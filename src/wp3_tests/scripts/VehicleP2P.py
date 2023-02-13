#! /usr/bin/env python3

import json
from queue import Queue, Empty
from collections.abc import Callable
from random import random
from threading import Lock
from contextlib import contextmanager, ContextDecorator
from pathlib import Path
from typing import List

import rospy
from std_msgs.msg import String

from wp3_tests.msg import Packet

class fragile(ContextDecorator):
    """
    Abortable contextmanagers

    [source](https://stackoverflow.com/a/23665658)
    """

    class Break(Exception):
      """Break out of the with statement"""

    def __init__(self, fn):
        self.fn = fn

    def __call__(self, *args, **kwargs):
        return self.fn(*args, **kwargs)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return exc[0] is self.Break

def block_until(pred: Callable[[], bool], *args, **kwargs):
    while not pred(*args, **kwargs): pass

def random_bytes(n):
    return bytes(int(random() * 255) for _ in range(n))

def load_param(name, value=None):
    if value is None:
        assert rospy.has_param(name), f'Missing parameter "{name}"'
    return rospy.get_param(name, value)

def who_sent(msg: Packet) -> str:
    return msg.header.frame_id

def calculate_checksum(data: bytes) -> int:
    chk: int = 0
    for byte in data:
        chk ^= byte
    return chk

def assert_checksum(data: bytes, chk: int) -> bool:
    for byte in data:
        chk ^= byte
    return chk == 0

def new_timer(delta, cb):
    dur = rospy.Duration(delta)
    tmr = rospy.Timer(dur, cb)
    return tmr


STATE_STARTUP = 'STARTUP_S'
STATE_STANDBY = 'STANDBY_S'
STATE_READY = 'READY_S'
STATE_RUNNING = 'RUNNING_S'
STATE_FINISHED = 'FINISHED_S'

# externally triggered
TRANS_NONE = 'NONE_T'
TRANS_SETUP = 'SETUP_T'
TRANS_START = 'START_T'
TRANS_STOP = 'STOP_T'

# internally triggered
TRANS_INIT = 'INIT_T'
TRANS_DONE = 'SIM_T'

LOG_FIELD_SEP = ','
LOG_ENTRY_SEP = ';'


class Vehicle:

    state: str
    _state_lock: Lock

    safe: bool
    counter: int
    x: float
    v: float

    log: List[str]

    outgoing_tmr: rospy.Timer
    simulate_tmr: rospy.Timer
    compute_tmr: rospy.Timer

    def __init__(self):

        ## Initialize node

        rospy.init_node('vehicle')

        ## Parameters

        self.NAME = load_param('name')
        self.MASTER = load_param('master')
        self.CLIENTS = load_param('clients')
        self.STANDBY_FREQ = load_param('~standby_freq')

        self.PEERS = [client
                      for client in self.CLIENTS
                      if client not in (self.NAME, self.MASTER)]

        ## Control Flow Resources

        self._state_lock = Lock()

        # incoming: {p1: [(t0, msg), (t1, msg), (t2, msg)],
        #            p2: [(t0, msg), (t1, msg), (t2, msg)],
        #            p3: [(t0, msg), (t1, msg), (t2, msg)]}
        self.incoming = {name: Queue() for name in self.PEERS}

    @fragile
    @contextmanager
    def switch_to(self, state: str):
        self._state_lock.acquire()

        # do not execute transition if we are already in state
        if self.state == state: raise fragile.Break

        self.state = state
        yield
        rospy.loginfo('Switch to %s', self.state)

        self._state_lock.release()

    def wait_for_state(self, state: str):
        return block_until(lambda: self.state == state)

    def transition(self, msg: String):

        # partition the message at "=" (divide up the string into
        # everything left/right of "=") and apply strip on each part
        trans, _, value = map(str.strip, msg.data.partition('='))
        t = (self.state, trans)

        ## NONE transition

        if trans == TRANS_NONE: return # do nothing

        ## STARTUP -> STANDBY

        if t == (STATE_STARTUP, TRANS_INIT):
            with self.switch_to(STATE_STANDBY):

                self.outgoing_pub = rospy.Publisher('outgoing', Packet, queue_size=1)
                self.outgoing_tmr = new_timer(1/self.STANDBY_FREQ, lambda _: self.empty_sender())

                self.master_sub = rospy.Subscriber('master', String, self.transition)
                self.incoming_sub = rospy.Subscriber('incoming', Packet, self.receiver)

                # check that at every peer has sent at least one message
                peers = self.PEERS[:]
                while peers:
                    peer = peers.pop(0)
                    if not self.incoming[peer].qsize():
                        peers.append(peer)

                rospy.loginfo('received packet from all peers')

        ## STANDBY -> READY

        if t == (STATE_STANDBY, TRANS_SETUP):
            with self.switch_to(STATE_READY):

                # self.conf: Dict[str, Any] = {
                #     'INIT_POS': ...,
                #     'TARG_VEL': ...,
                #     'TIME_STEP': ...,
                #     'DATA_SIZE': ...,
                #     'DATA_FREQ': ...,
                #     'COMP_TIME': ...,
                #     'COMP_ORDR': ...,
                # }
                self.conf = json.loads(value)

                self.log = []

                self.x = self.conf['INIT_POS']
                self.v = self.conf['TARG_VEL']
                self.dt = self.conf['TIME_STEP']
                self.safe = True
                self.compute_time = (
                    self.conf['COMP_TIME']
                    * len(self.PEERS)**self.conf['COMP_ORDR']
                )

        ## READY -> RUNNING

        if t == (STATE_READY, TRANS_START):
            with self.switch_to(STATE_RUNNING):

                self.outgoing_tmr.shutdown()

                self.outgoing_tmr = new_timer(1/self.conf['DATA_FREQ'],
                                              lambda _: self.random_sender(self.conf['DATA_SIZE']))

                self.compute_tmr = new_timer(self.conf['COMP_TIME'],
                                             self.compute)

                self.simulate_tmr = new_timer(self.conf['TIME_STEP'],
                                              self.simulate)

        ## RUNNING -> FINISHED

        if t == (STATE_RUNNING, TRANS_DONE):
            with self.switch_to(STATE_FINISHED):

                self.outgoing_tmr.shutdown()
                self.simulate_tmr.shutdown()
                self.compute_tmr.shutdown()

                self.outgoing_tmr = new_timer(1/self.STANDBY_FREQ, lambda _: self.log_sender(self.log))

        ## FINISHED -> STANDBY

        if t == (STATE_FINISHED, TRANS_STOP):
            with self.switch_to(STATE_STANDBY):

                self.outgoing_tmr.shutdown()
                self.outgoing_tmr = new_timer(1/self.STANDBY_FREQ, lambda _: self.empty_sender())

        ## RUNNING -> STANDBY

        if t == (STATE_RUNNING, TRANS_STOP):
            with self.switch_to(STATE_STANDBY):

                self.outgoing_tmr.shutdown()
                self.outgoing_tmr = new_timer(1/self.STANDBY_FREQ, lambda _: self.empty_sender())

        ## READY -> STANDBY

        if t == (STATE_READY, TRANS_STOP):
            with self.switch_to(STATE_STANDBY): pass

        rospy.logerr('ERROR: Invalid transition %s to %s', t[1], t[0])

    def __enter__(self):
        self.state = STATE_STARTUP
        self.transition(String(TRANS_INIT))
        return self.main

    def __exit__(self, *_):
        rospy.loginfo('EXITING')

    def main(self):
        rospy.spin()

    def sender(self, data: bytes):
        msg = Packet()
        msg.header.frame_id = self.NAME
        msg.header.stamp = rospy.Time.now()
        msg.count = self.counter
        msg.data = data
        msg.chk = calculate_checksum(msg.data)
        self.outgoing_pub.publish(msg)
        self.counter += 1

    def empty_sender(self):
        self.sender(bytes())

    def random_sender(self, data_size):
        data = random_bytes(data_size * 1000)
        self.sender(data)

    def log_sender(self, log: List[str]):
        data = LOG_ENTRY_SEP.join(log).encode('utf-8')
        self.sender(data)

    def receiver(self, msg: Packet):
        now = rospy.Time.now()

        headway = str(self.x / self.v)
        sender = who_sent(msg)
        sent = str(msg.header.stamp)
        arrival = str(now)
        count = str(msg.count)
        data = msg.data.hex()
        chk = str(msg.chk)
        safe = str(self.safe)

        fields = [sender, sent, arrival, count, data, chk, headway, safe]
        self.log.append(LOG_FIELD_SEP.join(fields))

        self.incoming[who_sent(msg)].put((now, msg))

        # we switch here because then we're assured safety breach is logged
        # at least once
        if not safe:
            self.transition(STATE_FINISHED)

    def simulate(self, _):
        self.x -= self.v * self.dt
        self.x = max(self.x, 0)

    def compute(self, _):
        start = rospy.Time.now()
        peers = self.PEERS[:]
        # it doesn't really make sense to do/wait to do computation on
        # data that hasn't reach us yet. So (for each peer) we pick up
        # the first packet was sent at t > target_time
        target_time = start - self.conf['COMP_TIME']/2
        headway = rospy.Duration(self.x / self.v)
        latency = {}
        try:
            while peers:
                peer = peers.pop(0)
                arrival, msg = self.incoming[peer].get(timeout=0.1)
                sent = msg.header.stamp
                latency[peer] = arrival - msg.header.stamp
                if sent > target_time:
                    peers.remove(peer)
        except Empty:
            # for some peer, we haven't received data from time (target_time)
            # i.e. we cannot compute the safety-critical function and are unsafe
            self.safe = False
        else:
            self.safe = headway > max(latency.values()) + self.compute_time

if __name__ == '__main__':

    ## Start node ##

    with Vehicle() as task:
        task()

