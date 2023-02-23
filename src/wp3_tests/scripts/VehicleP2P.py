#! /usr/bin/env python3

import json
from queue import Queue, Empty
from threading import Lock
from typing import List

import rospy
from std_msgs.msg import String

from wp3_tests.msg import Packet
from wp3_tests import (
    STATE_STARTUP, STATE_STANDBY, STATE_READY, STATE_RUNNING, STATE_FINISHED, STATE_TRANSACK,
    TRANS_NONE, TRANS_SETUP, TRANS_START, TRANS_STOP, TRANS_INIT, TRANS_DONE,
    LOG_FIELD_SEP, LOG_ENTRY_SEP,
    Shutdown,
    load_param,
    fragile,
    checksum,
    random_bytes,
    assert_checksum,
)

def who_sent(msg: Packet) -> str:
    return msg.header.frame_id

class Vehicle:

    trans: str          # latest transition request
    state: str          # current state of vehicle
    _state_lock: Lock
    _trans_queue: Queue

    safe: bool
    counter: int
    x: float
    v: float

    log: List[str]

    outgoing_tmr: rospy.Timer
    simulate_tmr: rospy.Timer
    compute_tmr: rospy.Timer

    enable_incoming: bool

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

        self._timers = {}

        self._state_lock = Lock()

        self._trans_queue = Queue()

        # incoming: {p1: [(t0, msg), (t1, msg), (t2, msg)],
        #            p2: [(t0, msg), (t1, msg), (t2, msg)],
        #            p3: [(t0, msg), (t1, msg), (t2, msg)]}
        self.incoming = {name: Queue() for name in self.PEERS}

        ## State variables

        self.safe = False
        self.counter = 0
        self.x = 0
        self.v = 0

    def __enter__(self):

        self.state = STATE_STARTUP
        self.put_transition(TRANS_INIT)

        return self.main

    def __exit__(self, *_):

        for tmr in self._timers.values():
            tmr.shutdown()

        rospy.loginfo('EXITING')

    def new_timer(self, name, delta, cb):
        if name in self._timers:
            self._timers[name].shutdown()
        dur = rospy.Duration(delta)
        tmr = rospy.Timer(dur, cb)
        self._timers[name] = tmr
        return tmr

    def put_transition(self, trans: str):
        self._trans_queue.put(String(trans))

    def transition(self, msg: String):

        # partition the message at "=" (divide up the string into
        # everything left/right of "=") and apply strip on each part
        self.trans, _, value = map(str.strip, msg.data.partition('='))
        t = (self.state, self.trans)

        ## NONE transition

        skip = False
        skip |= self.trans == TRANS_NONE # explicitly do nothing
        skip |= self.state == STATE_TRANSACK # middle of transition

        if skip: return

        ## STARTUP -> STANDBY

        elif t == (STATE_STARTUP, TRANS_INIT):
            with switch_to(self, STATE_STANDBY):

                self.outgoing_pub = rospy.Publisher('outgoing', Packet, queue_size=1)
                self.outgoing_tmr = self.new_timer('outgoing', 1/self.STANDBY_FREQ, lambda _: self.empty_sender())

                self.enable_incoming = False
                self.incoming_sub = rospy.Subscriber('incoming',
                                                     Packet,
                                                     self.receiver)
                self.master_sub = rospy.Subscriber('transition',
                                                   String,
                                                   self._trans_queue.put)

                # check that at every peer has sent at least one message
                peers = self.PEERS[:]
                while peers:
                    if rospy.is_shutdown(): raise Shutdown
                    rospy.sleep(0.1)

                    peer = peers.pop(0)
                    if not self.incoming[peer].qsize():
                        peers.append(peer)

                rospy.loginfo('Connected to all peers')

                self.put_transition(TRANS_NONE)

        ## STANDBY -> READY

        elif t == (STATE_STANDBY, TRANS_SETUP):
            with switch_to(self, STATE_READY):

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
                self.enable_incoming = True

        ## READY -> RUNNING

        elif t == (STATE_READY, TRANS_START):
            with switch_to(self, STATE_RUNNING):

                self.outgoing_tmr = self.new_timer('outgoing',
                                                   1/self.conf['DATA_FREQ'],
                                                   lambda _: self.random_sender(self.conf['DATA_SIZE']))

                self.compute_tmr = self.new_timer('compute',
                                                  self.conf['COMP_TIME'],
                                                  self.compute)

                self.simulate_tmr = self.new_timer('simulate',
                                                   self.conf['TIME_STEP'],
                                                   self.simulate)

        ## RUNNING -> FINISHED

        elif t == (STATE_RUNNING, TRANS_DONE):
            with switch_to(self, STATE_FINISHED):

                self.compute_tmr.shutdown()
                self.simulate_tmr.shutdown()

                self.outgoing_tmr = self.new_timer('outgoing',
                                                   1/self.STANDBY_FREQ,
                                                   lambda _: self.log_sender(self.log))

                self.put_transition(TRANS_NONE)

        ## FINISHED -> STANDBY

        elif t == (STATE_FINISHED, TRANS_STOP):
            with switch_to(self, STATE_STANDBY):

                self.outgoing_tmr = self.new_timer('outgoing',
                                                   1/self.STANDBY_FREQ,
                                                   lambda _: self.empty_sender())

        ## RUNNING -> STANDBY

        elif t == (STATE_RUNNING, TRANS_STOP):
            with switch_to(self, STATE_STANDBY):

                self.compute_tmr.shutdown()
                self.simulate_tmr.shutdown()

                self.outgoing_tmr = self.new_timer('outgoing',
                                                   1/self.STANDBY_FREQ,
                                                   lambda _: self.empty_sender())

        ## READY -> STANDBY

        elif t == (STATE_READY, TRANS_STOP):
            with switch_to(self, STATE_STANDBY): pass

        ## STANDBY -> STANDBY

        elif t == (STATE_STANDBY, TRANS_STOP):
            with switch_to(self, STATE_STANDBY): pass

        ## Invalid transition

        else:

            rospy.logerr('ERROR: Invalid transition %s from state %s', t[1], t[0])

    def main(self):
        while not rospy.is_shutdown():
            # we try-catch to not lock ourselves during get operation
            # that way we can check if the node is shutdown every 1 sec
            try:
                msg = self._trans_queue.get(timeout=1)
                self.transition(msg)
            except Empty:
                pass

    def sender(self, data: bytes):
        msg = Packet()
        msg.header.frame_id = self.NAME
        msg.header.stamp = rospy.Time.now()
        msg.state = self.state
        msg.count = self.counter
        msg.data = data
        msg.chk = checksum(msg.data)
        self.outgoing_pub.publish(msg)
        self.counter += 1

    def empty_sender(self):
        self.sender(bytes())

    def random_sender(self, data_size):
        data = random_bytes(data_size * 1000)
        self.sender(data)

    def log_sender(self, log: List[str]):
        data = bytes(LOG_ENTRY_SEP.join(log).encode('ascii'))
        self.sender(data)

    def receiver(self, msg: Packet):

        now = rospy.Time.now()

        self.incoming[who_sent(msg)].put((now, msg))

        if self.state != STATE_RUNNING:
            return

        headway = str(self.x / self.v)
        sender = who_sent(msg)
        sent = str(msg.header.stamp)
        arrival = str(now)
        count = str(msg.count)
        valid = str(assert_checksum(msg.data, msg.chk))
        safe = str(self.safe)

        fields = [sender, sent, arrival, count, valid, headway, safe]
        self.log.append(LOG_FIELD_SEP.join(fields))

    def simulate(self, _):

        if self.state != STATE_RUNNING:
            return

        self.x -= self.v * self.dt
        self.x = max(self.x, 0)
        rospy.loginfo(f'{self.state} (safe={self.safe}): x = {self.x}')

    def compute(self, _):

        if self.state != STATE_RUNNING:
            return

        start = rospy.Time.now()
        peers = self.PEERS[:]
        # it doesn't really make sense to do/wait to do computation on
        # data that hasn't reach us yet. So (for each peer) we pick up
        # the first packet was sent at t > target_time
        comp_time = rospy.Duration(self.conf['COMP_TIME']/2)
        target_time = start - comp_time
        headway = rospy.Duration(self.x / self.v)
        latency = {}
        # this is so we can do a trick in the end of `compute` to make
        # sure we don't put TRANS_DONE multiple times in case `compute`
        # is faster than the transitions.
        safety_check = self.safe
        try:
            while peers:
                peer = peers.pop(0)
                arrival, msg = self.incoming[peer].get(timeout=0.1)
                sent = msg.header.stamp
                latency[peer] = arrival - msg.header.stamp
                if not sent > target_time:
                    # put peer back and try to get a newer message
                    # (due to above stated reason)
                    peers.append(peer)
        except Empty:
            # for some peer, we haven't received data from time (target_time)
            # i.e. we cannot compute the safety-critical function and are unsafe
            safety_check = False
        else:
            compute_time = rospy.Duration(self.compute_time)
            safety_check = headway > max(latency.values()) + compute_time

        if self.safe and not safety_check:
            self.put_transition(TRANS_DONE)

        self.safe = safety_check

class switch_to(fragile):

    node: Vehicle

    def __init__(self, node, state):
        self.node = node
        self.state = state

    def __enter__(self):

        self = super().__enter__()

        self.node._state_lock.acquire()

        # do not execute transition if we are already in state
        if self.node.state == self.state:
            rospy.logerr('Already in state %s', self.state)
            raise fragile.Break

        self.node.state = STATE_TRANSACK

    def __exit__(self, *exc):

        self.node._state_lock.release()

        # wait until master releases transition
        #
        # processing of transitions (ie state switches)
        # will only happen in the main thread.
        # When in TRANSACK we want to wait with completing
        # the transition until we get NONE.
        # Since this is part of state switching (i.e. main thread)
        # we don't have another thread receiving and processing
        # transition requests, so we cannot simply `put_transition`
        # but we actually need to `_trans_queue.get` and call `transition`
        # ourselves.
        #
        # this mechanism is good for those states that release the
        # transition themselves as well, e.g. TRANS_INIT.
        while self.node.trans != TRANS_NONE:
            msg = self.node._trans_queue.get()
            self.node.transition(msg)

        self.node.state = self.state
        rospy.loginfo('Switched to %s', self.state)

        return super().__exit__(*exc)

if __name__ == '__main__':

    ## Start node ##

    with Vehicle() as task:
        task()

