#! /usr/bin/env python3

import queue
from random import random
from threading import Timer
from pathlib import Path

import rospy

from wp3_tests.msg import Packet


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

class Vehicle:

    def __init__(self):

        ## Initialize node

        rospy.init_node('vehicle')

        ## Parameters

        self.NAME = load_param('name')
        self.MASTER = load_param('master')
        self.CLIENTS = load_param('clients')

        self.START_POS = load_param('start_pos')
        self.TARGET_VEL = load_param('target_vel')
        self.DELTA_TIME = load_param('delta_time')
        self.ENABLE_LOG = load_param('enable_log')

        self.DATA_SIZE = load_param('data_size')
        self.STATE_FREQ = load_param('state_freq')
        self.COMPUTE_TIME = load_param('compute_time')
        self.COMPUTE_ORDER = load_param('compute_order')

        self.log_path = Path(__file__).parent.parent
        self.log_path /= f'log/{rospy.Time.now()!s}.txt'
        self.log_path.parent.mkdir(parents=True, exist_ok=True)

        self.peers = [client
                      for client in self.CLIENTS
                      if client not in (self.NAME, self.MASTER)]
        self.compute_time = (
            self.COMPUTE_TIME
            * len(self.peers)**self.COMPUTE_ORDER
        )

        self.safe = True
        self.counter = 0
        self.x = self.START_POS
        self.v = self.TARGET_VEL

    def __enter__(self):

        ## General Resources

        self.timers = []
        def add_timer(delta, cb):
            dur = rospy.Duration(delta)
            tmr = rospy.Timer(dur, cb)
            self.timers.append(tmr)

        self.rate = rospy.Rate(1/self.DELTA_TIME)

        self.incoming = queue.Queue()

        self.log_file = open(self.log_path, 'w')

        ## Publishers

        self.outgoing_pub = rospy.Publisher('outgoing', Packet, queue_size=1)
        add_timer(1/self.STATE_FREQ, self.sender)

        ## Subscribers

        self.incoming_sub = rospy.Subscriber('incoming', Packet, self.receiver)

        ## Start test

        peers = set(self.peers)
        while peers:
            _, msg = self.incoming.get()
            if who_sent(msg) in peers:
                peers.remove(who_sent(msg))

        rospy.loginfo('received packet from all peers')

        rospy.wait_for_message('start', Empty)
        rospy.loginfo('ENTERING')

        # safety critical function
        add_timer(self.COMPUTE_TIME, self.compute)

        return self.main

    def __exit__(self, *exc):

        rospy.loginfo('EXITING')

        ## General Resources

        for timer in self.timers:
            timer.shutdown()

        self.log_file.close()

        return is_shutdown(exc)

    def main(self):
        while not rospy.is_shutdown():
            self.x -= self.v * self.DELTA_TIME
            self.x = max(self.x, 0)
            self.rate.sleep()

    def sender(self, _):
        msg = Packet()
        msg.header.frame_id = self.NAME
        msg.header.stamp = rospy.Time.now()
        msg.count = self.counter
        msg.data = random_bytes(self.DATA_SIZE * 1000)
        msg.chk = calculate_checksum(msg.data)
        self.outgoing_pub.publish(msg)
        self.counter += 1

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

        cols = [sender, sent, arrival, count, data, chk, headway, safe]
        self.log_file.write(' '.join(cols) + '\n')

        self.incoming.put((now, msg))

    def compute(self, _):
        peers = set(self.peers)
        headway = rospy.Duration(self.x / self.v)
        comp_time = rospy.Duration(self.COMPUTE_TIME)
        latency = {}
        try:
            while peers:
                arrival, msg = self.incoming.get(timeout=0.1)
                name = who_sent(msg)
                latency[name] = arrival - msg.header.stamp
                if name in peers:
                    peers.remove(name)
        except queue.Empty: pass
        else:
            self.safe = headway > max(latency.values()) + comp_time
        if not self.safe:
            rospy.signal_shutdown('Not Safe!')

if __name__ == '__main__':

    ## Start node ##

    with Vehicle() as task:
        task()


