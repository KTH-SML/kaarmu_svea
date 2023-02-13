#! /usr/bin/env python3

from collections.abc import Callable
from pathlib import Path
from queue import Empty, Queue
import itertools
import json

import rospy
from std_msgs.msg import String

from wp3_tests.msg import Packet

def block_until(pred: Callable[[], bool], *args, **kwargs):
    while not pred(*args, **kwargs): pass

def load_param(name, value=None):
    if value is None:
        assert rospy.has_param(name), f'Missing parameter "{name}"'
    return rospy.get_param(name, value)

def who_sent(msg: Packet) -> str:
    return msg.header.frame_id


STATE_STARTUP = 'STARTUP_S'
STATE_STANDBY = 'STANDBY_S'
STATE_READY = 'READY_S'
STATE_RUNNING = 'RUNNING_S'
STATE_FINISHED = 'FINISHED_S'

TRANS_NONE = 'NONE_T'
TRANS_SETUP = 'SETUP_T'
TRANS_START = 'START_T'
TRANS_STOP = 'STOP_T'


class Master:

    def __init__(self):

        ## Initialize node

        rospy.init_node('master')

        ## Parameters

        self.NAME = load_param('name')
        self.MASTER = load_param('master')
        self.CLIENTS = load_param('clients')

        self.AGENTS = [client
                       for client in self.CLIENTS
                       if client not in (self.NAME, self.MASTER)]

        self.INIT_POS_LIST = load_param('init_pos')
        self.TARG_VEL_LIST = load_param('targ_vel')
        self.TIME_STEP_LIST = load_param('time_step')
        self.DATA_SIZE_LIST = load_param('data_size')
        self.DATA_FREQ_LIST = load_param('data_freq')
        self.COMP_TIME_LIST = load_param('comp_time')
        self.COMP_ORDR_LIST = load_param('comp_ordr')

        self.LOG_DIR = Path(load_param('log_dir')) / str(rospy.Time.now())

        ## Control Flow Resources

        # incoming: {p1: [(t0, msg), (t1, msg), (t2, msg)],
        #            p2: [(t0, msg), (t1, msg), (t2, msg)],
        #            p3: [(t0, msg), (t1, msg), (t2, msg)]}
        self.incoming = {name: Queue() for name in self.AGENTS}

    def __enter__(self):

        self.program = itertools.product(
            self.INIT_POS_LIST,
            self.TARG_VEL_LIST,
            self.TIME_STEP_LIST,
            self.DATA_SIZE_LIST,
            self.DATA_FREQ_LIST,
            self.COMP_TIME_LIST,
            self.COMP_ORDR_LIST,
        )
        self.names = [
            'INIT_POS',
            'TARG_VEL',
            'TIME_STEP',
            'DATA_SIZE',
            'DATA_FREQ',
            'COMP_TIME',
            'COMP_ORDR',
        ]

        self.transition_pub = rospy.Publisher('transition', String, queue_size=10)

        self.incoming_sub = rospy.Subscriber('incoming', Packet, self.receiver)

        # make sure clients are in standby
        return self.main

    def __exit__(self, *exc):
        # make sure clients are in standby
        rospy.loginfo('EXITING')

    def main(self):

        N = 1
        for n in map(len, [
            self.INIT_POS_LIST,
            self.TARG_VEL_LIST,
            self.TIME_STEP_LIST,
            self.DATA_SIZE_LIST,
            self.DATA_FREQ_LIST,
            self.COMP_TIME_LIST,
            self.COMP_ORDR_LIST,
        ]): N *= n

        for i, params in enumerate(self.program):

            ## prepare test-configuration params
            rospy.loginfo(f'Preparing test {i:03}/{N:03}')
            conf = dict(zip(self.names, params))
            conf_s = json.dumps(conf)

            ## wait for standby
            for agent in self.AGENTS:
                self.wait_for_agent_state(agent, STATE_STANDBY)
            rospy.loginfo(f'Agents in {STATE_STANDBY} state')

            ## trans. standby -> ready w/ params
            self.transition_all_agents(f'{TRANS_SETUP}={conf_s}')

            ## wait for ready
            for agent in self.AGENTS:
                self.wait_for_agent_state(agent, STATE_READY)
            rospy.loginfo(f'Agents in {STATE_READY} state')

            ## trans. ready -> running
            self.transition_all_agents(TRANS_START)

            ## wait for finished
            for agent in self.AGENTS:
                self.wait_for_agent_state(agent, STATE_FINISHED)
            rospy.loginfo(f'Agents in {STATE_FINISHED} state')

            ## save to log file w/ params
            # create directory for this specific test
            dir = self.LOG_DIR / f'test_{i}'
            dir.mkdir(parents=True, exist_ok=True)
            # save test-configuration
            with open(dir / 'conf') as f:
                f.write(conf_s)
            # save output for each agent
            for agent in self.AGENTS:
                try:
                    _, msg = self.incoming[agent].get(timeout=5)
                except Empty as e:
                    e.add_note(f'Lost connection to {agent}')
                    raise e
                # dump the log data in agent-specific file
                # each message contain the logs for this entire test
                # encoding: utf-8
                with open(dir / agent, 'wb') as f:
                    f.write(msg.data)

            # trans. finished -> standby

    def wait_for_agent_state(self, agent: str, state: str) -> None:
        def pred() -> bool:
            try:
                _, msg = self.incoming[agent].get_nowait()
                return state == msg.state
            except Empty:
                return False
        return block_until(pred)

    def transition_all_agents(self, trans: str) -> None:
        msg = String()
        msg.data = trans
        self.transition_pub.publish(msg)

    def receiver(self, msg: Packet):
        # arrival time is not really interesting in master but I'll keep for consistency
        now = rospy.Time.now()
        self.incoming[who_sent(msg)].put((now, msg))

if __name__ == '__main__':

    ## Start node ##

    with Master() as task:
        task()


