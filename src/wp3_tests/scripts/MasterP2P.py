#! /usr/bin/env python3

import itertools
import json
from pathlib import Path
from queue import Empty, Queue

import rospy
from std_msgs.msg import String

from wp3_tests.msg import Packet
from wp3_tests import (
    STATE_STARTUP, STATE_STANDBY, STATE_READY, STATE_RUNNING, STATE_FINISHED, STATE_TRANSACK,
    TRANS_NONE, TRANS_SETUP, TRANS_START, TRANS_STOP,
    SyncDict,
    load_param,
    blocker,
)

def who_sent(msg: Packet) -> str:
    return msg.header.frame_id

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
        self.USR_INP = load_param('usr_inp', False)

        ## Control Flow Resources

        # incoming: {p1: (t, msg),
        #            p2: (t, msg),
        #            p3: (t, msg)}
        self.incoming = {}

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

        self._trans = TRANS_NONE
        self.transition_pub = rospy.Publisher('transition', String, queue_size=10)
        self.transition_keep_alive_tmr = rospy.Timer(rospy.Duration(1/5),
                                                     lambda _: self.transition_pub.publish(String(self._trans)))

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

            ## wait for standby
            self.wait_for_state_all(STATE_STANDBY)

            ## prepare test-configuration params
            if self.USR_INP:
                rospy.loginfo(f'Press enter to start test {i:03}/{N:03}...')
                input()
            else:
                rospy.loginfo(f'Starting test {i:03}/{N:03}')
            conf = dict(zip(self.names, params))
            conf_s = json.dumps(conf)

            ## trans. standby -> ready w/ params
            self.transition_all_agents_and_wait(f'{TRANS_SETUP}={conf_s}', STATE_READY)

            ## trans. ready -> running
            self.transition_all_agents_and_wait(TRANS_START, STATE_RUNNING)

            ## wait for running -> finished
            print('waiting for finished')
            self.wait_for_state_all(STATE_FINISHED)
            print('reached finished')

            ## save to log file w/ params
            # create directory for this specific test
            dir = self.LOG_DIR / f'test_{i}'
            dir.mkdir(parents=True, exist_ok=True)
            rospy.loginfo(f'Saving test logs to {dir}')
            # save test-configuration
            with open(dir / 'conf', 'w') as f:
                f.write(conf_s)
                rospy.loginfo('Logged test configuration')
            # save output for each agent
            for agent in self.AGENTS:
                try:
                    _, msg = self.incoming[agent]
                except KeyError as e:
                    print(f'ERROR: Lost connection to {agent}')
                    raise e
                # dump the log data in agent-specific file
                # each message contain the logs for this entire test
                # encoding: ascii
                with open(dir / agent, 'wb') as f:
                    f.write(msg.data)
                rospy.loginfo(f'Logged {agent}')

            ## trans. finished -> standby
            self.transition_all_agents_and_wait(TRANS_STOP, STATE_STANDBY)

    def wait_for_agent_state(self, agent: str, state: str):
        @blocker
        def wait():
            try:
                _, msg = self.incoming[agent]
                return state == msg.state
            except KeyError:
                return False
        return wait()

    def wait_for_state_all(self, state: str):
        for agent in self.AGENTS:
            self.wait_for_agent_state(agent, state)
        rospy.loginfo(f'Agents in {state} state')

    def transition_all_agents_and_wait(self, trans: str, state: str) -> None:
        # tell which transition we want to do
        self._trans = trans
        # vehicles have acknowledge the transition and will begin
        # the transition
        self.wait_for_state_all(STATE_TRANSACK)
        # the vehicles wait to release the transition until NONE is sent
        self._trans = TRANS_NONE
        # wait for the vehicle to recognize the transition release
        self.wait_for_state_all(state)

    def receiver(self, msg: Packet):
        # arrival time is not really interesting in master but I'll keep for consistency
        now = rospy.Time.now()
        self.incoming[who_sent(msg)] = (now, msg)

if __name__ == '__main__':

    ## Start node ##

    with Master() as task:
        task()


