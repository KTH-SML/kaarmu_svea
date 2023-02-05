#! /usr/bin/env python3

from collections import deque

import rospy

from rosgraph_msgs.msg import Clock

from wp3_tests.srv import TimeMeasure

class SyncedClock:
    """

    ```TimeMeasure.srv
    time t0
    time t1
    ---
    time t2
    time t3
    ```

    """

    def __enter__(self):

        self.RATE = load_param('~rate', 10)
        self.CLOCK_SERVICES = load_param('~clock_services', [])

        self.measure_srv = rospy.Service('measure', TimeMeasure, self.measure_cb)
        self.clock_services = deque(
            wait_and_create_sprox(srv, TimeMeasure)
            for srv in self.CLOCK_SERVICES
        )

        self.clock = Clock()
        self.clock_pub = rospy.Publisher('/clock', Clock, queue_size=10)

        self.rate = rospy.Rate(self.RATE)

        return self.main

    def __exit__(self, *exc):
        return None

    def main(self):
        while not rospy.is_shutdown():
            self.clock_services.rotate()
            clock_service = self.clock_services[0]
            measurement = self.measure_clock(clock_service)
            self.time_sync(measurement)
            self.rate.sleep()

    def time_sync(self, meas: TimeMeasure):
        delay = ((meas.t1 - meas.t0) + (meas.t2 - meas.t3)) / 2

        # TODO


    def write_clock(self):
        self.clock_pub.publish(self.clock)

    def change_rate(self, hz):
        self.rate.sleep_dur = rospy.Duration(0, int(1e9/hz))

    def measure_clock(self, clock_service) -> TimeMeasure:
        req = TimeMeasure()
        req.t0 = rospy.Time.now()
        return clock_service(req)

    def measure_cb(self, req: TimeMeasure) -> TimeMeasure:
        t1 = rospy.Time.now()
        rep = TimeMeasure()
        rep.t0 = req.t0
        rep.t1 = t1
        rep.t2 = rospy.Time.now()
        return rep

def wait_and_create_sprox(name, service_type):
    rospy.wait_for_service(name)
    return rospy.ServiceProxy(name, service_type)

def load_param(name, value=None):
    if value is None:
        assert rospy.has_param(name), f'Missing parameter "{name}"'
    return rospy.get_param(name, value)


def run(cls, *a, **k):
    with cls(*a, **k) as task:
        task()

if __name__ == '__main__':

    run(SyncedClock)
