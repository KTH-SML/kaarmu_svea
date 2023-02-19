import string
from typing import Callable
from random import choices
from threading import Condition

import rospy


def load_param(name, value=None):
    if value is None:
        assert rospy.has_param(name), f'Missing parameter "{name}"'
    return rospy.get_param(name, value)


def block_until(pred: Callable[[], bool], *args, **kwargs):
    while not pred(*args, **kwargs):
        if rospy.is_shutdown(): raise Shutdown
        rospy.sleep(0.1)


def random_bytes(k):
    return bytes(map(ord, choices(string.ascii_letters, k=k)))


def checksum(data: bytes) -> int:
    chk: int = 0
    for byte in data:
        chk ^= byte
    return chk


def assert_checksum(data: bytes, chk: int) -> bool:
    for byte in data:
        chk ^= byte
    return chk == 0


class Shutdown(Exception): pass


class blocker:

    def __init__(self, pred: Callable[..., bool]):
        self._pred = pred

    def __call__(self, *args, **kwargs):
        while not self._pred(*args, **kwargs):
            if rospy.is_shutdown(): raise Shutdown
            rospy.sleep(0.1)


class fragile:
    """
    Abortable contextmanagers

    [source](https://stackoverflow.com/a/23665658)
    """

    class Break(Exception):
      """Break out of the with statement"""

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return exc[0] is self.Break


class SyncDict(dict):
    """
    Think Queue but dict-like.

    Why? Can be used when another thread updates the dict and
    you want to be sure that the access to the dict is synchronized,
    just like Queue!

    Main source of inspiration:
    - https://stackoverflow.com/questions/26767388/making-a-python-dictionary-threadsafe-like-queue
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.__cv = Condition()

    def put(self, key, value):
        with self.__cv:
            self[key] = value
            self.__cv.notify()

    def pop(self, key, default=..., wait=True, timeout=None):
        with self.__cv:
            if wait:
                self.__cv.wait_for(lambda: key in self, timeout)
            if default is Ellipsis:
                return super().pop(key)
            else:
                return super().pop(key, default)

