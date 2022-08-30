
import threading
import rospy

"""Functions for simulating communication delay.
"""

def _delayed_publish(publisher, msg, delay):
    """Publishes the given message after the given delay (in seconds)
    """
    rospy.sleep(delay)
    publisher.publish(msg)

def publish_msg(publisher, msg, delay):
    """Publishes the given message msg with the given publisher after the specified delay (in seconds) 
    without introducing the delay in the caller function. 
    """
    pub_thread = threading.Thread(target=_delayed_publish, args=(publisher, msg, delay))
    pub_thread.start()
