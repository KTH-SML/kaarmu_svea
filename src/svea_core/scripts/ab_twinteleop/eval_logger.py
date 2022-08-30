#!/usr/bin/env python

import rospy
import math
import numpy as np
import datetime
import os
import tf

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float64, Float64MultiArray

from svea_arduino.msg import lli_ctrl
from geometry_msgs.msg import Twist


class EvalLogger:
    """Class for logging data during teleoperation trials. Subscribes to topics, which the telerobot and virtual vehicle publishes to
    and records this through regular sampling.
    """


    def __init__(self, eval_type = "direct"):
        """Constructor method
        """
        self.twin_x = np.nan
        self.twin_y = np.nan
        self.twin_yaw = np.nan
        self.twin_vel = np.nan

        self.real_x = np.nan
        self.real_y = np.nan
        self.real_yaw = np.nan
        self.real_vel = np.nan

        self.vel_cmd = np.nan
        self.steer_cmd = np.nan

        self.twin_u = np.nan
        self.twin_steer = np.nan

        self.real_u = np.nan
        self.real_steer = np.nan

        self.lap = np.nan

        self.log = []
        currentDT = datetime.datetime.now()

        self.id = currentDT.strftime("%Y%m%d_%H%M%S")
        self.eval_type = eval_type
        self.latency = np.nan


    def start_logging(self):
        """Starts the logging loop that in every iteration records the currently read.
        """
        r = rospy.Rate(30)

        while not rospy.is_shutdown():
            log_line = [rospy.get_time(), self.twin_x, self.twin_y, self.twin_yaw, self.twin_vel, self.real_x, self.real_y, self.real_yaw,
             self.real_vel, self.vel_cmd, self.steer_cmd, self.twin_u, self.twin_steer, self.real_u, self.real_steer]
            log_line.append(self.latency)
            log_line.append(self.lap)
            self.log.append(log_line)
            r.sleep()

    def start_listen(self, input_type):
        """Start subscribers.
        """

        # control commands
        if input_type == "key":
            rospy.Subscriber("/key_vel", Twist, self._update_key_cmd)
        elif input_type == "mouse":
            rospy.Subscriber("/mouse_vel", Twist, self._update_mouse_cmd)
        else:
            print("ERROR bad control input method specified")

        # vlimits
        rospy.Subscriber("/v_limits", Float64MultiArray, self._update_v_limits)

        # poses
        rospy.Subscriber("/robot_pose_twin", PoseStamped, self._update_twin_pose)
        rospy.Subscriber("/robot_pose_sim", PoseStamped, self._update_real_pose)

        # controls
        rospy.Subscriber("/SVEA_twin/lli/ctrl_request", lli_ctrl, self._update_twin_ctrl_cmd)
        rospy.Subscriber("/SVEA_telerobot/lli/ctrl_request", lli_ctrl, self._update_real_ctrl_cmd)

        # latency
        rospy.Subscriber("/latency", Float64, self._update_latency)

        # lap count
        rospy.Subscriber("/lap", Float64, self._update_lap)

        # velocities
        rospy.Subscriber("/twin_vel", Float64, self._update_twin_vel)
        rospy.Subscriber("/svea_v", Float64, self._update_real_vel)

            
    def _update_lap(self, lap_msg):
        self.lap = lap_msg.data

    def _update_latency(self, lat_msg):
        self.latency = lat_msg.data

    def _update_twin_vel(self, vel_msg):
        self.twin_vel = vel_msg.data

    def _update_real_vel(self, vel_msg):
        self.real_vel = vel_msg.data

    def _update_key_cmd(self, key_msg):
        self.steer_cmd = math.radians(key_msg.angular.z)
        self.vel_cmd = key_msg.linear.x

    def _update_mouse_cmd(self, mouse_msg):
        self.steer_cmd = mouse_msg.angular.z
        self.vel_cmd = mouse_msg.linear.x

    def _update_v_limits(self, v_msg):
        data = v_msg.data
        self.v_upper = data[0]
        self.v_lower = data[1]

    def _update_real_pose(self, pose_msg):
        self.real_x = pose_msg.pose.position.x
        self.real_y = pose_msg.pose.position.y
        q = pose_msg.pose.orientation
        euler = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.real_yaw = euler[2]

    def _update_twin_pose(self, pose_msg):
        self.twin_x = pose_msg.pose.position.x
        self.twin_y = pose_msg.pose.position.y
        q = pose_msg.pose.orientation
        euler = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.twin_yaw = euler[2]

    def _update_twin_ctrl_cmd(self, ctrl_msg):
        self.twin_u = ctrl_msg.velocity
        self.twin_steer = ctrl_msg.steering

    def _update_real_ctrl_cmd(self, ctrl_msg):
        self.real_u = ctrl_msg.velocity
        self.real_steer = ctrl_msg.steering


    def _save_log(self):
        """Saves the recorded log as a CSV file in the home directory as "twinteleop_data/twinteleop_eval_TYPE_yyyymmdd_hhmmss.csv"
        """
        data = np.asarray(self.log)
        directory = os.path.expanduser('~/twinteleop_data')
        if not os.path.exists(directory):
            os.makedirs(directory)
        np.savetxt(os.path.expanduser('~/twinteleop_data/twinteleop_eval_'+self.eval_type+'_'+self.id+'.csv'), data, delimiter=",",
        header="time, twin_x, twin_y, twin_yaw, twin_vel, real_x, real_y, real_yaw, real_vel, vel_cmd, steer_cmd, twin_u, twin_steer, real_u, real_steer, latency, lap")



def init():
    """Initialization handles use with just python or in a launch file
    """
    # grab parameters from launch-file
    input_param = rospy.search_param('ctrl_input')
    input_type = rospy.get_param(input_param, "mouse")
    eval_param = rospy.search_param('eval_type')
    eval_type = rospy.get_param(eval_param, "direct")
    return input_type, eval_type

def main():
    """Starts the node, reads arguments from launch file, starts subscribers, defines shutdown action and starts the main loop.
    """
    rospy.init_node('eval_logger_node')
    print("starting logger node")
    input_type, eval_type = init()   # Read arguments from launch file
    el = EvalLogger(eval_type=eval_type)
    el.start_listen(input_type)

    if not eval_type=='test':
        rospy.on_shutdown(el._save_log)
    el.start_logging()


if __name__ == "__main__":
    main()
