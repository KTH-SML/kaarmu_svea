#!/usr/bin/env python

import rospy
import numpy as np
from collections import deque
import os

from geometry_msgs.msg import PolygonStamped, PointStamped, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float64, Float64MultiArray
from svea.msg import PathSegment

from svea.models.bicycle import SimpleBicycleModel
from svea.models.states import VehicleState
from svea.simulators.sim_SVEA import SimSVEA
from svea.simulators.viz_utils import publish_3Dcar
from svea.simulators.viz_utils import publish_path, publish_target
from svea.controllers.control_interfaces import ControlInterface
from svea.controllers.share_controllers import sat, Pid

from geometry_msgs.msg import Twist
import tf
from delay_sim import publish_msg


class ControlTowerModule:
    """A special evaluation class for handling the teleoperation interface at the site of the operator. Methods for controlling the virtual vehicle.
    Used to evaluate teleoperation performance.

    :param vehicle_name: A name to identify the vehicle in the ControlInterface
    :type vehicle_name: str
    :param vehicle_model: A model object that provides all relevant vehicle state information during operation
    :type vehicle_model: SimpleBicycleState
    :param rate: Sampling frequency of the main loop. Should match the frequency of the model and simulator.
    :type rate: float
    :param eval_type: Specifies the type of evaluation one of either "test", "direct" or "indirect", defaults to "test"
    :type eval_type: str, optional
    """

    def __init__(self, vehicle_name, vehicle_model, rate = 50.0, eval_type = 'test'):
        """Constructor method
        """
        self.vehicle_name = vehicle_name

        # Publishers and broadcastersfor visualization and/or communication
        self.car_poly_pub = rospy.Publisher("/3D_car_twin", PolygonStamped, queue_size=1)
        self.pose_pub = rospy.Publisher("/robot_pose_twin", PoseStamped, queue_size=1)
        self.past_path_pub = rospy.Publisher("/past_path_twin", Path, queue_size=1)
        self.path_seg_pub = rospy.Publisher("/twin_path", PathSegment, queue_size=1)
        self.ref_path_pub = rospy.Publisher("/ref_path", Path, queue_size=1)
        self.vel_pub = rospy.Publisher("/twin_vel", Float64, queue_size=1)
        self.delayed_mouse_cmd_pub = rospy.Publisher("/mouse_vel_delayed", Twist, queue_size=1)
        self.latency_pub = rospy.Publisher("/latency", Float64, queue_size=1)
        self.lap_pub = rospy.Publisher("/lap", Float64, queue_size=1)
        self.br = tf.TransformBroadcaster() # Broadcaster for camera following in rviz
        self.ctrl_pub = rospy.Publisher("/twin_ctrl", Float64MultiArray, queue_size=1)

        # Test parameters
        self.eval_type = eval_type          # Type of evaluation 
        self.latencies = [0.05, 0.1, 0.3]   # Different latencies to simulate   
        self.latency = 0.01
        
        self.path_seg_msg = None        # Path segment message

        # Reference path
        self.ref_x = [-2.87312459946, -0.707916498184, 2.01363801956, 0.655872106552, 1.58852481842, 3.47432780266, 2.39330291748, -5.01371335983, -2.87312459946]
        self.ref_y = [-2.86331605911, -4.7171459198, -1.40990185738, 0.352610588074, 3.43946838379, 5.91061258316, 6.97385311127, -1.01934432983, -2.86331605911]

        # Rate
        self.dt = 1/rate
        self.rate = rospy.Rate(rate)

        # Control objects
        self.pid = Pid(K = 0.1, Ti = 1.5*self.dt, Td = 0.0*self.dt, b = 0.5, N = 5.0, dt = self.dt)
        self.ctrl_interface = ControlInterface(self.vehicle_name).start()
        while not self.ctrl_interface.is_ready and not rospy.is_shutdown():
            rospy.sleep(0.1)
        self.ctrl_interface.VELOCITY_DEADBAND = 1

        self.vel_cmd = 0.0          # velocity command received from operator
        self.steering_cmd = 0.0     # steering command received from operator
        self.v_upper = 2.0          # Default upper limit on telerobot's velocity
        self.v_lower = -2.0         # Default lower limit on telerobots's velocity
        self.delta = 0.9            # Fraction of upper and lower v_limits for virtual vehicle margin
        self.u_max = 1.0            # Maximum velocity signal
      
        if eval_type == 'direct':
            self.u_max = 0.9    
        self.steer_max = 35.0*np.pi/180.0   # max steering 35 degrees

        # Path for visualization in RVIZ
        self.past_path_x = deque(maxlen=200)
        self.past_path_y = deque(maxlen=200)
        self.past_path_yaw = deque(maxlen=200)

        # Model representing virtual vehicle
        self.model = vehicle_model

    def _update_key_cmd(self, key_msg):
        """Callback function for handling operator input with keyboard interface.
        """
        self.steering_cmd = sat(key_msg.angular.z, self.steer_max)
        self.vel_cmd = key_msg.linear.x

    def _update_mouse_cmd(self, mouse_msg):
        """Callback function for handling operator input with mouse interface.
        """
        self.steering_cmd = sat(mouse_msg.angular.z, self.steer_max)
        self.vel_cmd = max(0, mouse_msg.linear.x)

    def _update_v_limits(self, v_msg):
        """Callback function for handling estimation of maximum velocity of telerobot.
        """
        data = v_msg.data
        self.v_upper = data[0]
        self.v_lower = data[1]

    
    def _delay_mouse_cmd(self, msg):
        """Callback function for handling mouse control commands with simulated delay.
        """
        if self.eval_type == 'direct':
            # In direct control, simulate delay
            publish_msg(self.delayed_mouse_cmd_pub, msg, self.latency)
        else:
            publish_msg(self.delayed_mouse_cmd_pub, msg, 0.0)
    
    
    def _start_listen(self, input_type):
        """Start subscribers to read operator input and maximum velocity estimation from telerobot 
        with simulated delay if eval_type is "direct".

        :param input_type: Specifying the operator input method. Must be "key" for keyboard interface or "mouse" for mouse interface.
        :type input_type: str
        """
        if input_type == "key":
            rospy.Subscriber("/key_vel", Twist, self._update_key_cmd)
        elif input_type == "mouse":
            rospy.Subscriber("/mouse_vel", Twist, self._delay_mouse_cmd)
            rospy.Subscriber("/mouse_vel_delayed", Twist, self._update_mouse_cmd)
        else:
            print("ERROR bad control input method specified")
        rospy.Subscriber("/v_limits", Float64MultiArray, self._update_v_limits)

    def append_to_path_seg_msg(self, x, y, yaw, v, time):
        """Append one point to the PathSegment message.

        :param x: x-coordinate
        :type x: float
        :param y: y-coordinate
        :type y: float
        :param yaw: yaw angle
        :type yaw: float
        :param v: velocity
        :type v: float
        :param time: Corresponding ros timestamp.
        :type time: float
        """

        if self.path_seg_msg is None:
            self.path_seg_msg = PathSegment()

        self.path_seg_msg.x.append(x)
        self.path_seg_msg.y.append(y)
        self.path_seg_msg.yaw.append(yaw)
        self.path_seg_msg.v.append(v)
        self.path_seg_msg.timestamp.append(time)


    def publish_and_reset_path_segment_msg(self):
        """Publish the PathSegment message and clear the contents afterwards.
        Apply simulated latency in the case of indirect control. 
        """

        self.path_seg_msg.header.stamp = rospy.Time.now()
        if self.eval_type == 'indirect':
            publish_msg(self.path_seg_pub, self.path_seg_msg, self.latency)
        else:
            self.path_seg_pub.publish(self.path_seg_msg)
        self.path_seg_msg = None


    def start(self, input_type):
        """Start the subscribers and the main loop.

        :param input_type: Specifying the operator input method. Must be "key" for keyboard interface or "mouse" for mouse interface.
        :type input_type: str
        """
        self._start_listen(input_type)
        rospy.sleep(0.5)
        self._drive()

    def append_past_path(self, x,y,yaw):
        """Append pose information to visualize the path taken in RVIZ."""
        
        self.past_path_x.append(x)
        self.past_path_y.append(y)
        self.past_path_yaw.append(yaw)

    def get_current_state(self):
        """Return the current state of the virtual vehicle.

        :return: The current state (x,y,yaw,v)
        :rtype: (float, float, float, float)
        """
        return self.model.x, self.model.y, self.model.yaw, self.model.v


    def _drive(self):
        """The main loop for operating the virtual vehicle.
        """

        rospy.sleep(1.0)
        u = 0.0
        print("Driving")

        if not self.eval_type == 'test':
            # Publish the reference path for visualization in RVIZ
            publish_path(self.ref_path_pub, self.ref_x, self.ref_y)
    
        self.latency = self.latencies[0]    # Initial latency
        lap_count = 0                       # Current number of laps completed

        # Set number of laps to complete depending on eval_type
        if self.eval_type == "indirect":
            max_lap_count = 2
        else:
            max_lap_count = 3

        done_loop = False   # Indicates if one lap has been completed

        while not rospy.is_shutdown():
            # Record time and observe virtual vehicle state
            current_time = rospy.get_time()
            ego_x, ego_y, ego_yaw, ego_v = self.get_current_state()
            self.vel_pub.publish(ego_v)
            
            # Compute distance to the start point of the track and detect if a lap is completed
            end_dist = np.sqrt((ego_x - self.ref_x[0])**2 + (ego_y - self.ref_y[0])**2)
            if end_dist < 0.5 and done_loop and not self.eval_type == 'test':
                # Reaching end means new lap starts
                # Randomize latency every new lap
                self.latency = self.latencies[np.random.randint(0, len(self.latencies[1:]))+1]
                lap_count += 1
                print(str(lap_count) + " laps out of " + str(max_lap_count) + " completed")
                done_loop = False

            if end_dist >= 0.5:
                done_loop = True

            if lap_count >= max_lap_count:
                # Terminate when the correct number of laps have been completed
                print("Trial done. Terminating...")
                rospy.sleep(2.0)
                rospy.signal_shutdown("Target reached")
                os.system("rosnode kill -a")
                break
            
            # Publish latency and lap number for logging
            self.latency_pub.publish(self.latency)
            self.lap_pub.publish(lap_count)

            # Observe operator input and saturate with regards to estimated maximum velocity of telerobot
            vel_cmd = self.vel_cmd
            v_des = max(self.delta*self.v_lower, min(self.delta*self.v_upper, vel_cmd))
            
            # Determine velocity control signal with PID and saturate
            u = self.pid.get_ctrl(v_des, ego_v, increase_integrator = (abs(u) < self.u_max))
            u = sat(u, self.u_max)
            
            # Apply u and steering
            self.ctrl_interface.send_control(self.steering_cmd, u)

            # Publish information for logging at telerobot
            twin_ctrl_msg = Float64MultiArray()
            twin_ctrl_msg.data = [current_time, vel_cmd, u, v_des, ego_v, self.steering_cmd]
            self.ctrl_pub.publish(twin_ctrl_msg)

            # Append for communiction and visualization
            self.append_past_path(ego_x, ego_y, ego_yaw)
            self.append_to_path_seg_msg(ego_x, ego_y, ego_yaw, ego_v, rospy.get_time())

            # Publish and broadcast for RVIZ visualization
            publish_3Dcar(self.car_poly_pub, self.pose_pub,
                        self.model.x, self.model.y, self.model.yaw)
            publish_path(self.past_path_pub, list(self.past_path_x), list(self.past_path_y), list(self.past_path_yaw))
            self.publish_and_reset_path_segment_msg()
            q = tf.transformations.quaternion_from_euler(0,0,self.model.yaw)
            self.br.sendTransform((self.model.x, self.model.y, 0), (0,0,q[2], q[3]), rospy.Time.now(), self.vehicle_name, "map")
            self.rate.sleep()


def init():
    """Initialization handles use with just python or in a launch file
    """
    # grab parameters from launch-file
    start_pt_param = rospy.search_param('start_pt')
    eval_param = rospy.search_param('eval_type')
    input_param = rospy.search_param('ctrl_input')
    init_pt = [0.0, 0.0, 0.0, 0.0]

    start_pt = rospy.get_param(start_pt_param, init_pt)

    if type(start_pt) == type(''):
        start_pt = start_pt.split(',')
        start_pt = [float(curr) for curr in start_pt]

    input_type = rospy.get_param(input_param, "mouse")
    eval_type = rospy.get_param(eval_param, "direct")

    return start_pt, input_type, eval_type

def main():
    """Start rosnode, get launch file arguments, initialize model and simulator and start the virtual vehicle main loop.
    """
    vehicle_name = "SVEA_twin"
    rospy.init_node('control_tower_module')
    start_pt, input_type, eval_type = init()   # Read arguments from launch file

    # initialize simulated model and control interface
    bicycle_state = VehicleState(*start_pt)
    bicycle_model = SimpleBicycleModel(bicycle_state)
    ctm = ControlTowerModule(vehicle_name, bicycle_state, eval_type = eval_type)

    # start background simulation thread
    simulator = SimSVEA(vehicle_name, bicycle_model, 1.0/50.0)
    simulator.start()
    
    print("starting CTM")
    ctm.start(input_type)


if __name__ == "__main__":
    main()