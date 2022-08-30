#!/usr/bin/env python

import sys
import os
import rospy
from timeit import default_timer
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from collections import deque
from path_handler import PathHandler, MapHandler, PathObject
from genetic_path_planner import GeneticPathPlanner

from geometry_msgs.msg import PolygonStamped, PointStamped, PoseStamped
from nav_msgs.msg import Path, OccupancyGrid
from std_msgs.msg import Float64, Float64MultiArray
from svea.msg import PathSegment

# from svea.models.bicycle import SimpleBicycleState
from svea.models.bicycle import SimpleBicycleModel
from svea.models.states import VehicleState
from svea.simulators.sim_SVEA import SimSVEA
from svea.simulators.viz_utils import plot_car, publish_3Dcar
from svea.simulators.viz_utils import publish_path, publish_target
from svea.controllers.control_interfaces import ControlInterface
from svea.controllers.share_controllers import MPC, sat, Pid

from geometry_msgs.msg import Twist
import tf



class TelerobotModule:
    """A class for handling motion control operations at the simulated telerobot.

    :param vehicle_name: A name to identify the vehicle in the ControlInterface
    :type vehicle_name: str
    :param vehicle_model: A model object that provides all relevant vehicle state information during operation
    :type vehicle_model: SimpleBicycleState
    """

    def __init__(self, vehicle_name, vehicle_model, is_eval = False):
        """Constructor method."""
        self.vehicle_name = vehicle_name

        # Publishers
        self.car_poly_pub = rospy.Publisher("/3D_car_sim", PolygonStamped, queue_size=1)
        self.pose_pub = rospy.Publisher("/robot_pose_sim", PoseStamped, queue_size=1)
        self.past_path_pub = rospy.Publisher("/past_path_sim", Path, queue_size=1)
        self.target_pub = rospy.Publisher("/target", PointStamped, queue_size=1)
        self.v_lim_pub = rospy.Publisher("/v_limits", Float64MultiArray, queue_size=1)
        self.v_pub = rospy.Publisher("/svea_v", Float64, queue_size=1)


        # Path objects
        self.lead_path_handler = PathHandler()
        self.planned_path_handler = PathHandler()
        self.planned_path = PathObject()

        # Past path for visualization
        self.past_path_x = deque(maxlen=200)
        self.past_path_y = deque(maxlen=200)
        self.past_path_yaw = deque(maxlen=200)

        # Time
        self.dt = 0.15                      # Sampling period
        self.rate = rospy.Rate(1/self.dt)   # Sampling rate
        self.t0 = None                      # Rostime when starting
        self.tmax = 60.0                    # Number of seconds to run the program

        # Control interface
        self.ctrl_interface = ControlInterface(self.vehicle_name).start()
        while not self.ctrl_interface.is_ready and not rospy.is_shutdown():
            rospy.sleep(0.1)
        self.ctrl_interface.VELOCITY_DEADBAND = 3

        # Indicates if the session is an evaluation trial
        self.is_eval = is_eval

        # Longitudinal control parameters
        self.th = 2.0                       # Time-headway
        self.u_max = 1.0                    # Maximum velocity command
        self.v0 = 2.0                       # Default estimated maximum velocity            
        self.v_lim = self.v0                # Estimated max velocity
        self.v_tau = 1.0                    # Time constant for low-pass filter in v_lim estimation
        self.v_alpha = self.dt/ \
            (self.dt + self.v_tau)          # Smoothing factor in low-pass filter
        self.delta_u = 0.5                  # Ratio of u/u_max above which maximum velocities are observed
        self.stop = False                   # Indicates if telerobot should stop

        # PID controller for longitudinal control
        self.pid = Pid(K = 1.0/self.th, Ti = 2.0*self.dt, Td = 0.0*self.dt, b = 1.0, N = 3.0, dt = self.dt)
        

        # MPC object for lateral control
        self.mpc = MPC(dt = self.dt, horizon_p = 2, horizon_c = 2, q=[1.0, 1.0, 0.05, 0.05], r = 0.02)
        
        # Lateral control parameters
        self.der_tau = 0.1                  # Time constant for low-pass filter in derivative estimation
        self.der_alpha = self.dt/ \
            (self.dt + self.der_tau)        # Smoothing factor in low-pass filter for derivative estimation
        self.steer_max = 35.0*np.pi/180.0   # Maximum steering angle 35 degrees
 
        self.go = False                     # Flag to indicate when all is set for telerobot to start

        # Model
        self.model = vehicle_model

        # Lists for logging for telerobot
        self.X = []
        self.Y = []
        self.YAW = []
        self.V = []
        self.VL = []    # V leader
        self.VHI = []
        self.VLO = []
        self.VDES = []
        self.SPACING = []
        self.SREF = []
        self.ES = []
        self.U = []
        self.T = []
        self.TARGET_X = []
        self.TARGET_Y = []
        self.EY = []
        self.EYAW = []
        self.EYDOT = []
        self.EYAWDOT = []
        self.CURV = []
        self.STEERING = []
        self.YAWDOT = []
        self.TMPC = []
        self.TTOT = []
        self.PATHS = []

        # List for logging for virtual vehicle
        self.TTWIN = []
        self.VCMD = []
        self.UTWIN = []
        self.VDESTWIN = []
        self.VTWIN = []
        self.TSTEER = []
        
        

    def _append_log(self, x, y, yaw, v, vl, v_des, spacing,s_ref, es, u, time,
    target_x, target_y, vhi, vlo, e0, curv, steering, mpc_time, tot_time):
        """Append data to lists for logging."""
        self.X.append(x)
        self.Y.append(y)
        self.YAW.append(yaw)
        self.V.append(v)
        self.VL.append(vl)
        self.VHI.append(vhi)
        self.VLO.append(vlo)
        self.VDES.append(v_des)
        self.SPACING.append(spacing)
        self.SREF.append(s_ref)
        self.ES.append(es)
        self.U.append(u)
        self.T.append(time)
        self.TARGET_X.append(target_x)
        self.TARGET_Y.append(target_y)
        self.EY.append(e0[0])
        self.EYAW.append(e0[2])
        self.EYDOT.append(e0[1])
        self.EYAWDOT.append(e0[3])
        self.CURV.append(curv)
        self.STEERING.append(steering)
        self.TTOT.append(tot_time)
        self.TMPC.append(mpc_time)

    def _make_long_plot(self):
        """Create plots with information relating to longitudinal control"""
        fig = plt.figure(0, figsize=(9, 6), dpi=120)
        ax1 = fig.add_subplot(2,2,1)
        ax2 = fig.add_subplot(2,2,2)
        ax3 = fig.add_subplot(2,1,2)
        fig.tight_layout()
        fig.subplots_adjust(left=0.1, bottom=0.10, right=None, top=None, wspace=None, hspace=0.25)

        # Unsubscribe from topics
        self.twin_path_sub.unregister()
        self.planned_path_sub.unregister()
        self.twin_ctrl_sub.unregister()
        
        # Create time vectors with baseline zero
        t0 = min(self.T[0], self.TTWIN[0])  # First time in rostime
        T = [t-t0 for t in self.T]          # Time vector for telerobot
        TTWIN = [t-t0 for t in self.TTWIN]  # Time vector for virtual vehicle aka. the twin

        # Spacing plot
        ax1.plot(T,self.SPACING,'b',label=r'$s$')
        ax1.plot(T,self.SREF,'g--', label = r'$s_r$')
        ax1.plot(T,self.ES,'r', label=r'$e_s$')
        ax1.legend()
        ax1.set_title("Spacing")
        ax1.set_xlabel("Time [s]")
        ax1.set_ylabel("Distance [m]")
        ax1.grid()
        ax1.set_xlim([0, T[-1]+3])

        # Control signals plot
        ax2.step(T, self.U, 'b', label = r'$u_T$', where='post')
        ax2.step(TTWIN, self.UTWIN,'g', label = r'$u_V$', where='post')
        ax2.set_title("Control signals")
        ax2.set_xlabel("Time [s]")
        ax2.legend()
        ax2.grid()
        ax2.set_xlim([0, T[-1]+3])
        
        # Velocities plot
        ax3.plot(T, self.V,'b',linewidth = 2, label=r'$v_T$')
        ax3.plot(TTWIN, self.VTWIN,'g',linewidth = 2, label=r'$v_V$')
        ax3.plot(TTWIN, self.VDESTWIN,'g--', label = r'$\tilde{v}_V$')
        ax3.plot(T, self.VHI,'r', label=r'$v_\mathrm{max}$')
        ax3.plot(TTWIN, self.VCMD,'black', label = r'$v_r$')
        ax3.set_xlim([0, T[-1]+4])
        ax3.set_ylim([-0.25, 1.2])
        ax3.legend()
        ax3.set_title("Velocities")
        ax3.set_xlabel("Time [s]")
        ax3.set_ylabel("Velocity [m/s]")
        ax3.grid()

    def _make_lat_plot(self):
        """Create plots with information relating to lateral control"""
        fig1 = plt.figure(1, figsize=(10, 6), dpi=110)
        ax1 = fig1.add_subplot(2,2,2)   # Errors
        ax2 = fig1.add_subplot(2,2,4)   # Steering control
        ax3 = fig1.add_subplot(1,2,1)   # Paths   
        fig1.tight_layout()
        fig1.subplots_adjust(left=0.08, bottom=0.10, right=0.92, top=None, wspace=0.32, hspace=0.34)
        
        
        fig2 = plt.figure(2)
        ax5 = fig2.add_subplot(1,2,1)   # Time
        ax6 = fig2.add_subplot(1,2,2)   # Derivatives
        
        # Unsubscribe from topics
        self.twin_path_sub.unregister()
        self.planned_path_sub.unregister()
        self.twin_ctrl_sub.unregister()

        # Create time vectors with baseline zero
        t0 = min(self.T[0], self.TTWIN[0])  # First time in rostime
        T = [t-t0 for t in self.T]          # Time vector for telerobot
        TTWIN = [t-t0 for t in self.TTWIN]  # Time vector for virtual vehicle aka. the twin

        # MPC Errors
        ax1.plot(T,self.EY, 'b', label=r'$e_y$')
        ax11 = ax1.twinx()
        ax11.plot(T,self.EYAW, 'g', label=r'$e_\theta$')
        ax11.set_ylabel('Angle [rad]', color = 'g')
        ax11.tick_params(axis='y', labelcolor='g')
        ax1.tick_params(axis='y', labelcolor='b')
        ax11.set_ylim([-0.5, 0.5])
        ax1.set_ylim([-0.3, 0.3])
        ax1.legend(loc = 2)
        ax11.legend(loc = 1)
        ax1.set_title("Errors")
        ax1.set_xlabel("Time  [s]")
        ax1.set_ylabel("Distance [m]", color = 'b')
        ax1.grid()

        # Steering Control signals
        ax2.step(TTWIN, self.TSTEER, 'g', label = r"$\delta_V$", where='post')
        ax2.step(T, self.STEERING, 'b', label = r"$\delta_T$", where='post')
        ax2.set_title("Steering")
        ax2.set_xlabel("Time [s]")
        ax2.set_ylabel('Angle [rad]')
        ax2.legend(ncol =2)
        ax2.grid()

        # paths
        lead_path = self.lead_path_handler.get_path()
        lead_x, lead_y = lead_path.get_xy()
        ax3.plot(lead_x, lead_y,'g', label = "VV")
        ax3.plot(self.X, self.Y,'b',label = "TR")
        ax3.scatter(lead_x[0], lead_y[0], c = 'r', label = "Start", s = 300)
        ax3.legend(loc = 4, scatterpoints=1)
        ax3.set_title("Paths")
        ax3.set_xlabel("x [m]")
        ax3.set_ylabel("y [m]")
        ax3.axis("equal")
        
        # Print computation time info
        avg_tmpc = np.mean(self.TMPC)
        std_tmpc = np.std(self.TMPC)
        print("svg and std_mpc time", avg_tmpc, std_tmpc)

        # Derivatives
        ax6.plot(T,self.EYDOT, label=r'$\dot{e}_y$')
        ax6.plot(T,self.EYAWDOT, label=r'$\dot{e}_\theta$')

        # Plot computation time, total and MPC
        ax5.plot(T, self.TMPC, label="MPC")
        ax5.plot(T, self.TTOT, label="TOT")
        ax5.axhline(y=self.dt, c='k',ls = "--", label = "dt")
        ax5.legend()
        ax5.set_title("Computation times")
        ax5.set_xlabel("Time")
        ax5.set_ylabel("Time")
        ax5.grid()

    def _make_complot(self):
        """Create plots with information relating to overal teleoperation performance"""
        fig1 = plt.figure(3, figsize=(10, 6), dpi=110)
        gs = gridspec.GridSpec(ncols=2, nrows=2, width_ratios= [1.75, 1.0] )
        ax3 = fig1.add_subplot(gs[:, 0])
        ax31 = fig1.add_subplot(gs[0, 1])
        ax32 = fig1.add_subplot(gs[1, 1])
        fig1.tight_layout()
        fig1.subplots_adjust(left=0.08, bottom=0.10, right=0.97, top=None, wspace=0.2, hspace=0.25)
        
        fig2 = plt.figure(4, figsize=(10, 6), dpi=110)
        ax4 = fig2.add_subplot(2,1,1)                     
        ax5 = fig2.add_subplot(2,1,2, sharex = ax4)       
        fig2.tight_layout()
        fig2.subplots_adjust(left=0.08, bottom=0.10, right=0.97, top=None, wspace=0.2, hspace=0.2)

        # Unsubscribe from topics
        self.twin_path_sub.unregister()
        self.planned_path_sub.unregister()
        self.twin_ctrl_sub.unregister()
        
        # Create time vectors with baseline zero
        t0 = min(self.T[0], self.TTWIN[0])  # First time in rostime
        T = [t-t0 for t in self.T]          # Time vector for telerobot
        TTWIN = [t-t0 for t in self.TTWIN]  # Time vector for virtual vehicle aka. the twin

        # Steering control signals
        ax5.step(TTWIN, self.TSTEER, 'g', label = r"$\delta_V$", where='post')
        ax5.step(T, self.STEERING, 'b', label = r"$\delta_T$", where='post')
        ax5.set_title("Steering")
        ax5.set_xlabel("Time [s]")
        ax5.set_ylabel('Angle [rad]')
        ax5.legend(ncol =2)
        ax5.grid()
        
        # Velocities
        ax4.plot(T, self.V,'b', label=r'$v_T$')
        ax4.plot(TTWIN, self.VTWIN,'g', label=r'$v_V$')
        ax4.plot(T, self.VHI,'r', label=r'$v_\mathrm{max}$')
        ax4.plot(TTWIN, self.VCMD,'black', label = r'$v_r$')
        ax4.set_xlim([0, T[-1]+4])
        ax4.set_ylim([-0.25, 1.2])
        ax4.legend(loc = 4, ncol = 2)
        ax4.set_title("Velocities")
        ax4.set_ylabel("Velocity [m/s]")
        ax4.grid()

        # Paths
        planned_path = self.planned_path.copy()
        lead_path = self.lead_path_handler.get_path()
        lead_x, lead_y = lead_path.get_xy()
        plan_x, plan_y = planned_path.get_xy()
        
        dirname = os.path.dirname(__file__)
        distmapfile = str(dirname) + '/distmap.csv'
        binmapfile = str(dirname) + '/binmap.csv'
        self.map_handler = MapHandler(distmapfile=distmapfile, binmapfile=binmapfile)

        left = self.map_handler.origin_x
        right = self.map_handler.origin_x + self.map_handler.w
        up = self.map_handler.origin_y
        down = self.map_handler.origin_y + self.map_handler.h

        ax3.imshow(self.map_handler.binmap, cmap='Greys', extent = (left, right, up, down), origin = 'lower')
        ax3.plot(lead_x, lead_y,'g', label = "VV")
        ax3.plot(self.X, self.Y,'b', label = "TR")
        ax3.plot(plan_x, plan_y, c = 'r', label = "Planned", zorder = 10)

        ax31.imshow(self.map_handler.binmap, cmap='Greys', extent = (left, right, up, down), origin = 'lower')
        ax31.plot(lead_x, lead_y,'g', label = "VV")
        ax31.plot(self.X, self.Y,'b',label = "TR")
        ax31.plot(plan_x, plan_y, c = 'r',label = "Planned")
        ax31.axis("equal")
        ax31.set_xlabel("x [m]")
        ax31.set_ylabel("y [m]")

        ax32.imshow(self.map_handler.binmap, cmap='Greys', extent = (left, right, up, down), origin = 'lower')
        ax32.plot(lead_x, lead_y,'g', label = "VV")
        ax32.plot(self.X, self.Y,'b',label = "TR")
        ax32.plot(plan_x, plan_y, c = 'r',label = "Planned")
        ax32.axis("equal")
        ax32.set_xlabel("x [m]")
        ax32.set_ylabel("y [m]")

        # Uncomment to plot all planned paths
        '''
        for path in self.PATHS:
            x,y = path.get_xy()
            ax3.plot(x, y, 'purple', zorder = 5)
        '''
        
        ax3.scatter(lead_x[0], lead_y[0], c = 'r', label = "Start", s = 75)
        ax3.legend(loc = 4, scatterpoints=1)
        ax3.set_xlabel("x [m]")
        ax3.set_ylabel("y [m]")
        ax3.axis("equal")
       
        # Print computation time info
        avg_tmpc = np.mean(self.TMPC)
        std_tmpc = np.std(self.TMPC)
        print("svg and std_mpc time", avg_tmpc, std_tmpc)


    def _update_twin_path(self, path_segment_msg):
        """Callback function to handle PathSegment messages from the virtual vehicle."""
        path_x = path_segment_msg.x
        path_y = path_segment_msg.y
        path_v = path_segment_msg.v
        path_yaw = path_segment_msg.yaw
        path_time = path_segment_msg.timestamp
        for i in range(len(path_x)):
            rospy.sleep(0.01)
            self.lead_path_handler.add_waypoint(path_x[i], path_y[i], path_yaw[i], path_v[i], path_time[i])
        
        # Set self.go = True after first message has been received from virtual vehicle
        if not self.go:
            self.t0 = rospy.get_time()
            self.go = True

    def append_past_path(self, x,y,yaw):
        """Append pose information to visualize the path taken in RVIZ."""
        self.past_path_x.append(x)
        self.past_path_y.append(y)
        self.past_path_yaw.append(yaw)

    def _update_planned_path(self, path_segment_msg):
        """Callback function to handle PathSegment messages from the path planner"""
        path_x = path_segment_msg.x
        path_y = path_segment_msg.y
        path = np.column_stack((path_x, path_y))
        self.planned_path = PathObject(path = path)
    
    def _update_twin_ctrl(self, ctrl_msg):
        """Callback function to handle log information sent from the virtual vehicle"""
        data = ctrl_msg.data
        # Append to lists for logging and plotting
        self.TTWIN.append(data[0])
        self.VCMD.append(data[1])
        self.UTWIN.append(data[2])
        self.VDESTWIN.append(data[3])
        self.VTWIN.append(data[4])
        self.TSTEER.append(data[5])


    def _start_listen(self):
        """Subscribe to the topics \\twin_path, \\planned_path_segment, and \\twin_ctrl."""
        print("Starting subscribers")

        # Waypoints of the virtual vehicle
        self.twin_path_sub = rospy.Subscriber("/twin_path", PathSegment, self._update_twin_path)
        
        # Complete planned path as a sequence of points
        self.planned_path_sub =rospy.Subscriber("/planned_path_segment", PathSegment, self._update_planned_path)
        
        # Additional information of virtual vehicle for logging and plotting
        self.twin_ctrl_sub = rospy.Subscriber("/twin_ctrl", Float64MultiArray, self._update_twin_ctrl)
        
    def start(self):
        """Start the subscribers and the main loop"""
        self._start_listen()
        # Wait for leader
        while not self.go:
            print("Waiting for leader")
            rospy.sleep(0.1)

        self._follow()

    def _get_current_state(self):
        """Return the current state of the vehicle model"""
        return self.model.x, self.model.y, self.model.yaw, self.model.v
        

    def _lat_dist(self, pos0, yaw0, pos1):
        """Return the transversal (lateral) distance between two points with regards to the orientation at the first point. 
        Positive if pos1 to the right of pos0.

        :param pos0: (x,y) Coordinates of the first point
        :type pos0: ndarray
        :param yaw0: Yaw angle at pos0
        :type yaw0: float
        :param pos0: (x,y) Coordinates of the second point
        :type pos0: ndarray
        :return: Transversal distance
        :rtype: float
        """
        # Perpendicular unit vector
        t = np.array([np.cos(yaw0+np.pi/2), np.sin(yaw0+np.pi/2)])
        
        # Separation vector
        d = pos1-pos0
        return np.dot(t,d)

    def _long_dist(self, pos0, yaw0, pos1):
        """Return the longitudinal distance between two points with regards to the orientation at the first point. 
        Positive if pos1 ahead of pos0.

        :param pos0: (x,y) Coordinates of the first point
        :type pos0: ndarray
        :param yaw0: Yaw angle at pos0
        :type yaw0: float
        :param pos0: (x,y) Coordinates of the second point
        :type pos0: ndarray
        :return: Longitudinal distance
        :rtype: float
        """
        # Tangent unit vector
        t = np.array([np.cos(yaw0), np.sin(yaw0)])

        # Separation vector
        d = pos1-pos0
        return np.dot(t,d)

    def _update_and_publish_v_limits(self, u, v):
        """Predicts the steady state velocity (vmax = v/u*umax) at maximum velocity command by estimating longitudinal dynamics with a first order model
        and updates the estimated maximum velocity v_lim with a low-pass filter. Only making predictions if u/u_max>u_thres. 
        Also publishing v_lim for the virtual vehicle to observe.

        :param u: Current velocity command
        :type u: float
        :param v: Current velocity
        :type v: float
        """

        
        if abs(u/self.u_max) > self.delta_u:
            # Predict and update self.v_lim with low-pass filter
            self.v_lim = self.v_alpha*abs(v*self.u_max/u)+(1-self.v_alpha)*self.v_lim
        else:
            # Apply default velocity as estimated maximum
            self.v_lim = self.v_alpha*abs(self.v0)+(1-self.v_alpha)*self.v_lim

        # Create message and publish
        v_lim_msg = Float64MultiArray()
        v_lim_msg.data = [self.v_lim, -self.v_lim]
        self.v_lim_pub.publish(v_lim_msg)

    def estimate_derivative(self, x, x_prev, x_dot_prev):
        """Estimate a derivative using backwards difference and a low-pass filter

        :param x: Current value
        :type x: float
        :param x_prev: Value from previous time step
        :type x_prev: float
        :param x_dot_prev: Derivative in previous time step
        :type x_dot_prev: float
        :return: Estimated current derivative
        :rtype: float
        """
        return self.der_alpha*(x-x_prev)/self.dt+(1-self.der_alpha)*x_dot_prev

    def _follow(self):
        """Main loop for motion control of the telerobot."""
        print("Following")

        # Initialization
        u = 0.0
        steering = 0.0
        time = 0.0
        v_des = 0
        ey_prev = None
        eyaw_prev = None
        ey_dot = 0.0
        eyaw_dot = 0.0
        yaw_dot = 0.0
        yaw_prev = 0.0
        closest_idx = 0
        current_path = None
        spacing = 0
        s_ref = 0
        es = 0
        e0 = [0,0,0,0]
        curv = 0
        mpc_time = 0
        tot_time = 0

        while not rospy.is_shutdown() and time < self.tmax:
            
            if not self.is_eval:
                print("Time: " + str(time))
            tot_start = default_timer()     # Timer
            rtime = rospy.get_time()        # Record timestamp for logging

            # Observe state, ego vehicle = telerobot
            ego_x, ego_y, ego_yaw, ego_v = self._get_current_state()    
            ego_pos = np.array([ego_x, ego_y])
            target_pos = ego_pos.copy()
            
            # Get virtual vehicle's path and state
            lead_path = self.lead_path_handler.get_path()
            v_twin = lead_path.get_vel()
            
            if self.planned_path.N < 3:
                # No path has yet been planned, don't drive
                self.ctrl_interface.send_control(0,0)
                tot_end = default_timer()
            else:
                # Execute motion according to planned path

                # Observe currently planned path and record it for the log
                current_path = self.planned_path.copy()
                self.PATHS.append(current_path)
 
                # update v limits to communicate to twin to limit divergence
                self._update_and_publish_v_limits(u, ego_v)

                ####### LONGITUDINAL CONTROL #######
                # Determine speed command with PID, using spacing and reference spacing

                s_ref = max(0,self.th*ego_v)   #Reference spacing

                # Target pose and index of the corresponding point on VV's (aka. virtual vehicle's or twin's) path
                twin_target_pose, target_idx = lead_path.get_pose(idx = -1, dist_offset = -s_ref) 

                # Point on planned path closest to the target point, within +-10 steps to allow paths to cross
                target_idx = current_path.get_closest_idx(twin_target_pose[0], twin_target_pose[1], idx_start = closest_idx-10, idx_stop = closest_idx + 10)
                
                # Target pose and position on the planned path
                target_pose,_ = current_path.get_pose(idx = target_idx, dist_offset = 0)
                target_pos = target_pose[0:2]

                # Index of the point on the planned path closest to telerobot
                closest_idx = current_path.get_closest_idx(ego_x, ego_y, idx_start = closest_idx, idx_stop = closest_idx + 10)
                
                # Length of the planned path segment between telerobot and end
                path_dist = np.sum(current_path.get_ds()[closest_idx:])
                
                # Spacing error = longitudinal distance between telerobot and the target point
                es = self._long_dist(ego_pos, ego_yaw, target_pos)
                
                # Total spacing
                spacing = es+s_ref
                
                # Force telerobot to stop when telerobot is close to the end of the planned path
                self.stop = path_dist < 0.2
                if not self.stop:
                    # Determine velocity command with PID
                    u = -self.pid.get_ctrl(0.0, es)
                    u = sat(u, self.u_max)  # Saturation
                else:
                    # Must stop
                    u = 0.0
                    self.pid.reset() # Avoid windup
                    

                ####### LATERAL CONTROL #######
                # Determine steering by MPC

                # Closest point on the planned path
                closest_pose = current_path.get_pose(closest_idx)
                closest_pos = closest_pose[0:2]
                closest_yaw = closest_pose[2]
                
                # Observe the path's curvature in the next time step
                _,next_idx = current_path.get_pose(idx = closest_idx, dist_offset = ego_v*self.dt)
                curv = -current_path.get_curv(idx = next_idx)

                # Lateral distance error
                ey = -self._lat_dist(ego_pos, ego_yaw, closest_pos)

                # Yaw error in relation to path tangent
                eyaw = (ego_yaw - closest_yaw)

                # Normalize
                while eyaw > np.pi:
                    eyaw -= 2*np.pi
                while eyaw < -np.pi:
                    eyaw += 2*np.pi

                if ey_prev is None or eyaw_prev is None:
                    # Initial previous values equal current values
                    ey_prev = ey
                    eyaw_prev = eyaw
                    yaw_prev = ego_yaw

                # Estimate derivatives with low pass filter
                ey_dot = self.estimate_derivative(ey, ey_prev, ey_dot)
                eyaw_dot = self.estimate_derivative(eyaw, eyaw_prev, eyaw_dot)
                yaw_dot = self.estimate_derivative(ego_yaw, yaw_prev, yaw_dot)

                # MPC errors
                e1 = ey
                e1_dot = ey_dot + ego_v*eyaw
                e0 = [e1, e1_dot, eyaw, eyaw_dot]

                # Save for next iteration
                ey_prev = ey
                eyaw_prev = eyaw
                yaw_prev = ego_yaw

                # Compute steering with mpc optimization
                mpc_start = default_timer()
                steering = self.mpc.get_ctrl(e0, ego_v, curv, steering)
                mpc_end = default_timer()
                mpc_time = mpc_end-mpc_start

            # Logging and publishing for visualization
            self.append_past_path(ego_x, ego_y, ego_yaw)
            self.v_pub.publish(ego_v)
            publish_target(self.target_pub, target_pos[0], target_pos[1])
            publish_path(self.past_path_pub, self.past_path_x, self.past_path_y)
            publish_3Dcar(self.car_poly_pub, self.pose_pub,self.model.x, self.model.y, self.model.yaw)
            tot_end = default_timer()
            tot_time = tot_end-tot_start
            self.YAWDOT.append(yaw_dot)
            self._append_log(ego_x, ego_y, ego_yaw, ego_v, v_twin, v_des, spacing, s_ref, 
                es, u, rtime, target_pos[0], target_pos[1], self.v_lim, -self.v_lim, e0, curv, steering, mpc_time, tot_time)
            
            if not self.is_eval:
                time += self.dt
            self.rate.sleep()
            
            # Send control
            self.ctrl_interface.send_control(steering, u)

        # Draw plots
        #self._make_long_plot()
        #self._make_lat_plot()
        self._make_complot()
        plt.show()

def init():
    """Initialization to read arguments from launch file
    """
    
    start_pt_param = rospy.search_param('start_pt')
    init_pt = [0.0, 0.0, 0.0, 0.0]
    eval_param = rospy.search_param('is_eval')

    # Indicates if it is an evaluation trial
    is_eval = rospy.get_param(eval_param, "false")

    start_pt = rospy.get_param(start_pt_param, init_pt)
    if type(start_pt) == type(''):
        start_pt = start_pt.split(',')
        start_pt = [float(curr) for curr in start_pt]

    return start_pt, is_eval

def main():
    """Start rosnode, get launch file arguments, initialize model and simulator and start the telerobot main loop.
    """
    vehicle_name = "SVEA_telerobot"
    rospy.init_node('telerobot_module')
    print("starting TRM node")
    start_pt, is_eval = init()   # Read arguments from launch file
    
    # initialize simulated model and control interface
    bicycle_state = VehicleState(*start_pt)
    bicycle_model = SimpleBicycleModel(bicycle_state)
    trm = TelerobotModule(vehicle_name, bicycle_state, is_eval)

    # start background simulation thread
    simulator = SimSVEA(vehicle_name, bicycle_model, 1.0/50.0)
    simulator.start()
    print("starting TRM")
    trm.start()


if __name__ == "__main__":
    main()



