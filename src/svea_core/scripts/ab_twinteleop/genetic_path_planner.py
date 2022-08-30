#!/usr/bin/env python

from path_handler import PathHandler, MapHandler, PathObject, BezierPath
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
from nav_msgs.msg import Path
import numpy as np
import os
import sys
import rospy
from svea.simulators.viz_utils import publish_path
from svea.msg import PathSegment
import tf

class GeneticPathPlanner:
    """Class for performing continuous local path planning with a genetic algorithm.

    :param map_handler: A MapHandler object containing information about the location of obstacles and and the distance to them.
    :type map_handle: MapHandler
    :param plan_rate: Sampling frequency of the planning loop in Hz, defaults to 10.0
    :type plan_rate: float, optional
    """

    def __init__(self, map_handler, plan_rate = 10.0):
        """Constructor method.
        """
        
        self.map = map_handler                              # Contains obstacle distance information
        self.twin_path_handler = PathHandler(maxlen=None)   # Handles the path taken by the virtual vehicle aka. twin
        self.planned_path = PathObject()                    # The currently planned path
        self.ego_x = None                                   # Telerobot state x
        self.ego_y = None                                   # Telerobot state y
        self.ego_yaw = None                                 # Telerobot state yaw
        self.ego_v = None                                   # Telerobot state velocity
        self.path_ds = 0.05                                 # Desired spacing between points in the sampled planed path (meters)
        self.plan_rate = plan_rate                          # Planning frequency
        
        ############# Parameters for genetic algorithm ################
        self.max_iter = 20          # Maximum number of iterations
        self.rho_safe = 0.2         # Minimum allowed obstacle distance
        self.curv_max = 2.5         # Maximum allowed path curvature (curvature = 1/(radius of curvature))            
        self.delta_F = 20           # Fitness update threshold. A change in best fitness less than self.delta_F indicates convergence
        self.n_break = 3            # Number of iterations with fitness update < self.delta_F required for the genetic algorithm to terminate.
        self.p_mut = 0.4            # Probability for a path candidate to mutate
        self.f_max = 100            # Maximum fitness
        self.pop_size = 20          # Population size
        self.alpha0 = 0.2           # Initial alpha as a fraction of optimal path length
        self.beta0 = 0.2            # Initial beta as a fraction of optimal path length
        self.tau0 = 0.5             # Time-headway for telerobot (path planning occurs beyond that point)
        self.tau2 = 1.0             # Time-headway for virtual vehicle defining the length of path prediction
        self.kd = 0.5               # Fraction of distance along the virtual vehicle's path to put the initial intermediate pose 
        
        # Maximum standard deviation in the random mutation resampling expressed as a fraction of optimal path length.
        self.lambda1 = 0.1          # Intermediate pose 
        self.lambda2 = 0.1          # End pose
        self.lambda_ab = 0.05       # Alpha and beta 

        # Parameter for decay rate as fitness increases in the random mutation resampling
        self.k1 = 3.0               # Intermediate pose
        self.k2 = 3.0               # End pose
        self.kab = 3.0              # Alpha and beta

        # Weights in the fitness function
        self.r_L = 100              # Weight on path length
        self.r_d = 500              # Weight on terminal offset
        self.r_in_obs = 1000        # Weight on path segment inside obstacle
        self.r_avg_obs = 0          # Weight on average obstacle distance
        self.r_too_curved = 100     # Weight on infeasible curvature
        self.r_avg_curv = 0         # Weight on average curvature
        
        # Variances for creating the inital population. Expressed as squared standard deviations in meters and radians
        sigma_init_w1 = (0.25**2,0.25**2,0.05**2)   # Variances for (x1, y1, theta1)
        sigma_init_w2 = (0.1**2,0.1**2,0.05**2)     # Variances for (x2, y2, theta2)
        sigma_init_alpha = (0.01**2,0.01**2)        # Variances for (alpha, beta)
        self.sigma_init = np.diag((sigma_init_w1+sigma_init_w2+sigma_init_alpha))   # Covariance matrix

        # Publishers
        self.path_seg_pub = rospy.Publisher("/planned_path_segment", PathSegment, queue_size=1) # Publisher for path-tracking
        self.path_pub = rospy.Publisher("/planned_path", Path, queue_size=1)                    # Publisher for path-visualization

        # Flags to indicate data is being published on subscribers
        self.twin_path_ready = False
        self.ego_pose_ready = False
        self.twin_pose_ready = False
        self.ego_v_ready = False


    def start_listen(self):
        """Start subscribers.
        """
        rospy.Subscriber("/twin_path", PathSegment, self._update_twin_path)
        rospy.Subscriber("/robot_pose_sim", PoseStamped, self._update_ego_pose)
        rospy.Subscriber("/robot_pose_twin", PoseStamped, self._update_twin_pose)
        rospy.Subscriber("/svea_v", Float64, self._update_ego_v)
        self.rate = rospy.Rate(self.plan_rate) # Rate object. Placed here to allow testing without starting ROS.


    def _update_twin_path(self, path_segment_msg):
        """Callback function to update virtual vehicle's path.
        """
        path_x = path_segment_msg.x
        path_y = path_segment_msg.y
        path_v = path_segment_msg.v
        path_yaw = path_segment_msg.yaw
        path_time = path_segment_msg.timestamp
        for i in range(len(path_x)):
            self.twin_path_handler.add_waypoint(path_x[i], path_y[i], path_yaw[i], path_v[i], path_time[i])
        self.twin_path_ready = True
        self.twin_v = path_v[-1]


    def _update_twin_pose(self, pose_msg):
        """Callback function to update the pose of the virtual vehicle.
        """
        self.twin_x = pose_msg.pose.position.x
        self.twin_y = pose_msg.pose.position.y
        q1 = pose_msg.pose.orientation.x
        q2 = pose_msg.pose.orientation.y
        q3 = pose_msg.pose.orientation.z
        q4 = pose_msg.pose.orientation.w
        orientation = tf.transformations.euler_from_quaternion((q1,q2,q3,q4))
        self.twin_yaw = orientation[2]
        self.twin_pose_ready = True

    def _update_ego_pose(self, pose_msg):
        """Callback function to update the pose of the telerobot.
        """
        self.ego_x = pose_msg.pose.position.x
        self.ego_y = pose_msg.pose.position.y
        q1 = pose_msg.pose.orientation.x
        q2 = pose_msg.pose.orientation.y
        q3 = pose_msg.pose.orientation.z
        q4 = pose_msg.pose.orientation.w
        orientation = tf.transformations.euler_from_quaternion((q1,q2,q3,q4))
        self.ego_yaw = orientation[2]
        self.ego_pose_ready = True

    def _update_ego_v(self, v_msg):
        """Callback function to update the velocity of the telerobot.
        """
        self.ego_v = v_msg.data
        self.ego_v_ready = True

        
    def plan_loop(self):
        """Main loop for continuous local path planning."""

        print("Starting plan loop")
        z0_idx = 0
        closest_idx = 0

        # Wait until data is being published to topics
        while not (self.twin_path_ready and self.ego_pose_ready and self.twin_pose_ready and self.ego_v_ready) and not rospy.is_shutdown():
            print("Planner waiting for pose and path")
            rospy.sleep(0.1)

        while not rospy.is_shutdown():
            # Observe virtual vehicle's path and pose and current pose of telerobot
            twin_path = self.twin_path_handler.get_path()
            twin_pose = [self.twin_x, self.twin_y, self.twin_yaw]   
            ego_pose = np.array([self.ego_x, self.ego_y, self.ego_yaw])
            
            # Determine initial conditions of the path to be planned
            # i.e. start pose, initial derivatives (wd and wdd) and the segment of previously planned path to keep (= fixed_path)
            if self.planned_path.N == 0:
                # If no path has been planned, start_pose at current pose
                wd = np.array([np.cos(ego_pose[2]), np.sin(ego_pose[2])])
                wdd = np.array([0, 0])
                start_pose = np.copy(ego_pose)
                fixed_path = self.planned_path.copy()
            else:
                # Find the startpoint on the previously planned path a distance d0 ahead of the current position
                d0 = max(0,self.ego_v)*self.tau0

                # Index of the point closest to telerobot on the planned path
                closest_idx = self.planned_path.get_closest_idx(ego_pose[0], ego_pose[1], idx_start = closest_idx, idx_stop = closest_idx+10)
                
                # Index indicatinf where the previously planned path is split into a fixed segment to keep and a second segment that is disregarded
                _, split_idx = self.planned_path.get_pose(idx=closest_idx, dist_offset=d0)  
                
                # Start pose and derivatives at the end of the fixed segment
                start_pose, _ = self.planned_path.get_pose(idx=split_idx, dist_offset=0)  
                wd, wdd = self.planned_path.get_derivatives(idx = split_idx)
                fixed_path,_ = self.planned_path.split_at_idx(split_idx)

            
            # Find the initial end pose with path prediction
            curv = -twin_path.get_curvs(idx=-1) # Endpoint curvature of virtual vehicle's path
            d2 = self.tau2*self.twin_v          # Length of path to predict
            if abs(curv) > 0.1:
                # Predict path
                yaw_next = twin_pose[2] + d2*curv
                x_next = twin_pose[0] + (np.sin(twin_pose[2]+d2*curv)-np.sin(twin_pose[2]))/curv
                y_next = twin_pose[1] - (np.cos(twin_pose[2]+d2*curv)-np.cos(twin_pose[2]))/curv
                end_pose = np.array([x_next, y_next, yaw_next])
            else:
                # Assume no curvature
                end_pose = np.array([twin_pose[0] + d2*np.cos(twin_pose[2]), twin_pose[1] + d2*np.sin(twin_pose[2]), twin_pose[2]])

            # Index of start pose
            z0_idx = twin_path.get_closest_idx(start_pose[0], start_pose[1], idx_start = z0_idx-100, idx_stop = z0_idx+100)
            
            # Distance between start pose and end pose
            d02 = np.sum(twin_path.get_ds()[z0_idx:]) + d2

            # Separation between end pose and intermediate (mid) pose, which defines the mid pose
            d1 = max(0, self.kd*d02 - d2)
            mid_pose,_ = twin_path.get_pose(idx=-1, dist_offset=-d1)
            
            # Plan path with genetic algorithm
            new_path = self.plan_path(start_pose, mid_pose, end_pose, wd, wdd)

            if new_path is not None:
                # Planner was successful. Append new path to fixed_path and publish
                if fixed_path.N > 0:
                    self.planned_path = fixed_path.extend_path(new_path, start_idx = 1) # Join new path with fixed path
                else:
                    self.planned_path = new_path.copy()
                # Create PathSegment message
                x,y = self.planned_path.get_xy()
                yaw = self.planned_path.get_yaw()
                path_seg_msg = PathSegment()
                path_seg_msg.x = x.tolist()
                path_seg_msg.y = y.tolist()
                path_seg_msg.yaw = yaw.tolist()
                self.path_seg_pub.publish(path_seg_msg)             # publish for tracking
                publish_path(self.path_pub, x.tolist(), y.tolist()) # publish for visualization
            
            # Sleep if successful
            if not new_path is None:
                self.rate.sleep()


    def plan_path(self, start_pose, mid_pose, end_pose, wd, wdd):
        """Perform local path planning with genetic algorithm.

        :param start_pose: Required start pose of the path to be planned (x,y,yaw)
        :type start_pose: ndarray
        :param mid_pose: Initial guess of intermediate pose for the planned path, (x,y,yaw)
        :type mid_pose: ndarray
        :param end_pose: Desired end pose of the planned path, (x,y,yaw)
        :type end_pose: ndarray
        :param wd: Required first order derivative at the start point of the planned path, (xd, yd) 
        :type wd: ndarray
        :param wdd: Required second order derivative at the start point of the planned path, (xdd, ydd) 
        :type wdd: ndarray
        :return: A path as a sequence of points if found or None otherwise
        :rtype: list of ndarray or None
        """


        start_pose = np.array(start_pose)
        mid_pose = np.array(mid_pose)
        end_pose = np.array(end_pose)
        n_good = 0              # Current number of consecutive good best scores
        i = 0                   # Iteration number
        abs_best_path = None    # All-time best path
        abs_best_fitness = 0    # All-time best fitness
        best_fitness_prev = 0   # Best fitness of the population in previous iteration


        # Optimal path length of a straight path connecting start point and end point
        self.L_min = np.sqrt((end_pose[0] - start_pose[0])**2 +(end_pose[1] - start_pose[1])**2)

        if np.linalg.norm(start_pose[0:2]-end_pose[0:2]) < 0.5:
            # Too small distance to plan
            return None

        # Create initial population
        current_pop = self.create_initial_pop(start_pose, mid_pose, end_pose, wd, wdd, num = self.pop_size)
        
        while not rospy.is_shutdown() and i < self.max_iter:
        
            # Evaluate current population and identify the best candidate
            fitness, feasible = self.evaluate(current_pop, end_pose)
            best_fitness = np.max(fitness)
            best_idx = np.argmax(fitness)

            if best_fitness > abs_best_fitness and feasible[best_idx]:
                # Save the best (feasible) solution
                abs_best_path = current_pop[best_idx].copy()
                abs_best_fitness = best_fitness
            
            # Determine number of consequtive good scores with small updates
            if abs(best_fitness-best_fitness_prev) < self.delta_F and feasible[best_idx]:
                n_good += 1
            else:
                n_good = 0

            best_fitness_prev = best_fitness
            
            # Break if self.n_break good consequtive scores has been recorded
            if n_good > self.n_break:
                # Determine the number of points to sample and return the sampled path
                L = abs_best_path.get_length()
                N = int(np.round(L/self.path_ds+1))
                return abs_best_path.get_bezier_path(num = N)
                
            # Population updated through Selection and Mutation
            current_pop = self.select(current_pop, fitness, len(current_pop))
            current_pop = self.mutate(current_pop, self.p_mut, fitness=fitness)
            i += 1

        if abs_best_path is None:
            # No feasible path was found --> Planner failed
            return None
        else:
            # Return the all-time best path with desired sampling frequency
            L = abs_best_path.get_length()
            N = int(np.round(L/self.path_ds+1))
            return abs_best_path.get_bezier_path(num = N)
            
        
    def evaluate(self, pop, end_pose):
        """Evaluates each path candidate of a population and returns a list of fitness values. 

        :param pop: List of path candidates corresponding to the Population
        :type pop: list of BezierPath
        :param end_pose: Desired end pose (x,y,yaw)
        :type end_pose: ndarray
        :return: Fitness values for each path candidate
        :rtype: list of float
        """
        # Lists holding the fitness values and booleans for indicating feasibility
        fitness = []
        feasible = []
        for bpath in pop:
            
            # Generate a low resolution path
            path = bpath.get_bezier_path(num = 20, quick=True)
            
            # Compute fitness and determine feasibility
            f, feas = self.get_fitness(path, end_pose)
            feasible.append(feas)
            fitness.append(f)
 
        return fitness, feasible

    def get_fitness(self, path, goal_pose):
        """Computes the fitness value of one path.

        :param path: Path represented by a sequency of points.
        :type path: list of ndarray
        :param goal_pose: Desired termination pose of the planned path, (x,y,yaw) 
        :type goal_pose: ndarray 
        """

        # Path candidate information
        ds = path.get_ds()                                  # Spacing between the smapled points on the path
        (x,y) = path.get_xy()                               # (x,y)-cooridantes of the points
        obs_dists = self.map.get_obstacle_distance(x, y)    # Distance to obstacles at each point
        curvs = np.abs(path.get_curvs())                    # Path curvature at each point
        max_curv = np.max(curvs)                            # Maximum curvature
    
        # Compute scores as performance indicators with weights
        s_L = self.r_L*abs(path.L-self.L_min)**2                                # Length score
        s_d = self.r_d*np.linalg.norm(path.get_pose(-1)-goal_pose)              # Terminal offset score
        s_in_obs = self.r_in_obs*np.sum(ds[obs_dists < self.rho_safe])          # Path inside obstacles score
        s_avg_obs = self.r_avg_obs*1/(1+np.mean(obs_dists))                     # Obstacle proximity score
        s_too_curved = self.r_too_curved*(max_curv > self.curv_max)*max_curv    # Infeasible curvature score
        s_avg_curv = self.r_avg_curv*max_curv                                   # Average curvature score

        # Sum of scores
        s = (1+s_L+s_d+s_in_obs+s_avg_obs+s_too_curved+s_avg_curv)

        # Fitness value
        f = self.f_max/s

        # Determine feasibility
        feasible = (s_too_curved == 0) and (s_in_obs == 0)

        return f, feasible
        

    def select(self, pop, fitness, num):
        """Random resampling of the population with inspiration of natural selection to promote candidates with higher fitness.

        :param pop: List of path candidates corresponding to the Population
        :type pop: list of BezierPath
        :param fitness: Fitness values for each path candidate in the population
        :type fitness: list of float
        :param num: Number of candidates in the new population
        :type num: int
        :return: A new list of path candidates i.e. a new population
        :rtype: list of BezierPath
        """

        # Normalize and exxagerate fitness differences
        favg = np.mean(fitness)
        fmod = (fitness/favg)**4
        fmod_sum = np.sum(fmod)

        # Probability for each candidate to be selected
        prob = fmod/fmod_sum
        
        # Resampling
        selected = np.random.choice(num, num, p=prob)
        return [pop[i].copy() for i in selected]


    def mutate(self, pop, prob, fitness):
        """Applies random to the candidates of the population by randomly resampling the mid end end poses of the candidates.

        :param pop: List of path candidates corresponding to the Population
        :type pop: list of BezierPath
        :param prob: Probability that a candidate mutates
        :type prob: float
        :param fitness: The fitness values for each candidate. Higher fitness --> smaller mutation
        :type fitness: list of float
        :return: A new list of path candidates i.e. a new population
        :rtype: list of BezierPath
        """

        mutated_pop = []
        i = 0
        for bpath in pop:
            if np.random.rand() <= prob:
                f = fitness[i]

                # Create the covariance matrix for resampling
                sig1 = (self.lambda1*self.L_min)**2
                sigma_pos1 = sig1/(1+(self.k1*f)**2)                # Variance for mid position
                sig2 = (self.lambda2*self.L_min)**2
                sigma_pos2 = sig2/(1+(self.k2*f)**2)                # Variance for end position
                sigma_yaw1 = 0.1*sigma_pos1                         # Variance for mid yaw
                sigma_yaw2 = 0.1*sigma_pos2                         # Variance for end yaw
                sigma_mut_w1 = (sigma_pos1,sigma_pos1,sigma_yaw1)
                sigma_mut_w2 = (sigma_pos2,sigma_pos2,sigma_yaw2)
                sig3 = (self.lambda_ab*self.L_min)**2               
                sigma_alphabeta = sig3/(1+(self.kab*f)**2)          # Variance for alpha and beta

                # Covariance matrix
                sigma_mut = np.diag((sigma_mut_w1+sigma_mut_w2+(sigma_alphabeta,sigma_alphabeta)))

                # Previous poses and alpha, beta used as means in Gaussian distribution
                w1_old = bpath.mid_pose     # Mid pose
                w2_old = bpath.end_pose     # End_pose
                alpha_old = bpath.alpha
                beta_old = bpath.beta

                # Mean
                mu = list(w1_old) + list(w2_old) + [alpha_old] + [beta_old]

                # Resampling from Gaussian distribution with means and covariance from above
                rand_sample = np.random.multivariate_normal(mu, sigma_mut)

                # Extract poses and alpha, beta
                w1 = rand_sample[0:3]   # Mid pose
                w2 = rand_sample[3:6]   # End_pose
                alpha = rand_sample[6]
                beta = rand_sample[7]

                # Apply the new poses and alpha, beta
                bpath.set_end_pose(w2[0], w2[1], w2[2])
                bpath.set_mid_pose(w1[0], w1[1], yaw = w1[2])
                # bpath.edit_endpoint(w2)
                # bpath.edit_waypoint(0, w1[0], w1[1], new_yaw = w1[2])
                bpath.alpha = alpha
                bpath.beta = beta
            
            mutated_pop.append(bpath)
            i += 1 
        return mutated_pop


    def create_initial_pop(self, start_pose, mid_pose, end_pose, wd, wdd, num = 5):
        """Generate an initial population.

        :param start_pose: Required start pose of all candidates, (x,y,yaw)
        :type start_pose: ndarray
        :param mid_pose: Initial mid pose quess, (x,y,yaw)
        :type mid_pose: ndarray
        :param end_pose: Desired termination pose, (x,y,yaw)
        :type end_pose: ndarray
        :param wd: Required first order derivative at the start point, (xd,yd)
        :type wd: ndarray
        :param wdd: Required second order derivative at the start point, (xdd,ydd)
        :type wdd: ndarray
        :param num: Number of path candidates in the population, defaults to 5
        :type num: int, optional
        :return: List of path candidates corresponding to the population
        :rtype: list of BezierPath
        """

        # The population
        pop = []        
 
        # Compute alpha and beta
        L1 = np.linalg.norm(mid_pose[0:2]-start_pose[0:2])
        L2 = np.linalg.norm(end_pose[0:2]-mid_pose[0:2])
        alpha = self.alpha0*L1
        beta = self.beta0*L2

        # Create the first path candidate without randomization
        simple_path = BezierPath(start_pose=start_pose, end_pose=end_pose, alpha = alpha, beta = beta)
        simple_path.set_mid_pose(mid_pose[0], mid_pose[1], yaw = mid_pose[2])
        pop.append(simple_path)

        # Given poses and alpha, beta are means in Gaussian distributions
        mu = list(mid_pose) + list(end_pose) + [alpha] + [beta]

        # Generate the other candidates as random samples from Gaussian distribution
        for i in range(num-1):
            # Random sampling
            rand_sample = np.random.multivariate_normal(mu, self.sigma_init)
            
            # Extract poses and alpha, beta
            w1 = rand_sample[0:3]   # Mid pose
            w2 = rand_sample[3:6]   # End pose
            alphar = rand_sample[6]
            betar = rand_sample[7]

            # Create the BezierPath object and append to population
            bpath = BezierPath(start_pose=start_pose, end_pose=w2, wd = wd, wdd = wdd,  alpha = alphar, beta = betar)
            bpath.set_mid_pose(w1[0], w1[1], yaw = w1[2])
            pop.append(bpath)

        return pop


def main():
    """Start the path planner node, load maps, initialize the genetic path planner and start the main loop.
    """
    rospy.init_node('path_planner')
    print("Starting path planner node")
    dirname = os.path.dirname(__file__)
    mh = MapHandler(distmapfile=str(dirname) +'/distmap.csv', binmapfile=str(dirname)+'/binmap.csv')
    gpp = GeneticPathPlanner(map_handler=mh)
    gpp.start_listen()
    gpp.plan_loop()

if __name__ == "__main__":
    main()