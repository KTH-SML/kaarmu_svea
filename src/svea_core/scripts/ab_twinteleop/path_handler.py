"""Classes for handling maps, Bezier path representations and general paths,
"""

from matplotlib import pyplot as plt
import numpy as np
from svea.msg import PathSegment
import copy
from ab_bezier_path import calc_4points_bezier_path, calc_bezier_path_and_derivatives


class MapHandler:
    """A class for handling gridmaps for path planning.
    Two types of maps. 1. Binary gridmap (binmap), where 1 indicates that the cell is occupied by an obstacle and 0 means it is not occupied.
    2. Distance gridmap (distmap), where each cell contains a numerical that indicates the distance in meters to the closest obstacle.
    
    :param distmapfile: Path to precomputed distmap csv file, defaults to None
    :type distmapfile: str, optional
    :param binmapfile: Path to precomputed binmap csv file, defaults to None
    :type binmapfile: str, optional
    """        

    def __init__(self, distmapfile = None, binmapfile = None):
        """Constructor method. Loads gridmaps if paths are given.
        """

        self.binmap = None
        self.distmap = None
        #self.ready = False
        if distmapfile is not None:
            self.load_distmap_from_csv(distmapfile)
        
        if binmapfile is not None:
            self.load_binmap_from_csv(binmapfile)
        

    def load_distmap_from_csv(self, filename):
        """Reads and loads distmap from the given csv file. Also loads embedded metadata
        about the resolution of the gridmap and the coordinates of the origin."

        :param filename: Path to the distmap csv-file.
        :type filename: str
        """

        data = np.genfromtxt(filename, delimiter=',')   # Read data from csv-file
        self.distmap = data[1:,:]                       # The map is contained in the rows below the first row
        self.res = data[0,0]                            # Resolution metaddata in first element of first row
        self.origin_x = data[0,1]                       # Origin x-coordinate metaddata in second element of first row
        self.origin_y = data[0,2]                       # Origin y-coordinate metaddata in third element of first row
        [m,n] = np.shape(self.distmap)                  # Dimensions of the grid matrix
        self.h = m*self.res                             # Height in meters of the map
        self.w = n*self.res                             # Width in meters of the map

    def set_binmap(self, gridmap, threshold = 45.0):
        """Uses thresholding on a probabilistic gridmap to generate a binary gridmap of 
        obstacles and sets this as the self.binmap attribute.

        :param gridmap: A probabilistic gridmap.
        :type gridmap: ndarray
        :param threshold: Threshold value to identify obstacles, defaults to 45.0
        :type threshold: float, optional
        """
        self.binmap = self.create_bin_map(gridmap, threshold)
    
    def set_hw(self, height, width):
        """Sets height and width of loaded maps in meters.

        :param height: Width of the gridmap matrix (number of cells)
        :type height: int
        :param width: Height of the gridmap matrix (number of cells)
        :type width: int
        """
        self.h = height*self.res
        self.w = width*self.res


    def create_dist_map(self, gridmap, resolution, origin_x, origin_y):
        """Use this method for generating a distmap csv-file from a probabilistic gridmap. Distance gridmap (distmap), 
        with each cell containing a numerical that indicates the distance in meters to the closest obstacle.

        :param gridmap: A probabilistic gridmap.
        :type gridmap: ndarray
        :param resolution: The size of the side of one quadratic gridmap cell in meters.
        :type resolution: float
        :param origin_x: x-coordinate of the origin of the gridmap.
        :type origin_x: float
        :param origin_y: y-coordinate of the origin of the gridmap.
        :type origin_y: float
        """
        
        self.res = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        if gridmap is None:
            self.distmap = gridmap
        else:
            # Create a binary map of obstacles.
            self.binmap = self.create_bin_map(gridmap)

            # Generate distmap from 
            self.distmap = self.bin_to_dist_map()

            # Add metadata in top row of csv-file
            toprow = np.zeros((1,np.shape(self.distmap)[1]))
            toprow[0,0] = self.res
            toprow[0,1] = self.origin_x
            toprow[0,2] = self.origin_y
            map_with_meta = np.vstack((toprow, self.distmap))

            # Create csv-file
            np.savetxt("distmap.csv", map_with_meta, delimiter=",")
            
            # Uncomment belowe to visualize the distmap
            """
            plt.imshow(self.distmap, interpolation='none', cmap='Reds_r')
            plt.title("Obstacle distance heatmap")
            plt.colorbar()
            plt.show()
            """
    
    def save_binmap_as_csv(self, binmap):
        """Saves the given binmap as a csv-file.

        :param binmap: Binary map.
        :type binmap: ndarray
        """
        np.savetxt("binmap.csv", binmap, delimiter=",")
    
    def load_binmap_from_csv(self, filename):
        """Loads a binary gridmap from the given csv-file.

        :param filename: Path of the binmap csv-file to load.
        :type filename: str
        """
        self.binmap = np.genfromtxt(filename, delimiter=',')


    def create_bin_map(self, gridmap, threshold = 45.0):
        """Generates a binary gridmap from a probabilistic gridmap through thresholding.

        :param gridmap: Probabilistic gridmap
        :type gridmap: ndarray
        :param threshold: Threshold vale above which a cell in gridmap is considered occupied by an obstacle, defaults to 45.0
        :type threshold: float, optional
        :return: Binary gridmap
        :rtype: ndarray
        """

        binmap = (gridmap > threshold) # Thresholding
        return binmap

    def bin_to_dist_map(self):
        """Computes distance gridmap (distmap) from a binary gridmap. Use this method for precomputing distmap.
        Every cell of distmap contains the distance to the closest obstacle in meters.

        :return: Distance gridmap
        :rtype: ndarray
        """
        
        [m, n] = np.shape(self.binmap)
        distmap = np.zeros((m,n))
        for i in range(m):
            for j in range(n):
                # Compute distance to nearest obstacle if the cell (i,j) is not occupied by obstacle
                if not self.binmap[i,j]:
                    dist = self._distance_to_obstacle(i=i, j=j, radius=3.0)
                    distmap[i,j] = dist
        
        return distmap
    
    def point_to_index(self, x, y):
        """Maps xy-coordinates to the indices of the corresponding gridmap cells.

        :param x: x-coordinates
        :type x: list
        :param y: y-coordinates
        :type y: list
        :return: The indices of the corresponding gridmap cell
        :rtype: list of ndarray
        """
        x_pos = np.array(x)
        y_pos = np.array(y)
        i = ((y_pos - self.origin_y)/self.res).astype('int')
        j = ((x_pos - self.origin_x)/self.res).astype('int')
        return [i,j]

    def index_to_point(self, i, j):
        """Maps gridmap cell indices to the cartesian coordinates of the cell's center point.

        :param i: First index of gridmap matrix.
        :type i: int
        :param j: Second index of gridmap matrix.
        :type j: int
        :return: Coordinates
        :rtype: list of float
        """
        
        x = j*self.res + self.origin_x + self.res/2
        y = i*self.res + self.origin_y + self.res/2
        return [x,y]
    
    def is_occupied(self, i = None, j = None, x = None, y = None):
        """Check if the given grid cell or coordinate is inside an obstacle.

        :param i: First index of gridmap matrix, defaults to None
        :type i: int, optional
        :param j: Second index of gridmap matrix, defaults to None
        :type j: int, optional
        :param x: x-coordinate, defaults to None
        :type x: float, optional
        :param y: y-coordinate, defaults to None
        :type y: float, optional
        :return: True if the cell contains an obstacle
        :rtype: Boolean
        """
        if self.binmap is None:
            return None

        # If i,j given, use that, otherwise use given coordinates
        if i is None or j is None:
            idx = self.point_to_index(x,y)
            i = idx[0]
            j = idx[1]
        return self.binmap[i,j]
    
    def get_obstacle_distance(self, x,y):
        """Returns the minimum obstacle distance at a given point (x,y).

        :param x: x-cooridinate
        :type x: float
        :param y: y-coordinate
        :type y: float
        :return: The smallest distance to an obstacle at (x,y)
        :rtype: float
        """
        if self.distmap is None:
            return None
        
        [i,j] = self.point_to_index(x,y)
        return self.distmap[i,j]
    

    def _distance_to_obstacle(self, x = None, y=None, i=None, j=None, radius = 1.0):
        """Compute the smallest distance to an obstacle at coordinate (x,y) or grid index (i,j).

        :param x: x-coordinate, defaults to None
        :type x: float, optional
        :param y: y-coordinate, defaults to None
        :type y: float, optional
        :param i: First index of gridmap, defaults to None
        :type i: int, optional
        :param j: Second index of gridmap, defaults to None
        :type j: int, optional
        :param radius: The radius around the given point within which to search for obstacles, defaults to 1
        :type radius: float, optional
        :return: Smallest obstacle distance
        :rtype: float
        """

        if self.binmap is None:
            #No map initilized yet
            return None

        # If (i,j) not given use (x,y)
        if i is None or j is None:
            ego_idx = self.point_to_index(x,y)
        else:
            ego_idx = [i,j]

        # Define ranges of index within which to search for obstacles
        [m, n] = np.shape(self.binmap)
        radius_idx = int(np.round(radius/self.res))
        ego_i = ego_idx[0]
        ego_j = ego_idx[1]
        imin = max(0,ego_i-radius_idx)
        imax = min(m, ego_i+radius_idx+1)
        jmin = max(0,ego_j-radius_idx)
        jmax = min(n, ego_j+radius_idx+1)
        
        # The square portion of the map for looking for obstacles
        # Potential obstacles outside the given circular radius are ignored later 
        submap = self.binmap[imin:imax, jmin:jmax]

        if np.any(submap):
            # If the submap contain obstacle(s) look at distance to each
            obs_idx = np.argwhere(submap)   # indices of obstacle cells

            dists = (obs_idx[:,0]-ego_i+imin)**2 + (obs_idx[:,1]-ego_j+jmin)**2   # squared distances in index-space
            min_dist = np.min(dists)                # shortest index-distance
            real_dist = self.res*np.sqrt(min_dist)  # Compute the real distance
            if real_dist <= radius:                 # return real distance if point is within given radius
                return real_dist           
            else:
                return radius

        else:
            # If no obstacles are found within the radius, return the given radius
            return radius
            

class BezierPath:
    """A class to handle sequences of Bezier curves as path representations. A BezierPath is represented by a 
    number of poses (x,y,yaw) that are connected with Bezier curves with continous curvature in the attachment points.

    :param start_pose: Defines starting point and direction of the path (x,y,yaw), defaults to None
    :type start_pose: list, optional
    :param end_pose: Defines termination point and direction of the path (x,y,yaw), defaults to None
    :type end_pose: list, optional
    :param wd: Defines first order derivative at the starting point (xd,yd), should point in the same direction as start point direction, defaults to None
    :type wd: list, optional
    :param wdd: Defines second order derivative at the starting point (xdd,ydd), defaults to [0,0]
    :type wdd: list, optional
    :param alpha: Tuning parameter for control points adjacent to the mid pose, defaults to 0.5
    :type alpha: float, optional
    :param beta: Tuning parameter for the control point adjacent to the end pose, defaults to 0.5
    :type beta: float, optional
    """

    def __init__(self, start_pose = None, end_pose = None, wd = None, wdd = [0,0], alpha = 0.5, beta = 0.5):
        """Constructor method.
        """
        
        self.waypoints = []     # List of intermediate waypoints (x,y, yaw) that are connected by bezier curves to create the bezier path
        self.mid_pose = None    # Mid point
        self.path = None        # Path object containing the actual path as a  sequence of points as a PathObject
        self.is_updated = False # Flag indicating if any parameter has changed since self.path was generated
        #self.offsets = []
        self.alpha = alpha
        self.beta = beta
        if wd is None:
            # Unless wd is specified, use the direction at the start point
            wd = np.array([np.cos(start_pose[2]), np.sin(start_pose[2])])
        wdd = np.array(wdd)

        # Set start and end points
        if start_pose is not None:
            self.set_start_pose(start_pose[0],start_pose[1],wd, wdd)
        
        if end_pose is not None:
            self.set_end_pose(end_pose[0],end_pose[1],end_pose[2])

    def set_start_pose(self, x, y, wd, wdd = [0,0]):
        """Sets starting point parameters.
        """
        self.start_pose = [x, y, np.arctan2(wd[1], wd[0])]
        self.wd = wd
        self.wdd = wdd
        self.is_updated = False

    def set_end_pose(self, x, y, yaw):
        """Sets termination point parameters.
        """
        self.end_pose = [x, y, yaw]
        self.is_updated = False


    def get_length(self):
        """Approximate path length as the sum of straight segments between waypoints.

        :return: Approximated path length
        :rtype: float
        """
        
        points = [self.start_pose] + [self.mid_pose] + [self.end_pose]
        L = 0.0
        for i in range(len(points)-1):
            p1 = np.array(points[i+1][0:2])
            p0 = np.array(points[i][0:2])
            L += np.linalg.norm(p1-p0)

        return L

    def set_mid_pose(self, x,y, yaw = None):
        """Set mid pose parameters."""
        self.mid_pose = [x,y,yaw]
        self.is_updated = False

    
    def copy(self):
        """Generate a deep copy of the BezierPath object."""
        return copy.deepcopy(self)
    

    def get_bezier_path(self, num = 100, quick = False):
        """Generate a PathObject from the Bezier path representation.

        :param num: Number of points in tthe generated path, defaults to 100
        :type num: int, optional
        :param quick: If true increases computation time by disregarding derivatives, defaults to False
        :type quick: bool, optional
        :return: A PathObject containing the path as a sequence of points corresponding to the Bezier path representation.
        :rtype: PathObject
        """
        if not self.is_updated or self.path.N != num:
            # Unless an updated path exists, generate a new one
            # Computes the path points and derivatives from the waypoints
            p, pd, pdd = self.create_bpath(self.alpha, self.beta, num = num, quick = quick)

            # Create the PathObject
            if not quick:
                # Generate PathObject with derivatives
                self.path = PathObject().create_from_bezier(p, pd, pdd)
            else:
                # Generate PathObject without derivatives except for specifying the initial yaw
                yaw0 = np.arctan2(self.wd[1], self.wd[0])
                self.path = PathObject(path=p, yaw0 = yaw0)
            
        return self.path


    def create_4cpoints(self, z0, z1, z2, wd, wdd, alpha=1.0, beta = 1.0):
        """Generate two sets of control points with 4 points in each, defining two 
        Bezier curves connecting smoothly at the given mid pose z1. Starting at z0, 
        and terminating at z2.  

        :param z0: Start pose (x,y,yaw)
        :type z0: list or ndarray
        :param z1: Mid pose (x,y,yaw)
        :type z1: list or ndarray
        :param z2: End pose (x,y,yaw)
        :type z2: list or ndarray
        :param wd: First order derivative at the start pose
        :type wd: list or ndarray
        :param wdd: Second order derivative at the start pose
        :type wdd: list or ndarray
        :param alpha: Offset for control points adjacent to mid pose, defaults to 1.0
        :type alpha: float, optional
        :param beta: Offset for control point adjacent to end pose, defaults to 1.0
        :type beta: float, optional
        :return: Two sets of control points, with coordinates being the rows of two matrices.
        :rtype: (ndarray, ndarray)
        """

        r2 = np.array([np.cos(z2[2]), np.sin(z2[2])])   # Direction at path end
        z0 = np.array(z0)
        z1 = np.array(z1)
        z2 = np.array(z2)
    
        # Control points, p = first segment, q = second segment
        p0 = z0[0:2]
        p3 = z1[0:2]
        q0 = z1[0:2]
        q3 = z2[0:2]

        p1 = wd/4.0 + p0
        p2 = wdd/12.0 + 2.0*p1 -p0

        q1 = 2*q0-p2
        alpha = np.dot(4*p2-p1+q3-4*q0,r2)
        q2 = q3-alpha*r2
        p = np.vstack((p0,p1,p2,p3))
        q = np.vstack((q0,q1,q2,q3))
        return p,q
    
    def create_5cpoints(self, z0, z1, z2, wd, wdd, alpha=1.0, beta = 1.0):
        """Generate two sets of control points with 5 points in each, defining two 
        Bezier curves connecting smoothly at the given mid pose z1. Starting at z0, 
        and terminating at z2.  

        :param z0: Start pose (x,y,yaw)
        :type z0: list or ndarray
        :param z1: Mid pose (x,y,yaw)
        :type z1: list or ndarray
        :param z2: End pose (x,y,yaw)
        :type z2: list or ndarray
        :param wd: First order derivative at the start pose
        :type wd: list or ndarray
        :param wdd: Second order derivative at the start pose
        :type wdd: list or ndarray
        :param alpha: Offset for control points adjacent to mid pose, defaults to 1.0
        :type alpha: float, optional
        :param beta: Offset for control point adjacent to end pose, defaults to 1.0
        :type beta: float, optional
        :return: Two sets of control points, with coordinates being the rows of two matrices.
        :rtype: (ndarray, ndarray)
        """

        # Directional vectors
        r1 = np.array([np.cos(z1[2]), np.sin(z1[2])])
        r2 = np.array([np.cos(z2[2]), np.sin(z2[2])])
        z0 = np.array(z0)
        z1 = np.array(z1)
        z2 = np.array(z2)
        
        # Control points, p = first segment, q = second segment
        p0 = z0[0:2]
        p4 = z1[0:2]
        q0 = z1[0:2]
        q4 = z2[0:2]

        p1 = wd/4.0 + p0
        p2 = wdd/12.0 + 2.0*p1 -p0

        q1 = q0+alpha*r1
        q3 = q4-beta*r2
        p3 = p4-alpha*r1
        q2 = p2 + 4*alpha*r1 
        p = np.vstack((p0,p1,p2,p3,p4))
        q = np.vstack((q0,q1,q2,q3,q4))
        return p,q
        
        
    def create_bpath(self, alpha, beta, num = 20, quick = False):
        """Generate the path as a sequence of points from the stored path parameters.

        :param alpha: Offset for control points adjacent to mid pose
        :type alpha: float
        :param beta: Offset for control points adjacent to end pose
        :type beta: float
        :param num: Number of points to generate, defaults to 20
        :type num: int, optional
        :param quick: If true reduces computation time by not computing derivatives, defaults to False
        :type quick: bool, optional
        :return: Path, expressed as stacked points in a matrix
        :rtype: ndarray
        """
        self.z0 = self.start_pose
        self.z1 = self.mid_pose
        self.z2 = self.end_pose
        if self.z1 is not None:
            
            # Compute Bezier control points
            (p,q) = self.create_5cpoints(self.z0,self.z1,self.z2, self.wd, self.wdd, alpha = alpha, beta = beta)

            # Approximate segment lengths for distributing the path points proportionally to their sizes.
            L1 = np.sqrt((self.z0[0]-self.z1[0])**2 + (self.z0[1]-self.z1[1])**2)
            L2 = np.sqrt((self.z1[0]-self.z2[0])**2 + (self.z1[1]-self.z2[1])**2)
            
            num1 = int(np.round(num*L1/(L1+L2)))    # Number of points in first segment
            num2 = num - num1                       # Number of points in second segment
            
            # Generate the path segments from the control points.
            P, Pd, Pdd = calc_bezier_path_and_derivatives(np.array(p), n_points = num1, quick = quick)
            Q, Qd, Qdd = calc_bezier_path_and_derivatives(np.array(q), n_points = num2+1, quick = quick) # num2+1 since the first point is excluded later
            
            # Join the segments excluding first point of Q to avoid a duplicate
            path = np.vstack((P,Q[1:]))
            if not quick:
                path_d = np.vstack((Pd,Qd[1:]))
                path_dd = np.vstack((Pdd,Qdd[1:]))
            else:
                path_d = None
                path_dd = None
            
        else:
            # If mid point has not been defined, generate only one segment using 4 control points
            path, path_d, path_dd = calc_4points_bezier_path(self.z0[0], self.z0[1], self.z0[2], self.z2[0], self.z2[1], self.z2[2], 2.0, n_points = num)
        
        return path, path_d, path_dd
        

class PathObject:
    """A class for representing paths as sequence of points and a number of convenience functions for handling them. 
    A path is represented with an array waypoints containing information of each point about position (x,y), orientation angle (yaw),
    velocity (if available), curvature, timestamp (if available), point separation (ds), and path derivatives.

    :param waypoints: A matrix of path data, if given, used to represent the path, defaults to None
    :type waypoints: ndarray, optional
    :param path: 2-column array with rows representing points of a path. 
                If given, used to represent the path and missing data is generated, defaults to None
    :type path: ndarray, optional
    :param num: Number of desired points on path. If given, path is resampled to match, defaults to None
    :type num: int, optional
    :param yaw0: Orientation at the path's start point. If given this will be used, otherwise it will be approximated 
                from the path points, which could be inaccurate defaults to None
    :type yaw0: float, optional
    """

    # Column indices 
    COL_X = 0
    COL_Y = 1
    COL_YAW = 2
    COL_V = 3
    COL_CURV = 4
    COL_TIME = 5
    COL_DS = 6
    COL_XD = 7
    COL_YD = 8
    COL_XDD = 9
    COL_YDD = 10

    def __init__(self, waypoints = None, path = None, num = None, yaw0 = None):
        """Constructor method.
        """

        if waypoints is not None:
            # The waypoint array is given. Use it.
            self.waypoints = waypoints
            self.N = len(self.waypoints)    # Number of points
            self.refresh()                  # Fill in potentially missing data
            
        elif path is not None:
            # A sequence of points is given. Use it and compute the other data from the points.

            # Create waypoint array
            self.waypoints = np.empty((len(path), 11))
            self.waypoints[:] = np.nan
            self.waypoints[:,[self.COL_X, self.COL_Y]] = path
            self.N = len(self.waypoints)
            if num is not None:
                self.resample(num = num)
            self.refresh(start_yaw= yaw0)
        else:
            # No data is given.
            self.waypoints = None
            self.N = 0
            self.L = 0

    def __repr__(self):
        """Printing options"""
        retstring = "x, y, yaw, v, curv, time, ds, xd, yd, xdd, ydd\n"
        with np.printoptions(precision=2, suppress=True, formatter={'float_kind':'{:0.2f}'.format}, threshold=np.inf):
            retstring += str(self.waypoints)
        return retstring
    
    def create_from_bezier(self, p, pd, pdd):
        """Create a PathObject from a path and its derivatives expressed as sequences of points.

        :param p: Path array
        :type p: ndarray
        :param pd: First order derivative of path array
        :type pd: ndarray
        :param pdd: Second order derivative of path array
        :type pdd: ndarray
        :return: An instance of PathObject
        :rtype: PathObject
        """
        # Create waypoint array
        self.waypoints = np.empty((len(p), 11))
        self.waypoints[:] = np.nan
        self.waypoints[:, [self.COL_X, self.COL_Y]] = p
        self.waypoints[:, [self.COL_XD, self.COL_YD]] = pd
        self.waypoints[:, [self.COL_XDD, self.COL_YDD]] = pdd
        self.N = len(self.waypoints)

        # Compute yaw and populate waypoint array
        yaw = np.arctan2(pd[:,1], pd[:,0])
        self.waypoints[:, self.COL_YAW] = yaw
        
        # Compute point separation ds and total length L
        ds = np.linalg.norm(p[1:,:]-p[0:-1,:], axis=1)
        self.L = np.sum(ds)
        self.waypoints[1:, self.COL_DS] = ds
        self.waypoints[0, self.COL_DS] = 0

        # Compute curvature
        curv = -self.compute_curv(self.waypoints[:,self.COL_YAW], self.waypoints[:,self.COL_DS])
        self.waypoints[:, self.COL_CURV] = curv
        return self


    def get_closest_idx(self, x, y, idx_start = None, idx_stop = None):
        """Find the index of the point on path that is closest to a point (x,y) not on the path.
        Optionally restrict the search space to a sub set of the points by specifying idx_start and idx stop.

        :param x: x-coordinate of the external point.
        :type x: float
        :param y: y-coordinate of the external point.
        :type y: float
        :param idx_start: Lower bound of indices to consider, defaults to None
        :type idx_start: int, optional
        :param idx_stop: Upper bound of indices to consider, defaults to None
        :type idx_stop: int, optional
        :return: Index of the closest path point.
        :rtype: int
        """
        
        if self.N == 1:
            return 0

        # Determine range of indices within, which to search.
        if idx_start is None:
            idx_start = 0
        if idx_stop is None:
            idx_stop = self.N-1

        idx_start = max(0,min(self.N-2, idx_start))
        idx_stop = max(0, min(self.N-1, idx_stop))

        # Compute distances and find the minimum
        points = self.waypoints[idx_start:idx_stop, :]
        indices = np.arange(idx_start, idx_start+len(points))
        pos = np.array([x, y])
        dists = np.linalg.norm(pos - points[:,(self.COL_X, self.COL_Y)], axis=1)
        min_idx = np.argmin(dists)
        return indices[min_idx]

    
    def get_xy(self, idx = None):
        """Get the positions of the points on the paths as lists.

        :param idx: If given, return the position of the point with correponding index, otherwise returns all points, defaults to None
        :type idx: int, optional
        :return: Lists of the x and y coordinates
        :rtype: (list of float, list of float) or (float, float)
        """
        
        if idx is None:
            x = self.waypoints[:, self.COL_X]
            y = self.waypoints[:, self.COL_Y]
        else:
            x = self.waypoints[idx, self.COL_X]
            y = self.waypoints[idx, self.COL_Y]
        return x,y
    def get_ds(self, idx = None):
        """Get point separation information (ds). 

        :param idx: If given, return the data of the point with correponding index, otherwise returns data for all points, defaults to None
        :type idx: int, optional
        :return: Point separation 
        :rtype: list of float or float
        """
        if idx is None:
            return self.waypoints[:, self.COL_DS]
        else:
            return self.waypoints[idx, self.COL_DS]
    
    def copy(self):
        """Get a deep copy of the PathObject.
        """
        return copy.deepcopy(self)

    def extend_path(self, extension_path, start_idx = 0):
        """Append a path to this one and return as a new path. 

        :param extension_path: Instance of PathObject to append
        :type extension_path: PathObject
        :param start_idx: Specify this to only include points with higher indices, defaults to 0
        :type start_idx: int, optional
        :return: Extended path
        :rtype: PathObject
        """
        if self.N == 0:
            wp = extension_path.waypoints
        else:
            wp = np.vstack((self.waypoints, extension_path.waypoints[start_idx:,:]))
        return PathObject(waypoints=wp)

    def get_vel(self, idx = -1):
        """Get velocity data at the given point.

        :param idx: Index of the point to get, defaults to -1
        :type idx: int, optional
        :return: Velocity data
        :rtype: float
        """
        return self.waypoints[idx, self.COL_V]


    def compute_ds(self, x,y):
        """Compute point separation.

        :param x: x-coordinates
        :type x: list of float or ndarray
        :param y: y-coordinates
        :type y: list of float or ndarray
        :return: Point separations
        :rtype: ndarray
        """
        pos1 = np.column_stack((x[1:], y[1:]))
        pos0 = np.column_stack((x[:-1], y[:-1]))
        DS = np.zeros(len(x))
        DS[1:] = np.linalg.norm(pos1-pos0, axis=1)
        DS[0] = 0
        return DS

    
    def refresh(self, start_yaw = None, end_yaw = None):
        """ Compute ds, yaw and curv from point locations given that x, y are complete

        :param start_yaw: Force an initial yaw angle, defaults to None
        :type start_yaw: float, optional
        :param end_yaw: Force a termination yaw, defaults to None
        :type end_yaw: float, optional
        """
        
        self.N = len(self.waypoints)
        if self.N > 1:
            x,y = self.get_xy()
            self.waypoints[:,self.COL_DS] = self.compute_ds(x, y)
            self.waypoints[:,self.COL_YAW] = self.compute_yaw(x, y)
            if start_yaw is not None:
                self.waypoints[0,self.COL_YAW] = start_yaw
            self.waypoints[:,self.COL_CURV] = self.compute_curv(self.waypoints[:,self.COL_YAW], self.waypoints[:,self.COL_DS])
            self.L = np.sum(self.waypoints[:,self.COL_DS])
            self.N = len(self.waypoints)
        else:
            self.L = 0


    def compute_curv(self, yaw, ds):
        """Compute curvature as dyaw/ds.

        :param yaw: Yaw angles
        :type yaw: ndarray
        :param ds: Point separation
        :type ds: ndarray
        :return: Curvatures
        :rtype: ndarray
        """
        ds = np.copy(ds)
        ds[ds==0] = 10000   # Avoiding singularities. Assuming curvature is small between points located at the same position.
        curvs = np.zeros(len(yaw))
        if len(ds) > 2:
            dyaw = yaw[2:] - yaw[:-2]
            dyaw[dyaw > np.pi] = dyaw[dyaw > np.pi] -2*np.pi
            dyaw[dyaw < -np.pi] = dyaw[dyaw < -np.pi] +2*np.pi
            curvs[1:-1] = (dyaw)/(ds[1:-1] + ds[2:])
        curvs[0] = (yaw[1]-yaw[0])/ds[1]
        curvs[-1] = (yaw[-1]-yaw[-2])/ds[-1]
        return -curvs

        
    def compute_yaw(self, x, y):
        """Compute yaw angles as the average angle of tangent lines to points ahead and behind each point.

        :param x: x-coordinates
        :type x: list of float or ndarray
        :param y: y-coordinate
        :type y: list of float or ndarray
        :return: Yaw angles
        :rtype: ndarray
        """
        yaws = np.zeros(len(x))
        r0 = np.column_stack((x[0:-2], y[0:-2]))
        r1 = np.column_stack((x[2:], y[2:]))
        dr = r1-r0
        yaws[1:-1] = np.arctan2(dr[:,1], dr[:,0])
        yaws[0] = np.arctan2(y[1]-y[0], x[1]-x[0])
        yaws[-1] = np.arctan2(y[-1]-y[-2], x[-1]-x[-2])
        return yaws


    def resample(self, num):
        """Change the number of points in the path by changing their separation and interpolating.

        :param num: New number of points to use.
        :type num: int
        :return: Path with the given number of points.
        :rtype: PathObject
        """
        if num is None or num <1:
            return None
        X, Y = self.get_xy()
        ds = self.compute_ds(X,Y)
        L = np.sum(ds)
        seg_len = L/(num-1)
        s = np.cumsum(ds)
        s_sample = seg_len*np.arange(num)
 
        X_new = np.interp(s_sample, s, X)
        Y_new = np.interp(s_sample, s, Y)
        new_path = np.column_stack((X_new, Y_new))
        return PathObject(path=new_path)


    def split_at_idx(self, idx):
        """Splits the path in two at the given index, which is located in the second path. 

        :param idx: Index indicating where to split
        :type idx: int
        :return: The two paths
        :rtype: (PathObject, PathObject)
        """
        if idx == 0:
            path1 = PathObject()
            path2 = self.copy()

        elif abs(idx) < self.N-2:
            path1 = PathObject(self.waypoints[0:idx+1,:])
            path2 = PathObject(self.waypoints[idx+1:,:])
        else: 
            path1 = self.copy()
            path2 = PathObject()
        return path1, path2

    def get_curvs(self, idx = None):
        """Get curvature along the entire path or at the specified index.
        """
        if idx is None:
            return self.waypoints[:, self.COL_CURV]
        else:
            return self.waypoints[idx, self.COL_CURV]
    
    def get_curv(self, idx = -1, horizon = 3, look_ahead = 0):
        """Alternative method for getting the curvature with variable number of points and center to include.

        :param idx: Base index for computing curvature, defaults to -1
        :type idx: int, optional
        :param horizon: Number of points to include in the computation in one direction, defaults to 3
        :type horizon: int, optional
        :param look_ahead: Specify to center the computation at other points points, defaults to 0
        :type look_ahead: int, optional
        :return: Curvature value
        :rtype: float
        """
        idx1 = min(self.N-1, idx + look_ahead + horizon)
        idx0 = max(0, idx + look_ahead - horizon)
        yaw1 = self.waypoints[idx1, self.COL_YAW]
        yaw0 = self.waypoints[idx0, self.COL_YAW]
        dyaw = yaw1-yaw0
        while dyaw > np.pi:
            dyaw -= 2*np.pi

        while dyaw < -np.pi:
            dyaw += 2*np.pi
        
        dists = self.waypoints[idx0+1:idx1+1, self.COL_DS]
        ds = np.sum(dists)
        if abs(ds) < 0.005:
            return float('nan')
        else:
            return -dyaw/ds
    

    def get_yaw(self, idx = None):
        """Get yaw angles along the entire path or at the specified index.
        """
        if idx is not None:
            return self.waypoints[idx, self.COL_YAW]
        else:
            return self.waypoints[:, self.COL_YAW]

    def get_derivatives(self, idx = None):
        """Get derivative information along the entire path or at the specified index.
        """
        if idx is not None:
            d = self.waypoints[idx, [self.COL_XD, self.COL_YD]]
            dd = self.waypoints[idx, [self.COL_XDD, self.COL_YDD]]
        else:
            d = self.waypoints[:, [self.COL_XD, self.COL_YD]]
            dd = self.waypoints[:, [self.COL_XDD, self.COL_YDD]]
        return d, dd

    def get_pose(self, idx = 0, dist_offset = None):
        """Get the pose (x,y,yaw) at the specified index, with an additional optional offset.

        :param idx: Path index, defaults to 0
        :type idx: int, optional
        :param dist_offset: An additional offset to get the pose of a point on the path a corresponding 
        distance away from the given index, both positive and negative values are valid, defaults to None
        :type dist_offset: float, optional
        :return: The pose (x,y,yaw) and also the closest index if dist_offset is specified.
        :rtype: list of float or (list of float, int)
        """
   
        idx = min(self.N-1, idx)
        if dist_offset is None:
            return self.waypoints[idx, [self.COL_X, self.COL_Y, self.COL_YAW]]

        if dist_offset == 0.0:
            return self.waypoints[idx, [self.COL_X, self.COL_Y, self.COL_YAW]], idx
        d = 0.0
        i = idx
        N = len(self.waypoints)
        abs_dist = np.abs(dist_offset)

        while abs(i) <= N :
            next_i = i + int(np.sign(dist_offset))
            if next_i >= N or next_i < -N:
                # if outside range of path return end-point
                return self.waypoints[i, [self.COL_X, self.COL_Y, self.COL_YAW]], i

            next_d = d + self.waypoints[i, self.COL_DS]
            if next_d > abs_dist:
                # if exceeding sought offset --> interpolate
                r1 = self.waypoints[next_i, [self.COL_X, self.COL_Y]]
                r0 = self.waypoints[i, [self.COL_X, self.COL_Y]]
                yaw1 = self.waypoints[next_i, self.COL_YAW]
                yaw0 = self.waypoints[i, self.COL_YAW]
                pos = r0 + (r1-r0)*(abs_dist-d)/(next_d-d)
                yaw = yaw0 + (yaw1-yaw0)*(abs_dist-d)/(next_d-d)

                return [pos[0], pos[1], yaw], i

            i = next_i
            d = next_d
        # If this is reached something is wrong
        return None

    def timestamp(self, index):
        """Get timestamp data at the given index.
        """
        return self.waypoints[index, self.COL_TIME]


class PathHandler:
    """A class for building path objects when . Similar to PathObject but with a few application specific methods.
    Each entry contains position (x,y), curvature, timestamp, and additional state variables of the leader at that point (vx, yaw)
    """

    # Column indices
    COL_X = 0
    COL_Y = 1
    COL_YAW = 2
    COL_VX = 3
    COL_CURV = 4
    COL_TIME = 5
    COL_DS = 6

    def __init__(self, maxlen = None):
        """Contructor method.

        :param maxlen: Maximum number of points in the path, defaults to None
        :type maxlen: int, optional
        """
        self.waypoints = None
        self.maxlen = maxlen

    def __repr__(self):
        """Print options
        """
        retstring = "x, y, yaw, v, curv, time\n"
        with np.printoptions(precision=2, suppress=True, formatter={'float_kind':'{:0.2f}'.format}):
            retstring += str(self.waypoints)
        return retstring
    
    def add_waypoint(self, x, y, yaw, vx, time):
        """Adds one waypoint in the form of one new row at the end of the waypoints arrau
        If No movement is recorded very small change between previous waypoint, exclude it
        """

        wp = np.ones((1,7))
        wp[0, self.COL_X] = x
        wp[0, self.COL_Y] = y
        wp[0, self.COL_YAW] = yaw
        wp[0, self.COL_VX] = vx
        wp[0, self.COL_TIME] = time
        wp[0, self.COL_CURV] = float('nan')
        wp[0, self.COL_DS] = float('nan')
        
        new_pos = np.array((x,y))
        ds = 0.0

        if self.waypoints is None:
            # if first entry
            self.waypoints = wp    
        elif np.shape(self.waypoints)[0]<2:
            # if very few entries, append
            old_pos = self.waypoints[-1, [self.COL_X, self.COL_Y]]
            new_pos = np.array([x,y])
            ds = np.linalg.norm(new_pos-old_pos)
            wp[0, self.COL_DS] = ds
            self.waypoints = np.vstack((self.waypoints, wp))
        elif np.linalg.norm(new_pos-self.waypoints[-2,[self.COL_X, self.COL_Y]])<0.01:
            # if >2 entrries and small change --> replace last entry with new
            old_pos = self.waypoints[-1, [self.COL_X, self.COL_Y]]
            new_pos = np.array([x,y])
            ds = np.linalg.norm(new_pos-old_pos)
            wp[0, self.COL_DS] = ds
            self.waypoints[-1,:] = wp
        else:
            # otherwise append
            old_pos = self.waypoints[-1, [self.COL_X, self.COL_Y]]
            new_pos = np.array([x,y])
            ds = np.linalg.norm(new_pos-old_pos)
            wp[0, self.COL_DS] = ds
            self.waypoints = np.vstack((self.waypoints, wp))
                    
        if self.maxlen is not None:
            # truncate if too long
            self.truncate_list()
    

    def truncate_list(self):
        """Removes the oldest waypoints until is has maxlen rows. 
        """
        while np.shape(self.waypoints)[0] > self.maxlen:       
            self.waypoints = np.delete(self.waypoints, (0), axis=0)

    def get_path(self):
        """Create a path object from the information in the path handler.

        :return: A corresponding path object
        :rtype: PathObject
        """
        return PathObject(waypoints = np.copy(self.waypoints))
