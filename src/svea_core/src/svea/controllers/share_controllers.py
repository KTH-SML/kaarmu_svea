"""
Contains a PID controller class and an MPC class, used for longitudinal and lateral control 
during shared teleoperation. Also contains a saturation function sat.
The moethods get_ctrl are intended to be called at regular intervals corresponding with 
the specified sampling periods.
"""
import numpy as np
from scipy.linalg import expm
import cvxpy as cp


class Pid:
    """Pid controller

    :param K: Proportional gain, defaults to 1.0
    :type K: float, optional
    :param Ti: Integration time, defaults to 100.0
    :type Ti: float, optional
    :param Td: Derivative time, defaults to 0.0
    :type Td: float, optional
    :param b: Fraction of reference to use in the P term, defaults to 1.0
    :type b: float, optional
    :param N: Maximum derivative gain, defaults to 1.0
    :type N: float, optional
    :param dt: Sampling period, defaults to 0.01
    :type dt: float, optional
    :param Imax: Maximum integral value, defaults to None
    :type imax: Float, optional
    """

    def __init__(self, K = 1.0, Ti = 100.0, Td =0.0, b = 1.0, N = 1.0, dt = 0.01, Imax = None):
        """Constructor method.
        """
        self.K = K
        self.Td = Td
        self.Ti = Ti
        self.P = 0.0
        self.I = 0.0
        self.D = 0.0
        self.N = N
        self.b = b          
        self.dt = dt        
        self.y_prev = None  # Previous system state
        self.r_prev = None  # Previous reference signal
        self.Imax = Imax

    def reset(self):
        """Set the P, I and D terms to zero and forget previous reference and system state.
        Call this if the controller is overridden by another control method and must be restarted.
        """
        self.P = 0.0
        self.I = 0.0
        self.D = 0.0
        self.y_prev = None
        self.r_prev = None


    def get_ctrl(self, ref, state, increase_integrator = True):
        """Compute control signal based on a reference value and the current system state.

        :param ref: Reference (desired) value
        :type ref: float
        :param state: State variable value to be controlled
        :type state: float
        :param increase_integrator: For avoiding integrator windup this can be set to False 
            to not increase the I-term, defaults to True
        :type increase_integrator: bool, optional
        :return: The control signal resulting from the specified controller parameters.
        :rtype: float
        """
        r = ref
        y = state

        # Initial values
        if self.y_prev is None or self.r_prev is None:
            self.y_prev = y
            self.r_prev = r

        # Compute the values for each term
        if increase_integrator:
            self.I = self.I + self.K*self.dt/self.Ti*(self.r_prev-self.y_prev)
            if self.Imax is not None:
                self.I = max(-self.Imax, min(self.Imax, self.I))

        self.D = self.Td/(self.Td+self.N*self.dt)*self.D - self.K*self.Td*self.N/(self.Td+self.N*self.dt)*(y - self.y_prev)
        self.P = self.K*(self.b*r-y)

        self.y_prev = y
        self.r_prev = r

        # Sum to get the control signal u
        u = self.P + self.I + self.D
        return u

def sat(x, x_max):
    """Saturation function.
    """
    return max(-x_max, min(x_max, x))


class MPC:
    """A class to handle MPC for lateral control of a SVEA vehicle. Optimization is 
    solved with CVXPY, which has poor real-time performance. The get_ctrl method should
     be called regularly matching the given sampling period dt.

    :param dt: Sampling period, defaults to 0.1
    :type dt: float, optional
    :param horizon_p: Prediction horizon, defaults to 2
    :type horizon_p: int, optional
    :param horizon_c: Control horizon, defaults to 2
    :type horizon_c: int, optional
    :param q: Weights to balance between different control errrors (e1, de1, e2, de2), defaults to [1.0, 1.0, 1.0, 1.0]
    :type q: list of float, optional
    :param r: Weight to balance between control errors and control size, defaults to 1.0
    :type r: float, optional
    """


    def __init__(self, dt = 0.1, horizon_p = 2, horizon_c = 2, q = [1.0, 1.0, 1.0, 1.0], r = 1.0):
        """Constructor method. Sets constant model and controller parameters and creates the CVXPY problem.
        """
        
        self._set_constants(dt = dt,horizon_p = horizon_p, horizon_c = horizon_c, q = q, r = r)
        self.prob = self._create_problem()


    def _set_constants(self, dt = 0.1, horizon_p = 3, horizon_c = 3, q = [1.0, 1.0, 1.0, 1.0], r = 1.0):
        """Sets constant model (hard-coded) and controller parameters.
        """

        # Model parameters
        self.mass = 6.0         # Vehicle mass [kg]
        self.Cf = 5             # Front tire cornering stiffness [N/rad]
        self.Cr = 5             # Rear tire cornering stiffness [N/rad]
        self.lf = 0.16          # Distance cg -> front wheels / assumed half of wheel base [m]
        self.lr = 0.16          # Distance cg -> real wheels / assumed half of wheel base [m]
        self.Iz = 0.07          # Moment of inertia around z-axis [kg m2]
        
        # Controller parameters
        self.dt = dt                                # Sampling interval
        self.Np = horizon_p                         # Prediction horizon
        self.Nc = horizon_c                         # Control horizon
        self.q = q                                  # Tracking error weight
        self.Q = np.diag(np.tile(self.q, self.Np))  # weight matrix Q
        self.r = r                                  # Control weight
        self.R = self.r*self.Np*np.eye(self.Nc)     # Weight matrix R
        self.u_max = 35.0*np.pi/180                 # Maximum steering angle 35 degrees

    
    def _update_parameters(self, vx, curv, e0_val, u0_val):
        """Update parameters of the linear system model
        y = A*y +B*u + C*theta_des and y(k+1) = Phi*y(t)+Gamma*(B*u(t)+C*theta_des))

        :param vx: Current longitudinal velocity
        :type vx: float
        :param curv: Current path curvature
        :type curv: float
        :param e0_val: Current measured control errors (e1, de1, e2, de2)
        :type e0_val: list of float
        :param u0_val: Currently applied steering angle (radians)
        :type u0_val: float
        """
        
        vx = np.abs(vx) # Only considering forward motion

        # The elements of the matrix A
        a11 = -(2.0*self.Cf+2.0*self.Cr)/(self.mass*vx)
        a12 = (2.0*self.Cf+2.0*self.Cr)/self.mass
        a13 = (-2.0*self.Cf*self.lf+2.0*self.Cr*self.lr)/(self.mass*vx)
        a31 = -(2.0*self.Cf*self.lf-2.0*self.Cr*self.lr)/(self.Iz*vx)
        a32 = (2.0*self.Cf*self.lf-2.0*self.Cr*self.lr)/self.Iz
        a33 = -(2.0*self.Cf*self.lf**2+2.0*self.Cr*self.lr**2)/(self.Iz*vx)

        # The elements of the vector B
        b1 = 2*self.Cf/self.mass
        b3 = 2*self.lf*self.Cf/self.Iz

        # The elements of the vector C
        c1 = a13 - vx
        c3 = a33

        # Matrices A and B and C concatenated into D
        A = np.array([[0, 1, 0, 0], [0, a11, a12, a13],[0,0,0,1], [0, a31, a32, a33]])
        D = np.array([[0, 0], [b1, c1], [0, 0], [b3, c3]])
        
        # Compute Phi and Gamma in the discretized model
        G = np.zeros((6,6))
        G[0:4,0:4] = A
        G[0:4,4:6] = D
        
        expmatrix = expm(self.dt*G) # Matrix exponential
        phi = expmatrix[0:4,0:4]
        gamma = expmatrix[0:4, 4:6]

        # Update parameters
        self.A.value = phi
        self.B.value = gamma[:,0]
        self.C.value = gamma[:,1]*vx*curv
        self.e0.value = e0_val
        self.u0.value = u0_val
    

    def _create_problem(self):
        """Set up the MPC problem within the CVXPY framework.

        :return: An instantce of the cvxpy problem object.
        :rtype: Problem
        """
        # Model parameters
        self.A = cp.Parameter((4,4))       
        self.B = cp.Parameter(4)
        self.C = cp.Parameter(4)
        self.E = cp.Variable(4*self.Np)     # current and future errors
        self.U = cp.Variable(self.Nc)       # controls
        self.dU = cp.Variable(self.Nc)      # control updates
        self.e0 = cp.Parameter(4)           # initial errors
        self.u0 = cp.Parameter()            # previous control
        
        # Cost function
        cost = cp.quad_form(self.E, self.Q) + cp.quad_form(self.dU, self.R)

        # Problem constraints
        constr = []                       
        for np in range(self.Np):
            if np == 0:
                # Current error and control is given
                e = self.e0
                u = self.u0
            else:
                e = self.E[(4*(np-1)):(4*(np-1)+4)]
                u = self.U[min(np-1, self.Nc-1)]    

            # Errors are predicted with system model
            next_e = self.A*e + self.B*u + self.C
            constr += [self.E[4*np] == next_e[0]]
            constr += [self.E[4*np+1] == next_e[1]]
            constr += [self.E[4*np+2] == next_e[2]]
            constr += [self.E[4*np+3] == next_e[3]]
        
        for nc in range(self.Nc):
            if nc == 0:
                # Initial control update with given current steering angle
                constr += [self.dU[0] == self.U[0] - self.u0]
            else:
                # dU is the difference in steering angle between time steps    
                constr += [self.dU[nc] == self.U[nc] - self.U[nc-1]]
        # All steering angles smaller than or equal to u_max
        constr += [cp.norm(self.U, 'inf') <= self.u_max]

        # Return the cvxpy problem
        return cp.Problem(cp.Minimize(cost), constr)
   
        
    def _solve_problem(self, e0_val, u0_val, curv_val, vx_val):
        """Solve the cvxpy problem once with the problem parameters updated according to the arguments.

        :param e0_val: Currrent measured error (e1, de1, e2, de2)
        :type e0_val: list of float
        :param u0_val: Currently applied steering angle
        :type u0_val: float
        :param curv_val: Path curvature
        :type curv_val: float
        :param vx_val: Current longitudinal velocity
        :type vx_val: float
        """

        self._update_parameters(vx_val, curv_val, e0_val, u0_val)
        self.prob.solve(solver=cp.OSQP, warm_start = True)          # This is by far the slowest step


    def get_ctrl(self, e0, v, curv, u0):
        """Compute the steering angle to apply in the next time step by solving the MPC optimization problem. 

        :param e0: Currrent measured error (e1, de1, e2, de2)
        :type e0: list of float
        :param v: Current longitudinal velocity
        :type v: float
        :param curv: Path curvature
        :type curv: float
        :param u0: Currently applied steering angle
        :type u0: float
        :return: Steering angle (radians)
        :rtype: float
        """
        # Small velocity, return current steering angle to avoid model singularites.
        if np.abs(v) < 0.05:
            return u0

        # Solve MPC optimization. If it fails, return current steering angle.
        try:
            self._solve_problem(e0, u0, curv, v)
        except Exception as e:
            print("MPC FAILED", e)
            return u0
        
        # Retrieve the optimal control values found and retur the first element.
        u_opt = self.U.value
        return u_opt[0]