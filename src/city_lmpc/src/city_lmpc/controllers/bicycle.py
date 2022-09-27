import numpy as np
from casadi import Opti, DM, diag, norm_2, if_else, norm_1, norm_fro, sqrt, vertcat
from city_lmpc.models.frenet import FrenetModel


class BicycleMPC():
    TOL_TARGET = 1e-1
    TOL_CON = 1e-5  # Tolerance for breaking constraints
    counter = 0

    def __init__(self, model: FrenetModel, N=7, Q=[1, 1, 1, 1], R=[1, 100], xlb=[-np.inf, -np.inf, -np.inf, -np.inf], xub=[np.inf, np.inf, np.inf, np.inf], ulb=[-np.inf, -np.inf], uub=[np.inf, np.inf], state_noise=False, input_noise=False) -> None:
        self.model = model
        # Number of states and inputs
        self.n_states = len(self.model.dae.x)
        self.n_inputs = len(self.model.dae.u)
        # Number of control intervals
        self.N = N
        # Initial and final states
        self.x0 = self.model.x0
        self.xf = DM.zeros(self.n_states, 1)
        # MPC Parameters
        self.Q = diag(Q)
        self.R = diag(R)
        self.xlb = DM(xlb)
        self.xub = DM(xub)
        self.ulb = DM(ulb)
        self.uub = DM(uub)
        # Noise settings
        self.state_noise = state_noise
        self.input_noise = input_noise
        # setup optimizer
        self._build_optimizer()

    def _build_optimizer(self):
        opti = Opti()
        self.opti = opti

        u = opti.variable(self.n_inputs, self.N)
        self.u = u
        x = opti.variable(self.n_states, self.N + 1)
        self.x = x

        x_ref = opti.parameter(self.n_states, self.N + 1)
        self.x_ref = x_ref
        x0 = opti.parameter(self.n_states, 1)
        self.x0 = x0

        # Extract variables (for better readability in equations below)
        N, xlb, xub, ulb, uub, Q, R = self.N, self.xlb, self.xub, self.ulb, self.uub, self.Q, self.R

        # set objective function
        self.cost = 0
        n_sub = self.n_states - 1
        for k in range(N):
            x_err = x[:n_sub, k] - x_ref[:n_sub, k]
            self.cost += x_err.T @ Q[:n_sub, :n_sub] @ x_err
            #   from https://math.stackexchange.com/questions/341749/how-to-get-the-minimum-angle-between-two-crossing-lines
            angle_diff = np.pi - \
                norm_2(norm_2(x[-1, k] - x_ref[-1, k]) - np.pi)
            self.cost += angle_diff**2 * Q[-1, -1]

            self.cost += u[:, k].T @ R @ u[:, k]

        k += 1
        x_err = x[:n_sub, k] - x_ref[:n_sub, k]
        self.cost += x_err.T @ Q[:n_sub, :n_sub] @ x_err
        angle_diff = np.pi - \
            norm_2(norm_2(x[-1, k] - x_ref[-1, k]) - np.pi)
        self.cost += angle_diff**2 * Q[-1, -1]

        opti.minimize(self.cost)

        # set dynamics constraints
        for t in range(N):
            x_next, _ = self.model.f(
                # curvature is included as a parameter in the simulation
                x[:, t], u[:, t], state_noise=self.state_noise, input_noise=self.input_noise)
            opti.subject_to(
                x[:, t + 1] == x_next)

        opti.subject_to(opti.bounded(ulb, u, uub))
        opti.subject_to(opti.bounded(xlb, x, xub))
        opti.subject_to(x[:, [0]] == x0)

        p_opts = {
            "verbose": False,
            "expand": True,
            "print_in": False,
            "print_out": False,
            "print_time": False}
        s_opts = {"max_iter": 150,
                  "print_level": 1,
                  #   "mu_strategy": "adaptive",
                  #   "mu_init": 1e-5,
                  #   "mu_min": 1e-15,
                  "fixed_variable_treatment": "make_constraint",
                  "barrier_tol_factor": 1,
                  }
        opti.solver("ipopt", p_opts,
                    s_opts)

        # self.mpc = opti.to_function("MPC", [x0, x_ref, curvature], [u[:, 0], self.cost, x[:, N], u], [
        #     "xt", "xN", "qN", "intruder_c"], ["u_opt", "cost", "xN", "u"])

    def get_ctrl(self, x0, x_ref):
        """
            Args:
                x0 - is the final state in the horizon pulled from the sampled safe set
                x_ref - DM of dimensions (n_states,N+1) containing the reference trajectory
                curvature - DM of dimensions (1,N) containing the curvature over the time horizon
            Returns:
                u_opt - is the first optimal control input
        """
        self.opti.set_value(self.x0, x0)
        self.opti.set_value(self.x_ref, x_ref)
        try:
            self.opti.solve()
        except RuntimeError as e:
            self.opti.debug.show_infeasibilities()
            raise(e)
        u_opt = np.expand_dims(self.opti.value(self.u[:, 0]), axis=1)
        x_pred = self.opti.value(self.x)
        return u_opt, x_pred
