# system path management (to import from adjacent directories)
from city_lmpc.models.generic import GenericModel
import numpy as np
from casadi import norm_2

# Example of how to use the GenericModel


class BicycleModel(GenericModel):
    TAU = 0.1
    NEW_PARAM = 0.1
    WB = 0.324  # [m] Wheelbase of SVEA vehicle
    INPUT_NOISE_STD = 0.1
    STATE_NOISE_STD = 0.01

    def __init__(self, x0, dt=0.1):
        # transform the rotation of the vehicle from global to track frame
        # phi = x0[-1]
        # x0[-1] = phi - phi_s0
        super().__init__(x0, dt)

    def __str__(self) -> str:
        with np.printoptions(precision=2, suppress=True):
            x, u = self.get_trajectory()
            out = "Trajectory of platoon"
            out += f"\nx: {x[0,:]}\n"
            if len(self.input_trajectory) > 0:
                out += f"u: {u[0,:]}"
            return out

    def _build_dae(self):
        # add states
        x = self.dae.add_x("x")
        y = self.dae.add_x("y")
        v = self.dae.add_x("v")
        phi = self.dae.add_x("phi")

        # add inputs
        v_u = self.dae.add_u("v_u")
        delta = self.dae.add_u("delta")

        x_dot = v * np.cos(phi)
        y_dot = v * np.sin(phi)
        v_dot = (v_u - v) / self.TAU
        phi_dot = v * np.tan(delta) / self.WB

        self.dae.add_ode("x_dot", x_dot)
        self.dae.add_ode("y_dot", y_dot)
        self.dae.add_ode("v_dot", v_dot)
        self.dae.add_ode("phi_dot", phi_dot)

        # set units
        self.dae.set_unit("x", "m")
        self.dae.set_unit("y", "m")
        self.dae.set_unit("v", "m/s")
        self.dae.set_unit("phi", "rad")


if __name__ == "__main__":
    x0 = [0, 0, 0, np.pi]
    model = BicycleModel(x0)
    t = 20
    u = np.zeros((2, t))
    u[0, :] = 0.5
    u[1, :] = np.pi / 4

    model.sim(t, u)
    x, u = model.get_trajectory()
    print(x, u)
