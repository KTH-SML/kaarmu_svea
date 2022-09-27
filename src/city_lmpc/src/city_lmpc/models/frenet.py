# system path management (to import from adjacent directories)
import sys
from os import path

from city_lmpc.models.generic import GenericModel
from casadi import horzcat, vertcat
import numpy as np

# Example of how to use the GenericModel


class FrenetModel(GenericModel):
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
        s = self.dae.add_x("s")
        e = self.dae.add_x("e")
        v = self.dae.add_x("v")
        phi = self.dae.add_x("phi")

        # add inputs
        v_u = self.dae.add_u("v_u")
        delta = self.dae.add_u("delta")

        # add parameters (count towards inputs)
        kappa = self.dae.add_p("kappa")
        s_0_arc = self.dae.add_p("s_0_arc")
        phi_0_arc = self.dae.add_p("phi_0_arc")

        # differential equations of model
        phi_s = kappa * (s - s_0_arc) + phi_0_arc
        # phi_s = np.arctan2(np.sin(phi_s), np.cos(phi_s))
        # phi_con = np.arctan2(np.sin(phi), np.cos(phi))
        dphi = phi - phi_s
        # dphi = np.arctan2(np.sin(dphi), np.cos(dphi))

        # dphi = np.pi - norm_2(norm_2(phi_con - phi_s) - np.pi)

        s_dot = v * np.cos(dphi) / (1 - kappa * e)
        e_dot = v * np.sin(dphi)
        v_dot = (v_u - v) / self.TAU
        phi_dot = v * np.tan(delta) / self.WB

        self.dae.add_ode("s_dot", s_dot)
        self.dae.add_ode("e_dot", e_dot)
        self.dae.add_ode("v_dot", v_dot)
        self.dae.add_ode("phi_dot", phi_dot)

        # set units
        self.dae.set_unit("s", "m")
        self.dae.set_unit("e", "m")
        self.dae.set_unit("v", "m/s")
        self.dae.set_unit("phi", "rad")


if __name__ == "__main__":
    x0 = [0, 0, 0, np.pi]
    model = FrenetModel(x0)
    t = 20
    u = np.zeros((4, t))
    u[0, :] = 0.5
    u[1, :] = np.pi / 4

    model.sim(t, u)
    x, u = model.get_trajectory()
