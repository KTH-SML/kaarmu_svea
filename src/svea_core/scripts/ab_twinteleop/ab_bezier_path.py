"""

Path Planning with Bezier curve.

author: Atsushi Sakai(@Atsushi_twi)

COMMENT from Anton Bjoernberg (ab): This version is about 10 times faster than original by exchanging for loops
to numpy array operations
"""
from __future__ import division, print_function

import scipy.special
import numpy as np
import matplotlib.pyplot as plt
show_animation = True


def calc_4points_bezier_path(sx, sy, syaw, ex, ey, eyaw, offset, n_points = 100, quick = False):
    """
    Compute control points and path given start and end position.

    :param sx: (float) x-coordinate of the starting point
    :param sy: (float) y-coordinate of the starting point
    :param syaw: (float) yaw angle at start
    :param ex: (float) x-coordinate of the ending point
    :param ey: (float) y-coordinate of the ending point
    :param eyaw: (float) yaw angle at the end
    :param offset: (float)
    :return: (numpy array, numpy array)
    """
    dist = np.sqrt((sx - ex) ** 2 + (sy - ey) ** 2) / offset
    control_points = np.array(
        [[sx, sy],
         [sx + dist * np.cos(syaw), sy + dist * np.sin(syaw)],
         [ex - dist * np.cos(eyaw), ey - dist * np.sin(eyaw)],
         [ex, ey]])

    path = calc_bezier_path(control_points, n_points=n_points)
    path_d = None
    path_dd = None
    if not quick:
        c = bezier_derivatives_control_points(control_points, 2)
        control_points_d = c[1]
        control_points_dd = c[2]
        path_d = calc_bezier_path(control_points_d, n_points=n_points)
        path_dd = calc_bezier_path(control_points_dd, n_points=n_points)

    return path, path_d, path_dd

def calc_bezier_path_and_derivatives(control_points, n_points = 100, quick = False):
    """Compute a Bezier curve and its first and second order derivatives.

    :param control_points: List of control points (x,y) that defines the Bezier curve
    :type control_points: list of ndarray
    :param n_points: number of points to generate, defaults to 100
    :type n_points: int, optional
    :param quick: If true, derivatives are not computed for faster execution, defaults to False
    :type quick: bool, optional
    :return: The bezier curve and the derivatives as lists of points
    :rtype: (list of ndarray, list of ndarray, list of ndarray)
    """

    path = calc_bezier_path(control_points, n_points=n_points)
    if not quick:
        c = bezier_derivatives_control_points(control_points, 2)
        control_points_d = c[1]
        control_points_dd = c[2]
        path_d = calc_bezier_path(control_points_d, n_points=n_points)
        path_dd = calc_bezier_path(control_points_dd, n_points=n_points)
    else:
        path_d = None
        path_dd = None
    return path, path_d, path_dd


def calc_bezier_path(control_points, n_points=100):
    """
    Compute bezier path (trajectory) given control points.

    :param control_points: (numpy array)
    :param n_points: (int) number of points in the trajectory
    :return: (numpy array)
    """
    traj = []
    """
    for t in np.linspace(0, 1, n_points):
        traj.append(bezier(t, control_points))
    """
    t = np.linspace(0, 1, n_points)
    traj2 = bezier(t, control_points).T
    return np.array(traj2)


def bernstein_poly(n, i, t):
    """
    Bernstein polynom.

    :param n: (int) polynom degree
    :param i: (int)
    :param t: (float)
    :return: (float)
    """

    return scipy.special.comb(n, i) * t ** i * (1 - t) ** (n - i)


def bezier(t, control_points):
    """
    Return one point on the bezier curve.

    :param t: (float) number in [0, 1]
    :param control_points: (numpy array)
    :return: (numpy array) Coordinates of all points
    """
    n = len(control_points) - 1
    i = np.arange(n+1)
    ii, tt = np.meshgrid(i, t)
    bern = bernstein_poly(n, ii, tt)
    bc = np.dot(control_points.T, bern.T)   # By Anton
    return bc


def bezier_derivatives_control_points(control_points, n_derivatives):
    """
    Compute control points of the successive derivatives of a given bezier curve.

    A derivative of a bezier curve is a bezier curve.
    See https://pomax.github.io/bezierinfo/#derivatives
    for detailed explanations

    :param control_points: (numpy array)
    :param n_derivatives: (int)
    e.g., n_derivatives=2 -> compute control points for first and second derivatives
    :return: ([numpy array])
    """
    w = {0: control_points}
    for i in range(n_derivatives):
        n = len(w[i])
        w[i + 1] = np.array([(n - 1) * (w[i][j + 1] - w[i][j]) for j in range(n - 1)])
    return w


def curvature(dx, dy, ddx, ddy):
    """
    Compute curvature at one point given first and second derivatives.

    :param dx: (float) First derivative along x axis
    :param dy: (float)
    :param ddx: (float) Second derivative along x axis
    :param ddy: (float)
    :return: (float)
    """
    return (dx * ddy - dy * ddx) / (dx ** 2 + dy ** 2) ** (3 / 2)


def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):
    """Plot arrow."""
    if not isinstance(x, float):
        for (ix, iy, iyaw) in zip(x, y, yaw):
            plot_arrow(ix, iy, iyaw)
    else:
        plt.arrow(x, y, length * np.cos(yaw), length * np.sin(yaw),
                  fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)


def main():
    """Plot an example bezier curve."""
    start_x = 10.0  # [m]
    start_y = 1.0  # [m]
    start_yaw = np.radians(180.0)  # [rad]

    end_x = -0.0  # [m]
    end_y = -3.0  # [m]
    end_yaw = np.radians(-45.0)  # [rad]
    offset = 3.0

    #start_time = default_timer()
    path, control_points = calc_4points_bezier_path(
        start_x, start_y, start_yaw, end_x, end_y, end_yaw, offset)
    #end_time = default_timer()
    #print("ttime", end_time-start_time)

    # Note: alternatively, instead of specifying start and end position
    # you can directly define n control points and compute the path:
    # control_points = np.array([[5., 1.], [-2.78, 1.], [-11.5, -4.5], [-6., -8.]])
    # path = calc_bezier_path(control_points, n_points=100)

    # Display the tangent, normal and radius of cruvature at a given point
    t = 0.86  # Number in [0, 1]
    x_target, y_target = bezier(t, control_points)
    derivatives_cp = bezier_derivatives_control_points(control_points, 2)
    point = bezier(t, control_points)
    dt = bezier(t, derivatives_cp[1])
    ddt = bezier(t, derivatives_cp[2])
    # Radius of curvature
    radius = 1 / curvature(dt[0], dt[1], ddt[0], ddt[1])
    # Normalize derivative
    dt /= np.linalg.norm(dt, 2)
    tangent = np.array([point, point + dt])
    normal = np.array([point, point + [- dt[1], dt[0]]])
    curvature_center = point + np.array([- dt[1], dt[0]]) * radius
    circle = plt.Circle(tuple(curvature_center), radius, color=(0, 0.8, 0.8), fill=False, linewidth=1)

    assert path.T[0][0] == start_x, "path is invalid"
    assert path.T[1][0] == start_y, "path is invalid"
    assert path.T[0][-1] == end_x, "path is invalid"
    assert path.T[1][-1] == end_y, "path is invalid"

    if show_animation:
        fig, ax = plt.subplots()
        ax.plot(path.T[0], path.T[1], label="Bezier Path")
        ax.plot(control_points.T[0], control_points.T[1], '--o', label="Control Points")
        ax.plot(x_target, y_target)
        ax.plot(tangent[:, 0], tangent[:, 1], label="Tangent")
        ax.plot(normal[:, 0], normal[:, 1], label="Normal")
        ax.add_artist(circle)
        plot_arrow(start_x, start_y, start_yaw)
        plot_arrow(end_x, end_y, end_yaw)
        ax.legend()
        ax.axis("equal")
        ax.grid(True)
        plt.show()


def main2():
    """Show the effect of the offset."""
    start_x = 10.0  # [m]
    start_y = 1.0  # [m]
    start_yaw = np.radians(180.0)  # [rad]

    end_x = -0.0  # [m]
    end_y = -3.0  # [m]
    end_yaw = np.radians(-45.0)  # [rad]

    for offset in np.arange(1.0, 5.0, 1.0):
        path, control_points = calc_4points_bezier_path(
            start_x, start_y, start_yaw, end_x, end_y, end_yaw, offset)
        assert path.T[0][0] == start_x, "path is invalid"
        assert path.T[1][0] == start_y, "path is invalid"
        assert path.T[0][-1] == end_x, "path is invalid"
        assert path.T[1][-1] == end_y, "path is invalid"

        if show_animation:
            plt.plot(path.T[0], path.T[1], label="Offset=" + str(offset))

    if show_animation:
        plot_arrow(start_x, start_y, start_yaw)
        plot_arrow(end_x, end_y, end_yaw)
        plt.legend()
        plt.axis("equal")
        plt.grid(True)
        plt.show()


if __name__ == '__main__':
    main()
    #  main2()
