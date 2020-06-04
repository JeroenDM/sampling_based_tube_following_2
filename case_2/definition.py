import numpy as np
import matplotlib.pyplot as plt

from acrolib.quaternion import Quaternion

from acrobotics.shapes import Box
from acrobotics.geometry import Scene
from acrobotics.robot_examples import Kuka
from acrobotics.tool_examples import torch3
from acrobotics.util import get_default_axes3d, plot_reference_frame
from acrobotics.util import rot_x, rot_z, translation

from acrobotics.path.tolerance import Tolerance, NoTolerance, SymmetricTolerance
from acrobotics.path.path_pt import TolEulerPt
from acrobotics.path.factory import create_line
from acrobotics.path.util import tf_inverse
from acrobotics.planning.types import Solution


def create_robot():
    robot = Kuka()
    robot.tool = torch3
    return robot


def create_scene(position):
    """ Welding case."""
    obs_h = 0.03
    obs_off = 0.04

    L = 0.5  # length of all stuff in y-dir
    obs_l = L / 6

    u_off = 0.005

    # U-profile
    # obstacle
    # table
    # side U-profile 1
    # side U-profile 2
    dims = [
        [0.27, L, 0.21],
        [0.2, obs_l, obs_h],
        [0.7, 0.7, 0.1],
        [0.2, 0.1, 0.055],
        [0.2, 0.1, 0.055],
    ]
    pos = [
        [dims[0][0] / 2, 0, dims[0][2] / 2],
        [-(dims[1][0] / 2) - obs_off, 0, obs_h / 2],
        [0, 0, -0.10],
        [-dims[3][0] / 2, L / 2 - dims[3][1] / 2, dims[3][2] / 2],
        [-dims[3][0] / 2, -L / 2 + dims[3][1] / 2, dims[3][2] / 2],
    ]

    pos = [p + position for p in pos]

    shapes = [Box(d[0], d[1], d[2]) for d in dims]
    tfs = [translation(p[0], p[1], p[2]) for p in pos]

    path_start = np.array([0, -L / 2 + dims[3][1] + u_off, 0]) + position
    path_stop = np.array([0, L / 2 - dims[3][1] - u_off, 0]) + position
    return Scene(shapes, tfs), path_start, path_stop


def create_path(start, stop, n_path, rx_samples=5, ry_samples=5, rz_samples=90):
    R_pt = rot_z(np.pi / 2) @ rot_x(3 * np.pi / 4)
    pos_tol = 3 * [NoTolerance()]
    rot_tol = [
        SymmetricTolerance(np.deg2rad(10), rx_samples),
        SymmetricTolerance(np.deg2rad(15), ry_samples),
        Tolerance(-np.pi, np.pi, rz_samples),
    ]

    start_pt = TolEulerPt(start, Quaternion(matrix=R_pt), pos_tol, rot_tol)
    return create_line(start_pt, stop, n_path)


def show_path(robot, path, scene, solution: Solution):
    fig2, ax2 = get_default_axes3d(
        xlim=[0, 1.5], ylim=[-0.75, 0.75], zlim=[-0.75, 0.75]
    )
    scene.plot(ax2, c="g")
    for pt in path:
        plot_reference_frame(ax2, pt.transformation_matrix, 0.1)

    if solution.success:
        robot.animate_path(fig2, ax2, solution.joint_path)
    else:
        print("Cannot show solution of a failed solver solution.")


if __name__ == "__main__":
    # show the planning setup
    robot = create_robot()
    scene, path_start, path_stop = create_scene(np.array([0.85, 0, 0]))
    path = create_path(path_start, path_stop, 20)

    _, ax = get_default_axes3d()
    robot.plot(ax, [0, 1.5, 0, 0, 0, 0], c="black")
    scene.plot(ax, c="green")
    for pt in path:
        plot_reference_frame(ax, tf=pt.transformation_matrix)
    plt.show()
