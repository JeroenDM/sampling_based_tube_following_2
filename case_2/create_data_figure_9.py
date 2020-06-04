import time
import numpy as np

from acrobotics.path.sampling import SamplingSetting, SampleMethod, SearchStrategy
from acrobotics.planning.types import CostFuntionType, SolveMethod, PlanningSetup
from acrobotics.planning.settings import SolveMethod, OptSettings, SolverSettings
from acrobotics.planning.solver import solve

from definition import create_robot, create_path, create_scene

# =============================================================================
# Define some utilities to run the experiment and process the results.
# ==============================================================================
def create_settings_grid(iters, use_constraints_cost, constraints_cost_weight=1.0):
    s = SamplingSetting(
        search_strategy=SearchStrategy.GRID,
        iterations=iters,
        tolerance_reduction_factor=2.0,
        use_state_cost=use_constraints_cost,
        state_cost_weight=constraints_cost_weight,
    )
    s2 = SolverSettings(SolveMethod.sampling_based, CostFuntionType.sum_squared, s)
    return s2


def create_opt_settins(q_init, cow):

    s2 = SolverSettings(
        SolveMethod.optimization_based,
        CostFuntionType.sum_squared,
        opt_settings=OptSettings(
            q_init=q_init, max_iters=500, con_objective_weight=cow
        ),
    )
    return s2


def calc_mean_deviation(rxyz):
    """Calculate the mean deviation on the x and y rotation
    compared to the ideal value, wich is zero since tolerance
    is expressed in the local path frame."""

    rx = rxyz[:, 0]
    ry = rxyz[:, 1]
    return np.sum(np.abs(rx) + np.abs(ry))


def JVM(sol):
    """
    Calculate Joint Velocity Measure for a solution path.
    """
    qp = np.array(sol.joint_positions)
    return np.sum(np.diff(qp, axis=0) ** 2)


def Jcon(rxyz):
    """ Cacluate the value of the objective related to the path constraints. """
    return np.sum(rxyz[:, :2] ** 2)


def calc_tol_dev(robot, path, sol):
    """
    Calculate deviation of the welding angles from the nominal path point pose.
    """
    rxyz = np.zeros((N_PATH, 3))
    for i, qi, pt in zip(range(N_PATH), sol.joint_positions, path):
        tol_dev = pt.transform_to_rel_tolerance_deviation(robot.fk(qi))
        rxyz[i] = tol_dev[3:]
    return rxyz


# =============================================================================
# Run the simulations for the two planners, and different lambda values
# Warning: running the simulation can take a couple of minutes.
# ==============================================================================
N_PATH = 20
robot = create_robot()
scene, start, stop = create_scene(np.array([0.85, 0, 0]))
path = create_path(start, stop, N_PATH, 5, 5, 30)
setup = PlanningSetup(robot, path, scene)

lambda_values = [0.0, 1.0, 3.0, 10.0, 30.0, 100.0]

with open("case_2_sampling_based.csv", "a") as file:
    file.write("lambda,mean_dev\n")
    for w in lambda_values:
        s = create_settings_grid(1, True, w)
        sol = solve(setup, s)
        rxyz = calc_tol_dev(robot, path, sol)
        mean_dev = calc_mean_deviation(rxyz)
        file.write(f"{w},{mean_dev}\n")

# Use the home position as an initial guess for the next algorithm
# in general it is not always trivial to find a good intial guess
q_home = np.array([0, 1.5, 0, 0, 0, 0])
q_init = np.ones((N_PATH, 6)) * q_home

# I used a slightly different position for the planning scene by accident
# This does not influence the results much as we focus on the influence of
# lambda here.
scene, start, stop = create_scene(np.array([0.8, 0, 0]))
path = create_path(start, stop, N_PATH, 5, 5, 30)
setup = PlanningSetup(robot, path, scene)

with open("case_2_optimization_based.csv", "a") as file:
    file.write("cost,time,mean_dev,lambda,success\n")
    for w in lambda_values:
        start = time.time()
        try:
            sol2 = solve(setup, create_opt_settins(q_init, w))
            stop = time.time()

            rxyz = calc_tol_dev(robot, path, sol2)
            mean_dev = calc_mean_deviation(rxyz)

            file.write(f"{sol2.path_cost},{stop - start},{mean_dev},{w},1\n")
        except:
            stop = time.time()
            file.write(f"{np.nan},{stop - start},{np.nan},{w},0\n")

