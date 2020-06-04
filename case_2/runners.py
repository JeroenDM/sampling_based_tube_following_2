import numpy as np
import pandas as pd


from acrobotics.planning.solver import solve
from acrobotics.planning.types import (
    Solution,
    CostFuntionType,
    PlanningSetup,
    SolveMethod,
)
from acrobotics.planning.settings import OptSettings, SolverSettings
from acrobotics.path.sampling import SampleMethod, SamplingSetting, SearchStrategy
from acrobotics.path.path_pt import TolEulerPt

from definition import create_robot, create_scene, create_path, show_path

NDOF = 6


def create_settings_min_incremental(desired_num_samples, iters, sample_method):
    s = SamplingSetting(
        search_strategy=SearchStrategy.MIN_INCREMENTAL,
        iterations=iters,
        sample_method=sample_method,
        num_samples=None,
        desired_num_samples=desired_num_samples,
        max_search_iters=int(10e4),
        tolerance_reduction_factor=2.0,
        use_state_cost=True,
        state_cost_weight=1.0,
    )
    s2 = SolverSettings(SolveMethod.sampling_based, CostFuntionType.sum_squared, s)
    return s2


def create_settings_incremental(num_samples, iters, sample_method):
    s = SamplingSetting(
        search_strategy=SearchStrategy.INCREMENTAL,
        iterations=iters,
        sample_method=sample_method,
        num_samples=num_samples,
        tolerance_reduction_factor=2.0,
        use_state_cost=True,
        state_cost_weight=1.0,
    )
    s2 = SolverSettings(SolveMethod.sampling_based, CostFuntionType.sum_squared, s)
    return s2


def create_settings_grid(iters):
    s = SamplingSetting(
        search_strategy=SearchStrategy.GRID,
        iterations=iters,
        tolerance_reduction_factor=2.0,
        use_state_cost=True,
        state_cost_weight=1.0,
    )
    s2 = SolverSettings(SolveMethod.sampling_based, CostFuntionType.sum_squared, s)
    return s2


def results_to_dict(settings: SamplingSetting, solution: Solution, path):
    data = {}
    data["search_strategy"] = settings.search_strategy.value
    data["iters"] = settings.iterations

    if settings.search_strategy == SearchStrategy.MIN_INCREMENTAL:
        data["desired_num_samples"] = settings.desired_num_samples
        data["sample_method"] = settings.sample_method.value
        data["num_samples"] = np.nan
    elif settings.search_strategy == SearchStrategy.INCREMENTAL:
        data["num_samples"] = settings.num_samples
        data["sample_method"] = settings.sample_method.value
        data["desired_num_samples"] = np.nan
    elif settings.search_strategy == SearchStrategy.GRID:
        pt_tol: TolEulerPt = path[0].rot_tol
        data["num_samples"] = (
            pt_tol[0].num_samples * pt_tol[1].num_samples * pt_tol[2].num_samples
        )
        data["sample_method"] = np.nan
        data["desired_num_samples"] = np.nan

    data["cost"] = solution.path_cost
    data["time"] = solution.run_time

    # add joint path to dict/csv
    n_path = len(solution.joint_positions)
    for j in range(n_path):
        for i in range(NDOF):
            data[f"q_{j}_{i}"] = solution.joint_positions[j][i]

    return data


def run_experiments(parameters, filename):
    robot = create_robot()
    scene, start, stop = create_scene(np.array([0.85, 0, 0]))

    # df = pd.DataFrame(
    #     columns=["search_strategy", "iters", "desired_num_samples", "cost", "time"]
    # )
    columns = [
        "search_strategy",
        "sample_method",
        "iters",
        "desired_num_samples",
        "num_samples",
        "cost",
        "time",
    ]

    # add joint path to dict/csv
    for j in range(parameters["n_path"]):
        for i in range(NDOF):
            columns.append(f"q_{j}_{i}")

    header = ",".join(columns) + "\n"

    with open(filename, "a") as f:
        f.write(header)

        for ss in parameters["search_strategy"]:
            if ss == SearchStrategy.GRID:
                for rtol in parameters["r_tol_samples"]:
                    path = create_path(
                        start, stop, parameters["n_path"], rtol[0], rtol[1], rtol[2]
                    )

                    setup = PlanningSetup(robot, path, scene)
                    s = create_settings_grid(parameters["iters"])

                    sol = solve(setup, s)
                    res = results_to_dict(s.sampling_settings, sol, path)
                    # df = df.append(res, ignore_index=True)
                    f.write(",".join([str(v) for v in res.values()]) + "\n")

            elif ss == SearchStrategy.MIN_INCREMENTAL:
                path = create_path(start, stop, parameters["n_path"])
                setup = PlanningSetup(robot, path, scene)
                for dns in parameters["desired_num_samples"]:

                    s = create_settings_min_incremental(
                        dns, parameters["iters"], parameters["sample_method"]
                    )

                    sol = solve(setup, s)
                    res = results_to_dict(s.sampling_settings, sol, path)
                    # df = df.append(res, ignore_index=True)
                    f.write(",".join([str(v) for v in res.values()]) + "\n")

            elif ss == SearchStrategy.INCREMENTAL:
                path = create_path(start, stop, parameters["n_path"])
                setup = PlanningSetup(robot, path, scene)
                for ns in parameters["num_samples"]:

                    s = create_settings_incremental(
                        ns, parameters["iters"], parameters["sample_method"]
                    )

                    sol = solve(setup, s)
                    res = results_to_dict(s.sampling_settings, sol, path)
                    # df = df.append(res, ignore_index=True)
                    f.write(",".join([str(v) for v in res.values()]) + "\n")

            else:
                raise NotImplementedError()

    return 0
