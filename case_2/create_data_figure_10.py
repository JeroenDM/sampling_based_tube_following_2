import numpy as np

from acrobotics.path.sampling import SamplingSetting, SearchStrategy, SampleMethod
from acrobotics.planning.types import CostFuntionType, SolveMethod

from runners import run_experiments

# =============================================================================
# Compare grid search using a fixed uniform grid (SearchStrategy.GRID) and
# incremental deterministic samples
# (SearchStrategy.INCREMENTAL and SampleMethod.deterministic_uniform)
# with incremental sampling with a minimum number of collision free samples
# (SearchStrategy.MIN_INCREMENTAL)
# Warning: running the simulation can take a couple of minutes.
# ==============================================================================
param = {
    "n_path": 20,
    "search_strategy": [
        SearchStrategy.INCREMENTAL,
        SearchStrategy.MIN_INCREMENTAL,
        SearchStrategy.GRID,
    ],
    "iters": 1,
    "sample_method": SampleMethod.deterministic_uniform,
    "desired_num_samples": [10, 30, 50, 100, 150],
    "num_samples": [500, 900, 1200, 1500],
    "r_tol_samples": [[3, 3, 20], [4, 4, 30], [5, 5, 40], [5, 5, 90]],
}
run_experiments(param, "case_2_all_and_halton.csv")

# Add a run where we use random incremental sampling in grid search
# (SearchStrategy.INCREMENTAL and SampleMethod.random_uniform) to the above results.
# Since this planner has a random factor, results may vary slightly on each run.
param = {
    "n_path": 20,
    "search_strategy": [SearchStrategy.INCREMENTAL],
    "iters": 1,
    "sample_method": SampleMethod.random_uniform,
    "num_samples": [500, 900, 1200, 1500],
}
run_experiments(param, "case_2_random.csv")

# =============================================================================
# Run sampling- and optimization-based planner for different values of lambda,
# the weight assigned to the path constraints in the cost function.
# ==============================================================================

