import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# ==================================================================================
# Load and process results
# ==================================================================================
# Read and process results from iterative grid refining.
# the 'sq' stand for sum squared, the objective function
df1 = pd.read_csv("data/case_1_iterative_sq.csv")

# Total number of samples per path point after iteration i,
# iteration numbers start from 0, hence the +1.
df1["total_samples"] = df1["samples"] * (df1["iteration"] + 1)


# The total number of samples is unique for each experiment
# that is repeated 10 times.
# Calculate average cost and time over 10 experiments
df1 = (
    df1.groupby(["total_samples", "iteration"])
    .agg({"cost": np.mean, "time": np.mean})
    .reset_index()
)

# The time is recorded for every iteration separately
# We need a cumulative sum to get the total time afters i iterations
df1["samples"] = (df1["total_samples"] / (df1["iteration"] + 1)).astype(int)
df1["total_time"] = df1.groupby(["samples"]).agg({"time": np.cumsum})

print(df1.columns)

# Read and process results from grid search.
df2 = pd.read_csv("data/case_1_grid_search_sq.csv")

# Every experiment for a given amount of samples is repeated 10 times,
# so take the average.
df2 = df2.groupby(["samples"]).agg({"cost": np.mean, "time": np.mean}).reset_index()
print(df2.columns)

# ==================================================================================
# Create plot and save it
# ==================================================================================
fig, ax = plt.subplots(figsize=(7, 5))

ax.plot(df2["time"], df2["cost"], "k-o")
ax.plot(df1["total_time"], df1["cost"], "o")


ax.set_xlabel(r"Time $[s]$", fontsize=18)
ax.set_ylabel(r"Joint Velocity Measure $[rad^2]$", fontsize=18)
ax.tick_params(labelsize=18)
ax.legend(["Algorithm 1: single run", "Algortihm 2: iterative"], fontsize=18)
plt.tight_layout()

plt.savefig("figures/case_1_time_vs_cost.png", dpi=200)

