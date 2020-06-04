import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# https://gist.github.com/thriveth/8560036
colors = [
    "#377eb8",
    "#ff7f00",
    "#4daf4a",
    "#f781bf",
    "#a65628",
    "#984ea3",
    "#999999",
    "#e41a1c",
    "#dede00",
]

# ==================================================================================
# Load and process results
# ==================================================================================
df_random = pd.read_csv("data/case_2_random.csv")
df_all = pd.read_csv("data/case_2_all_and_halton.csv")

grid = df_all.loc[df_all["search_strategy"] == 0]
inc_df = df_all.loc[df_all["search_strategy"] == 1]
min_inc_df = df_all.loc[df_all["search_strategy"] == 2]

# ==================================================================================
# Create plot and save it
# ==================================================================================
fig, ax = plt.subplots(figsize=(7, 5))

ax.plot(grid["time"], grid["cost"], "o", color=colors[0], markersize=10)
ax.plot(inc_df["time"], inc_df["cost"], "^", color=colors[1], markersize=10)
ax.plot(df_random["time"], df_random["cost"], "d", color=colors[2], markersize=10)
ax.plot(min_inc_df["time"], min_inc_df["cost"], "X", color=colors[5], markersize=10)

ax.set_xlabel(r"Time $[s]$", fontsize="18")
ax.set_ylabel(r"Objective: $\mathrm{J}_{path} + \mathrm{J}_{con}$", fontsize="18")
ax.tick_params(labelsize=18)
ax.set_ylim([-1, 10])
ax.legend(
    [
        "Algorithm 1: Uniform grid",
        "Algorithm 1: Halton sampling",
        "Algorithm 1: Random sampling",
        "Algorithm 3",
    ],
    fontsize=16,
)
ax.set_title("Run time vs objective value", fontsize=18)
plt.tight_layout()

plt.savefig("figures/case_2_time_vs_cost.png", dpi=200)

