import time
import pandas as pd
import matplotlib.pyplot as plt

# ==================================================================================
# Load and process results
# ==================================================================================
df1 = pd.read_csv("data/case_2_sampling_based.csv")
df2 = pd.read_csv("data/case_2_optimization_based.csv")

# ==================================================================================
# Create plot and save it
# ==================================================================================
fig, ax = plt.subplots(figsize=(7, 5))

ax.plot(df1["lambda"], df1["mean_dev"], "k-o")
ax.plot(df2["lambda"], df2["mean_dev"], "k--.")

ax.set_xlabel(r"Constraints objective weight $\lambda$ []", fontsize=18)
ax.set_ylabel(r"Tolerance deviation [$rad^2$]", fontsize=18)
ax.tick_params(labelsize=18)
ax.set_xlim([-1, 40])
ax.set_title("Sum Squared Tolerance Deviation", fontsize=18)
ax.legend(["Sampling-based, algorithm 1", "Optimization-based"], fontsize=16)
plt.tight_layout()

plt.savefig("figures/case_2_lambda_vs_mean_dev.png", dpi=200)
