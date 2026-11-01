import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

data = pd.read_csv("data/control.csv", header=None)

columns = ["Time", "V", "gamma", "theta", "q"]
if data.shape[1] >= 6:
    columns.append("delta_e")

data = data.iloc[:, : len(columns)]
data.columns = columns

theta_target = 0.06715404
delta_e_trim = -0.12391925

plots = [
    ("V", "V [m/s]", "blue"),
    ("gamma", "γ [rad]", "red"),
    ("theta", "θ [rad]", "green"),
    ("q", "q [rad/s]", "purple"),
]

if "delta_e" in data:
    plots.append(("delta_e", "δe [rad]", "orange"))

fig, axes = plt.subplots(len(plots), 1, figsize=(11, 2.5 * len(plots)), sharex=True)

for ax, (column, ylabel, color) in zip(axes, plots):
    ax.plot(data["Time"], data[column], color=color, linewidth=1.5)
    ax.set_ylabel(ylabel)
    ax.grid(True, linestyle="--", alpha=0.5)

axes[2].axhline(
    theta_target, color="black", linestyle="--", linewidth=1.2, label="Target pitch"
)
axes[2].legend()

if "delta_e" in data:
    axes[-1].axhline(delta_e_trim, color="gray", linestyle="--", label="Trim elevator")
    axes[-1].legend()

axes[-1].set_xlabel("Time [s]")
fig.suptitle("Aeropredict — Closed-Loop Response")

plt.tight_layout()
plt.show()
