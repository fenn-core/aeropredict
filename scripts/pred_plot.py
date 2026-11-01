import pandas as pd
import matplotlib.pyplot as plt

data = pd.read_csv(
    "data/control.csv", header=None, names=["Time", "V", "gamma", "theta", "q"]
)

fig, axes = plt.subplots(4, 1, figsize=(10, 10), sharex=True)

axes[0].plot(data["Time"], data["V"], color="blue", linewidth=1.5)
axes[0].set_ylabel("V [m/s]")
axes[0].set_title("Predicted Trajectory")

axes[1].plot(data["Time"], data["gamma"], color="red", linewidth=1.5)
axes[1].set_ylabel("γ [rad]")

axes[2].plot(data["Time"], data["theta"], color="green", linewidth=1.5)
axes[2].set_ylabel("θ [rad]")

axes[3].plot(data["Time"], data["q"], color="purple", linewidth=1.5)
axes[3].set_ylabel("q [rad/s]")
axes[3].set_xlabel("Time [s]")

for ax in axes:
    ax.grid(True, linestyle="--", alpha=0.5)

plt.tight_layout()
plt.show()
