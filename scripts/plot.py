import pandas as pd
import matplotlib.pyplot as plt

# CSV files
files = [
    "data/data0.csv",
    "data/data1.csv",
    "data/data2.csv",
    "data/data3.csv",
    "data/data4.csv",
]

angles = [-4, -2, 0, 2, 4]

dt = 0.01

dfs = [pd.read_csv(file) for file in files]

for df in dfs:
    df["time"] = df.index * dt

names = ["V", "γ", "θ", "q"]


fig, axes = plt.subplots(2, 2, figsize=(14, 9))
axes = axes.flatten()


for i, column in enumerate(dfs[0].columns[1:5]):
    for j, df in enumerate(dfs):
        axes[i].plot(df["time"], df[column], linewidth=1.5, label=f"Angle {angles[j]}")

    axes[i].set_title(names[i])
    axes[i].set_xlabel("Time (s)")
    axes[i].set_ylabel("Value")
    axes[i].grid(True, alpha=0.3)
    axes[i].legend()

plt.tight_layout()
plt.show()
