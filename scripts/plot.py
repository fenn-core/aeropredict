import pandas as pd
import matplotlib.pyplot as plt

csv_file = "data/data.csv"
df = pd.read_csv(csv_file)


fig, axes = plt.subplots(2, 2, figsize=(12, 8))
axes = axes.flatten()


names = ["V", "γ", "θ", "q"]

for i, column in enumerate(df.columns[:4]):
    axes[i].plot(df[column], linewidth=1.5)
    axes[i].set_title(names[i])
    axes[i].set_xlabel("Sample")
    axes[i].set_ylabel("Value")
    axes[i].grid(True, alpha=0.3)


plt.tight_layout()

plt.show()
