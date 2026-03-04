import math

import matplotlib.pyplot as plt
import numpy as np

# Load CSV-like data
data = np.loadtxt("out", delimiter=",")

x = data[:, 0]
y = data[:, 1]
z = data[:, 2]

plt.figure()
plt.title("Projection Uncertainty (in pixels), looking out to infinity")

print(f"Mean={np.mean(z)}")

# Create contour plot with 1px increments
levels = np.arange(0, math.ceil(np.max(z)), 0.1)  # 0, 1, 2, ..., 10
contour = plt.tricontour(x, y, z, levels=levels, colors="black", linewidths=0.5)
contourf = plt.tricontourf(x, y, z, levels=levels, cmap="viridis")
plt.clabel(contour, inline=True, fontsize=8, fmt="%0.1f px")
plt.colorbar(contourf, label="Uncertainty (px)")

plt.xlabel("x")
plt.ylabel("y")
plt.gca().invert_yaxis()
plt.axis("equal")
plt.tight_layout()

# plt.show()
plt.savefig("heatmap.svg")
