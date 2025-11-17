import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

data = np.loadtxt("track.csv", delimiter=",")

x = data[:, 0]
y = data[:, 1]
yaw_deg = data[:, 2]

plt.figure(figsize=(6, 6))
plt.plot(x, y, linestyle='None', markersize=0.2, marker='o',color='tab:red')
plt.xlabel("x")
plt.ylabel("y")
plt.title("Trajectory from track.csv")
plt.axis("equal")
plt.minorticks_on()

plt.grid(which='major', linestyle='--', linewidth=0.4)
plt.grid(which='minor', linestyle=':', linewidth=0.2)

plt.gca().xaxis.set_major_locator(ticker.MaxNLocator(integer=False, nbins=20))
plt.gca().yaxis.set_major_locator(ticker.MaxNLocator(integer=False, nbins=20))

plt.show()
