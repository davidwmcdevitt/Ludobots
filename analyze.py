from matplotlib import pyplot as plt
import numpy as np

back = np.load("data/backLeg.npy")
front = np.load("data/frontLeg.npy")


plt.plot(back, label = "Back", linewidth = 5)
plt.plot(front, label = "Front")
plt.legend()
plt.show()