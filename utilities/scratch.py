import matplotlib.pyplot as plt
import numpy as np

LONG = 1.4224
SHORT = 1.0668

list = np.array([(0, 0)])
list = np.vstack((list, (LONG, 0)))
list = np.vstack((list, (LONG, SHORT)))
list = np.vstack((list, (0, SHORT)))


print(np.shape(list))

plt.plot(list[:, 0], list[:, 1])
plt.grid()
plt.xlabel("X (meters)")
plt.ylabel("Y (meters)")
plt.title("Robot Position in World Frame")
plt.savefig("rectangle_output.png")
