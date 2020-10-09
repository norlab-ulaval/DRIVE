import numpy as np
import matplotlib.pyplot as plt


max_it = 60
x = []
y = []
z = []

for i in range(max_it):
    x.append(i)
    y.append(3 * np.sin(i * 2 * np.pi / max_it))
    z.append((3 * 2 / np.pi) * np.arcsin(np.sin(2 * np.pi * i / max_it)))

plt.scatter(x, y)
plt.scatter(x, z)

plt.show()