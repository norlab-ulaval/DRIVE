import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Create a grid of points
Y, X = np.mgrid[-3:3:100j, -3:3:100j]
U = -1 - X**2 + Y
V = 1 + X - Y**2

# Create the figure and axis
fig, ax = plt.subplots()
quiver = ax.quiver(X, Y, U, V)

# Set limits and labels
ax.set_xlim(-3, 3)
ax.set_ylim(-3, 3)
ax.set_title('Animated Quiver Plot')

# Update function for the animation
def update(frame):
    # Update the U and V arrays to change the direction or magnitude of the vectors
    U = -1 - X**2 + (Y + frame / 10.0)  # Animate Y component
    V = 1 + X - (Y**2 + frame / 10.0)  # Animate X component
    quiver.set_UVC(U, V)  # Update the quiver with new U and V
    return quiver,

# Create the animation
ani = FuncAnimation(fig, update, frames=np.arange(0, 100), blit=True)

plt.show()