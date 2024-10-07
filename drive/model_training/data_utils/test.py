import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
global frame_index   
# Create some sample data
x = np.linspace(0, 2 * np.pi, 100)

# Set up the figure and axis
fig, ax = plt.subplots()
line, = ax.plot(x, np.sin(x))

# Initialize frame index
frame_index = 0
max_frames = 50  # Define the number of frames

def update(frame):
    """Update the line for the current frame."""
    y = np.sin(x + frame * 0.1)  # Update data for the line
    line.set_ydata(y)  # Update line data
    return line,

def on_key(event, frame_index):
    """Handle key press events."""
    
    if event.key == 'space':
        frame_index = (frame_index + 1) % max_frames  # Cycle through frames
        update(frame_index)  # Update to the next frame
        plt.draw()  # Redraw the current figure

# Connect the key press event with a lambda function to pass the frame index
fig.canvas.mpl_connect('key_press_event', lambda event: on_key(event, frame_index))

# Set the title and labels
ax.set_title('Press Space to Change Frame')
ax.set_xlabel('X-axis')
ax.set_ylabel('Y-axis')
ax.set_ylim(-1.5, 1.5)  # Set y limits to avoid rescaling

# Display the initial frame
update(frame_index)
plt.show()