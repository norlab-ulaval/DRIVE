import matplotlib.pyplot as plt

# Sample data
x = [1, 2, 3, 4]
y_min = [0, 1, 2, 3]
y_max = [3, 4, 5, 6]

# Create vertical lines
lines = plt.vlines(x, y_min, y_max, colors='b')

# Display the plot
plt.title('Vertical Lines Example')
plt.show()

# Now, let's say you want to change the positions of the vertical lines
# You can access the data of the lines and update it
new_x = [20, 30, 40, 50]  # New x positions for the vertical lines
for line, new_x_pos in zip(lines.get_segments(), new_x):
    line[0][0] = new_x_pos  # Update start x position
    line[1][0] = new_x_pos  # Update end x position

# Redraw the lines with updated positions
plt.clf()  # Clear the previous plot
plt.vlines(new_x, y_min, y_max, colors='b')  # Re-draw the lines
plt.title('Updated Vertical Lines Example')
plt.show()
