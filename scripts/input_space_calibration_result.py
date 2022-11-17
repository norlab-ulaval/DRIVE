import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
plt.rcParams.update({
    "text.usetex": True,
    "font.family": "sans-serif",
    "font.sans-serif": "Helvetica",
})
plt.rc('axes', titlesize=8)
plt.rc('axes', labelsize=10)
plt.rc('xtick', labelsize=10)
plt.rc('ytick', labelsize=10)

fig, ax = plt.subplots(1, 1)
fig.set_figheight(10)
fig.set_figwidth(10)

ax.spines['left'].set_position('zero')
ax.spines['right'].set_color('none')
ax.spines['bottom'].set_position('zero')
ax.spines['top'].set_color('none')

x_min = -5
x_max = 5
y_min = -2
y_max = 2

ax.set_xlim(x_min, x_max)
ax.set_xticks([x_min, x_max])
ax.text(x_min + x_min/10, x_min/10, 'Angular velocity [rad/s]', rotation='vertical')
ax.set_ylim(y_min, y_max)
ax.set_yticks([y_min, y_max])
ax.text(y_min/2, y_max + y_max/10, 'Linear velocity [m/s]', rotation='horizontal')


marmotte_input_space_df = pd.read_pickle('../calib_data/marmotte/input_space_data.pkl')

calibrated_radius = marmotte_input_space_df['calibrated_radius [m]']
calibrated_baseline = marmotte_input_space_df['calibrated baseline [m]']
minimum_linear_vel_positive = marmotte_input_space_df['minimum_linear_vel_positive [m/s]']
minimum_linear_vel_negative = marmotte_input_space_df['minimum_linear_vel_negative [m/s]']
minimum_angular_vel_positive = marmotte_input_space_df['minimum_angular_vel_positive [rad/s]']
minimum_angular_vel_negative = marmotte_input_space_df['minimum_angular_vel_negative [rad/s]']
maximum_linear_vel_positive = marmotte_input_space_df['maximum_linear_vel_positive [m/s]']
maximum_linear_vel_negative = marmotte_input_space_df['maximum_linear_vel_negative [m/s]']
maximum_angular_vel_positive = marmotte_input_space_df['maximum_angular_vel_positive [rad/s]']
maximum_angular_vel_negative = marmotte_input_space_df['maximum_angular_vel_negative [rad/s]']

num_points = 100
line_width = 5
alpha = 0.2
cmd_angular_vel_linspace = np.linspace(x_min, x_max, num_points)
cmd_linear_max_vel_linspace = np.linspace(y_max, y_max, num_points)
cmd_linear_min_vel_linspace = np.linspace(y_min, y_min, num_points)

## plot initial input space
ax.plot(cmd_angular_vel_linspace, cmd_linear_min_vel_linspace, color='C0', lw=line_width, label='Uncharacterized')
ax.plot(cmd_angular_vel_linspace, cmd_linear_max_vel_linspace, color='C0', lw=line_width)
ax.vlines(x_min, y_min, y_max, color='C0', lw=line_width)
ax.vlines(x_max, y_min, y_max, color='C0', lw=line_width)
ax.fill_between(cmd_angular_vel_linspace, cmd_linear_max_vel_linspace, y2=cmd_linear_min_vel_linspace, alpha=alpha, color='C0')

## plot characterized input space
char_angular_vel_linspace_negative = np.linspace(maximum_angular_vel_negative, 0, int(num_points / 2)).flatten()
char_angular_vel_linspace_positive = np.linspace(0, maximum_angular_vel_positive, int(num_points / 2)).flatten()
char_q1_vel_linspace = np.linspace(0, maximum_linear_vel_positive, int(num_points / 2)).flatten()
char_q2_vel_linspace = np.linspace(maximum_linear_vel_positive, 0, int(num_points / 2)).flatten()
char_q3_vel_linspace = np.linspace(maximum_linear_vel_negative, 0, int(num_points / 2)).flatten()
char_q4_vel_linspace = np.linspace(0, maximum_linear_vel_negative, int(num_points / 2)).flatten()

q1_char_input_space = ax.plot(char_angular_vel_linspace_negative, char_q1_vel_linspace, color='C1', lw=line_width, label='Characterized')
q2_char_input_space = ax.plot(char_angular_vel_linspace_positive, char_q2_vel_linspace, color='C1', lw=line_width)
q3_char_input_space = ax.plot(char_angular_vel_linspace_positive, char_q3_vel_linspace, color='C1', lw=line_width)
q4_char_input_space = ax.plot(char_angular_vel_linspace_negative, char_q4_vel_linspace, color='C1', lw=line_width)

ax.fill_between(char_angular_vel_linspace_negative, char_q1_vel_linspace, y2=char_q4_vel_linspace, alpha=alpha, color='C1')
ax.fill_between(char_angular_vel_linspace_positive, char_q2_vel_linspace, y2=char_q3_vel_linspace, alpha=alpha, color='C1')

ax.legend()

plt.savefig('figs/input_space_calib.png')

plt.show()