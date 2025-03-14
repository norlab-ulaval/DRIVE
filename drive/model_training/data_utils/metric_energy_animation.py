import matplotlib.pyplot as plt
import numpy as np

from matplotlib.widgets import Button, Slider
from drive.model_training.data_utils.metric_energy import Dataset2Evaluate, SlopeMetric
# Load dataset 

dataset_name = "drive_dataset_warthog"
dataset = Dataset2Evaluate(dataset_name)
metric_computer = SlopeMetric("SlopeMetric","warthog")
terrain = "grass"

metric_to_observed = "total_energy_metric" # 'total_energy_metric', 'rotationnal_energy_metric', 'translationnal_energy_metric'

# The parametrized function to be plotted
def f(t, length, basewidth):

    value = metric_computer.combpute_for_var_basewidth(basewidth,length, dataset,terrain)
    ax.set_title(f"Total Metric of the {dataset_name} on {terrain}")
    return  value[metric_to_observed]

t = np.linspace(0, 1, 1000)

# Define initial parameters
init_length= metric_computer.length  #along x 
init_width = metric_computer.basewidth #along y

# Create the figure and the line that we will manipulate
fig, ax = plt.subplots()
box = ax.boxplot(f(t, init_length, init_width))
ax.set_xlabel('Time [s]')

# adjust the main plot to make room for the sliders
fig.subplots_adjust(left=0.25, bottom=0.25)

# Make a horizontal slider to control the frequency.
axfreq = fig.add_axes([0.25, 0.1, 0.65, 0.03])
basewidth_slider = Slider(
    ax=axfreq,
    label='Basewidth [m]',
    valmin=0.01,
    valmax=10,
    valinit=init_width,
)

# Make a vertically oriented slider to control the amplitude
axamp = fig.add_axes([0.1, 0.25, 0.0225, 0.63])
length_slider = Slider(
    ax=axamp,
    label="Length [m]",
    valmin=0.01,
    valmax=10,
    valinit=init_length,
    orientation="vertical"
)


# The function to be called anytime a slider's value changes
def update(val):
    ax.clear()
    ax.set_ylim(0,1)

    y = f(t, length_slider.val, basewidth_slider.val)
    
    box = ax.boxplot(y)
    
    fig.canvas.draw_idle()


# register the update function with each slider
basewidth_slider.on_changed(update)
length_slider.on_changed(update)

# Create a `matplotlib.widgets.Button` to reset the sliders to initial values.
resetax = fig.add_axes([0.8, 0.025, 0.1, 0.04])
button = Button(resetax, 'Reset', hovercolor='0.975')


def reset(event):
    basewidth_slider.reset()
    length_slider.reset()
button.on_clicked(reset)

plt.show()