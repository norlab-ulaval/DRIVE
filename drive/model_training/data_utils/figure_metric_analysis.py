# Import
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np

FILE_PATH_WARTHOG = "tests_figures/mean_heat_map_gma_warthog_metric.csv"
FILE_PATH_HUSKY = "tests_figures/mean_heat_map_gma_husky_metric.csv"
SQUARES_TO_ANALYZE_WARTHOG = [{'x': -0.5, 'y':3, 'width': 1, 'height': 1, 'axis': 'linear'},
                              {'x': 3.5, 'y':-1, 'width': 0.5, 'height': 2, 'axis': 'yaw'},]
SQUARES_TO_ANALYZE_HUSKY = [{'x': -0.25, 'y':0.75, 'width': 0.5, 'height': 0.25, 'axis': 'linear'},
                            {'x': 1.5, 'y':-0.25, 'width': 0.5, 'height': 0.5, 'axis': 'yaw'},]
COLORMAP_TERRAIN = {"asphalt":"grey", "ice":"blue","gravel":"orange","grass":"green","sand":"orangered","avide":"grey","avide2":"grey","mud":"darkgoldenrod","tile":"lightcoral"}
# Open the csv file as a dataframe
data_warthog = pd.read_csv(FILE_PATH_WARTHOG)
data_husky = pd.read_csv(FILE_PATH_HUSKY)
font = {'family' : 'normal',
        'weight' : 'bold',
        'size'   : 8}
plt.rc('font', **font)
plot_fs = 12
plt.rc('font', family='serif', serif='Times')
plt.rc('text', usetex=True)
plt.rc('xtick', labelsize=8)
plt.rc('ytick', labelsize=8)
plt.rc('axes', labelsize=8)
mpl.rcParams['lines.dashed_pattern'] = [2, 2]
mpl.rcParams['lines.linewidth'] = 1.0 


# Find every unique value in the 'terrain' column
terrains_warthog = data_warthog['terrain'].unique()
terrains_husky = data_husky['terrain'].unique()

columns_to_analyze = ['last_window_metric', 'last_window_cmd_total_energy_metric']

terrain_list = []
last_window_metric = []
last_window_cmd_total_energy_metric = []
list_window_metric_std = []
list_window_cmd_total_energy_metric_std = []
marker_list = []
for terrain in terrains_warthog:
    if terrain == 'gravel':
        continue
    # Find the rows that have the terrain value
    terrain_data = data_warthog[data_warthog['terrain'] == terrain]
    marker_list.append("o")
    marker_list.append("o")
    for square in SQUARES_TO_ANALYZE_WARTHOG:
        square_data = terrain_data[(abs(terrain_data['cmd_body_yaw_mean']) >= square['x']) & (abs(terrain_data['cmd_body_yaw_mean']) <= square['x'] + square['width']) & (abs(terrain_data['cmd_body_x_mean']) >= square['y']) & (abs(terrain_data['cmd_body_x_mean']) <= square['y'] + square['height'])]
        terrain_list.append(terrain)
        print(f"Square data shape: {square_data.shape}")
        for column in columns_to_analyze:
            print(f"Analyzing {column} for terrain {terrain} in square {square}")
            # Find the mean of the square
            median = square_data[column].median()
            print(f"Mean: {median}")
            std = square_data[column].std()
            print(f"STD: {std}")
            if column == 'last_window_metric':
                last_window_metric.append(median)
                list_window_metric_std.append(std)
            elif column == 'last_window_cmd_total_energy_metric':
                last_window_cmd_total_energy_metric.append(median)
                list_window_cmd_total_energy_metric_std.append(std)
                #if square['axis'] == 'linear':
                #    last_window_cmd_total_energy_metric.append(square_data['last_window_cmd_translationnal_energy_metric'].median())
                #    list_window_cmd_total_energy_metric_std.append(square_data['last_window_cmd_translationnal_energy_metric'].std())
                #elif square['axis'] == 'yaw':
                #    last_window_cmd_total_energy_metric.append(square_data['last_window_cmd_rotationnal_energy_metric'].median())
                #    list_window_cmd_total_energy_metric_std.append(square_data['last_window_cmd_rotationnal_energy_metric'].std())

for terrain in terrains_husky:
    #if terrain == 'gravel' or terrain == 'asphalt':
    #    continue
    # Find the rows that have the terrain value
    terrain_data = data_husky[data_husky['terrain'] == terrain]
    marker_list.append("o")
    marker_list.append("o")
    for square in SQUARES_TO_ANALYZE_HUSKY:
        square_data = terrain_data[(abs(terrain_data['cmd_body_yaw_mean']) >= square['x']) & (abs(terrain_data['cmd_body_yaw_mean']) <= square['x'] + square['width']) & (abs(terrain_data['cmd_body_x_mean']) >= square['y']) & (abs(terrain_data['cmd_body_x_mean']) <= square['y'] + square['height'])]
        terrain_list.append(terrain)
        for column in columns_to_analyze:
            print(f"Analyzing {column} for terrain {terrain} in square {square}")
            # Find the mean of the square
            median = square_data[column].median()
            print(f"Mean: {median}")
            std = square_data[column].std()
            print(f"STD: {std}")
            if column == 'last_window_metric':
                last_window_metric.append(median)
                list_window_metric_std.append(std)
            elif column == 'last_window_cmd_total_energy_metric':
                last_window_cmd_total_energy_metric.append(median)
                list_window_cmd_total_energy_metric_std.append(std)


# Plot the results with last_window_metric as x and last_window_cmd_total_energy_metric as y as circles with alpha the size of the std
for i in range(len(terrain_list)):
    if terrain_list[i] == 'grass' or terrain_list[i] == 'sand' or terrain_list[i] == 'gravel':
        continue
    plt.scatter(last_window_metric[i], last_window_cmd_total_energy_metric[i], color=COLORMAP_TERRAIN[terrain_list[i]], label=terrain_list[i], marker=marker_list[i], s=2)
    if i % 2:
        ellipse = mpl.patches.Ellipse((last_window_metric[i], last_window_cmd_total_energy_metric[i]), 4*list_window_metric_std[i], 4*list_window_cmd_total_energy_metric_std[i], 
                                      edgecolor=COLORMAP_TERRAIN[terrain_list[i]], facecolor=COLORMAP_TERRAIN[terrain_list[i]], alpha=0.2, linestyle='dashed')
    else:
        ellipse = mpl.patches.Ellipse((last_window_metric[i], last_window_cmd_total_energy_metric[i]), 4*list_window_metric_std[i], 4*list_window_cmd_total_energy_metric_std[i],
                                      edgecolor=COLORMAP_TERRAIN[terrain_list[i]], facecolor=COLORMAP_TERRAIN[terrain_list[i]], alpha=0.2)
    plt.gca().add_artist(ellipse)

# Build a custom legend at the bottom of the plot for the terrains
plt.hlines(0.5*25*500, 0, 1, colors='orange', linestyles='-', label='Warthog')
plt.hlines(0.5*1*75, 0, 1, colors='yellow', linestyles='-', label='Husky')
# Set the y axis as a log scale
plt.yscale('log')
# Merge the husky terrain list with the warthog terrain list
terrains = terrains_warthog.tolist() + terrains_husky.tolist()
# Get only the unique values
terrains = ["mud", "ice", "asphalt"]
legend_elements = [plt.Line2D([0], [0], marker='s', color='w', label=terrain[0].upper() + terrain[1:], markerfacecolor=COLORMAP_TERRAIN[terrain], markersize=10, alpha=0.2, markeredgecolor='black', markeredgewidth=1) for terrain in terrains]
# Append the max energy line to the legend
legend_elements.append(plt.Line2D([0], [0], color='orange', label='Warthog'))
legend_elements.append(plt.Line2D([0], [0], color='yellow', label='Husky'))
legend_elements.append(plt.Line2D([0], [0], color='black', linestyle='-', label='Forward'))
legend_elements.append(plt.Line2D([0], [0], color='black', linestyle='dashed', label='Turning'))
#plt.legend(handles=legend_elements, loc='best')
# Add the legend under the plot
plt.tight_layout(rect=[0.1, 0.2, 0.95, 0.95])  # Leave space at the bottom for the legend
plt.legend(handles=legend_elements, loc='lower center', bbox_to_anchor=(0.5, -0.37), ncol=3, prop={'size': 8})
plt.xlabel('Unpredictability')
plt.ylabel('Kinetic energy (J)')
plt.xlim(0, 1)
plt.ylim(0, 10000) #0.5*25*500*1.1
fig = plt.gcf()
fig.set_figwidth(88/25.4)
fig.set_figheight(4)
#plt.tight_layout()
#plt.show()
plt.savefig('tests_figures/last_window_metric_vs_last_window_cmd_total_energy_metric.pdf', format='pdf')