import pandas as pd 
import numpy as np
import matplotlib.pyplot as plt

path = "tests_figures/mean_heat_map_gma_warthog_metric.csv"

df = pd.read_csv(path)
print(df.columns)

print("_" * 20 + "Translation" + "_" * 20)
keep_translation_df = df.loc[np.abs(df['cmd_body_yaw_mean']) < 0.10]
print(keep_translation_df.last_window_metric.describe())
print("Percentile 5 ",np.percentile(keep_translation_df.last_window_metric, 5))

print("_" * 20 + "Rotation" + "_" * 20)
keep_rotation_df = df.loc[np.abs(df['cmd_body_x_mean']) < 0.10]
print(keep_rotation_df.last_window_metric.describe())
print("Percentile 5 ",np.percentile(keep_rotation_df.last_window_metric, 5))

keep_rotation_df.plot.hist(column='last_window_metric', bins=50, alpha=0.5, color='blue', label='Rotation')
median_rotation = keep_rotation_df.last_window_metric.median()
plt.axvline(median_rotation, color='blue', linestyle='dashed', linewidth=1)
keep_translation_df.plot.hist(column='last_window_metric', bins=50, alpha=0.5, color='red', label='Translation')
median_translation = keep_translation_df.last_window_metric.median()

print("median_rotation", median_rotation)
print("median_translation", median_translation)
plt.axvline(median_translation, color='red', linestyle='dashed', linewidth=1)
plt.legend()
plt.xlabel("last_window_metric")
plt.ylabel("Frequency")
plt.title("Histogram of last_window_metric for Rotation and Translation")
plt.show()