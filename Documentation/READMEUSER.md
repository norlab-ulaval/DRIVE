# To run the code

python -m drive.model_training.data_utils.graph_module



The code that creates figure 3 is metric_energy_boxplot.py. the figure 3 is saved under figure/fig_metric_boxplot.pdf 

The code that creates figure 8 is the code drive/model_training/data_utils/slip_boxplot.py



# For the submission 

The article contains 14 figures. Here is a mapping between the figure and the code. 

Figure 1. Article of Dom 
Figure 2. Article. 
Figure 3-4. ./figure/inkscape_figure.svg
Figure 5. Dom's article.
Figure 6. IDK 
Figure 7. 
Figure 8. .drive/model_training/data_utils/slip_boxplot.py
Figure 9 .drive/model_training/data_utils/figure_accel_hist_space_analysis.py


Dataset used by figure: 
**Figure 7 :**
dataset: Dataset2
filter: (All the data with a filter 5 m/s and 4 rad/s)  
file: drive/model_training/data_utils/figure_accel_hist_space_analysis.py
data_file:"drive_datasets/results_multiple_terrain_dataframe/Use_to_scatter_filtered_cleared_path_warthog_following_robot_param_all_terrain_steady_state_dataset.pkl"
fct: plot_figure_7

**Figure 8 :** 
dataset: Dataset2
filter: (All the data with a filter 4 m/s and 4 rad/s)  
file: drive/model_training/data_utils/slip_boxplot.py
data_file: "drive_datasets/results_multiple_terrain_dataframe_copy_backup/filtered_cleared_path_warthog_following_robot_param_all_terrain_steady_state_dataset.pkl" 
fct: slip_boxplot_both_robot_slip_angle_added.py -> 

**Figure 9 :** 
**dataset**: Dataset2
**filter**: (All the data with a filter 4 m/s and 4 rad/s)  
**file**: drive/model_training/data_utils/graph_module_heat_map_gma.py
**data_file**:"drive_datasets/results_multiple_terrain_dataframe_copy_backup/filtered_cleared_path_warthog_following_robot_param_all_terrain_steady_state_dataset.pkl"
**fct**: plot_heat_map_gaussian_moving_average
**file_save**: tests_figures/mean_heat_map_gma_warthog.pdf

![alt text](image-3.png)


**Figure 10 :** 
**dataset**: Dataset2 
**filter**: (4 m/s 4rad/s)
**file**: drive/model_training/data_utils/metric_energy_boxplot.py
**data_file**:"drive_datasets/results_multiple_terrain_dataframe_copy_backup/metric/warthog_metric_cmd_raw_slope_metric.csv"
**fct**: boxplot_all_terrain_husky_warthog_robot(df_concat,robot_list=["husky","warthog"])
**file_saved**: figure/fig_metric_boxplot.pdf

![alt text](image-4.png)



**Figure 11 :** 
**dataset**: Dataset2
**filter**: (4 m/s 4rad/s)
**file**: drive/model_training/data_utils/graph_module_heat_map_gma_metric.py
**data_file**:"drive_datasets/results_multiple_terrain_dataframe/metric/warthog_metric_to_watermelon.csv"
**fct**: plot_heat_map_gaussian_moving_average(path, path_to_geom, cline, proportionnal, nbr_of_samples_to_consider=None)
**file_saved**: tests_figures/mean_heat_map_gma_warthog_metric.pdf

![alt text](image-5.png)




**Figure 12 :** 
figure/fig_lesson_learned.svg


**Figure 13 :** 
**dataset**: Dataset1
**filter**: Original sampling for sand 
**file**: drive/model_training/data_utils/figure_path_analysis.py
**data_file**:  "drive_datasets/results_multiple_terrain_dataframe/all_terrain_steady_state_dataset.pkl"
**fct**: create_figure(df_all_terrain, range_limit = (-RANGE_LIMIT, RANGE_LIMIT), absolute=HEATMAP_ABSOLUTE)
**file_saved**: tests_figures/sand/fig_path_analysis.pdf

![alt text](image-6.png)


**Figure 14 :** 
dataset: Dataset2
filter: (All the data with a filter 5 m/s and 4 rad/s)  
file: drive/model_training/data_utils/figure_accel_hist_space_analysis.py
data_file:"drive_datasets/results_multiple_terrain_dataframe/Use_to_scatter_filtered_cleared_path_warthog_following_robot_param_all_terrain_steady_state_dataset.pkl"
fct: plot_figure_9




**Figure 9 :** 
**dataset**: 
**filter**: 
**file**: 
**data_file**:
**fct**: 


DATASET 1: 

![alt text](image.png)

Dataset 2:

![alt text](image-2.png)

The main difference between the Dataset 1 and 2 is the presence of 3 dataset: 
Is that Dataset 1 contains two additionnal grass dataset that were subsampling a specific area. It contains also 1 dataset for asphalt that subsample small velocities. 

