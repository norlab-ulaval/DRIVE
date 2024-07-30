import yaml 
import pandas as pd





df = pd.read_pickle("/home/nicolassamson/ros2_ws/src/DRIVE/calib_data/warthog_wheels_grass_2024_7_29_14h46s15/model_training_datasets/raw_dataframe.pkl")
print(df.head(5))

print(df.iloc[0:2])