import numpy as np 
import pandas as pd 


path = "drive_datasets/results_multiple_terrain_dataframe/all_terrain_steady_state_dataset.pkl"

pd.read_pickle(path)
df = pd.read_pickle(path)
print(df.loc[df["terrain"] == "sand"].shape)

