import pandas as pd 


df_warthog = pd.read_csv("drive_datasets/results_multiple_terrain_dataframe/metric/results_slope_metric_warthog.csv")

df_husky = pd.read_csv("drive_datasets/results_multiple_terrain_dataframe/metric/results_slope_metric_husky.csv")


def compute_metric(df, terrain1="grass",terrain2="asphalt"):



    asphalt_results = df.loc[df.terrain == terrain2][["metric_rotationnal_energy_metric","metric_translationnal_energy_metric",
                        "metric_total_energy_metric"]].to_numpy()
    grass_results = df.loc[df.terrain == terrain1][["metric_rotationnal_energy_metric","metric_translationnal_energy_metric",
                        "metric_total_energy_metric"]].to_numpy()



    difference_husky = (asphalt_results - grass_results)/grass_results *100

    print("metric_rotationnal_energy_metric  ","  metric_translationnal_energy_metric",
                        "  metric_total_energy_metric")
    
    print(difference_husky)

print(df_warthog.columns)


df_husky_only = df_husky[["metric_rotationnal_energy_metric","terrain","metric_translationnal_energy_metric",
                        "metric_total_energy_metric"]]

df_warthog_only = df_warthog[["metric_rotationnal_energy_metric","terrain","metric_translationnal_energy_metric",
                        "metric_total_energy_metric"]]

print("husky")
compute_metric(df_husky_only, terrain1="grass",terrain2="asphalt")

print("warthog")
compute_metric(df_warthog_only, terrain1="grass",terrain2="asphalt")

