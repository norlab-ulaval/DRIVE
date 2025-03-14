import pandas as pd
import matplotlib.pyplot as plt

def compute_and_plot_metric(df):


    ref_area = 84.579219 
    df["diff_area_metric"] = ref_area- df["resulting_area"] 

    df["steady_state_speed_area_metric"] = df["resulting_area"] - 0

    df["1_over_steady_state_speed_area_metric"] = 1/df["resulting_area"]

    df["1_over_diff_area_metric"] = 1/(ref_area-df["resulting_area"])

    fig, axs = plt.subplots(4,1)
    fig.set_figheight(9)
    df.plot.bar(x= "terrain", y="diff_area_metric",ax =axs[0],rot=0)

    df.plot.bar(x= "terrain",y="steady_state_speed_area_metric",ax =axs[1],rot=0,color="red")

    df.plot.bar(x= "terrain",y="1_over_steady_state_speed_area_metric",ax =axs[2],rot=0,color="orange")

    df.plot.bar(x= "terrain",y= "1_over_diff_area_metric",ax =axs[3],rot=0,color="green")

    
    plt.subplots_adjust(hspace=0.5)
    plt.show()



if __name__ == "__main__":

    path = "/home/nicolassamson/ros2_ws/src/DRIVE/drive_datasets/results_multiple_terrain_dataframe/area_test/areas_saved.pkl"
    
    df = pd.read_pickle(path)

    

    df.drop(index=2,inplace=True)
    print(df)
    compute_and_plot_metric(df)
    print(df)
    print(4/0.3)

