import numpy as np 
import pandas as pd
from extractors import *
import matplotlib.pyplot as plt


import numpy as np
from matplotlib.colors import LinearSegmentedColormap



#N = 10000
## Generate 100 samples from a uniform distribution between -5 and 5
#X = np.random.uniform(-5, 5, N)
#Y = np.random.uniform(-5, 5, N)
#Z = X*Y
#
#
#plt.hist(Z,bins=100)
#plt.show()

PATH = "drive_datasets/results_multiple_terrain_dataframe/metric/warthog_metric_cmd_raw_slope_metric_scatter.csv"
df = pd.read_csv(PATH)
PATH_TO_SAVE = "figure/metric_danger_zone/trash"

print(df.columns)

def reshape_df(df,size_2=119):


    dico_results = {}
    for col in df.columns: 
        bad_total_energy_metric =df[col].to_numpy()
        shape = bad_total_energy_metric.shape
        new_shape = (shape[0]//size_2,size_2)
        total_energy_metric  = bad_total_energy_metric.reshape(new_shape)
        dico_results[col] = total_energy_metric

    return dico_results

def iter_scatter(df):

    




    
    dico_results = reshape_df(df,size_2=119)

    
    total_energy_metric= dico_results["y_coordinates_total_energy_metric"] 
    unpredictibility_metric = dico_results["cmd_metric_total_energy_metric"]
    
    terrain = dico_results["terrain"][:,0]
    
    color_dict = {"asphalt":"grey", "ice":"blue","gravel":"orange","grass":"green","sand":"darkgoldenrod","avide":"grey","avide2":"grey","mud":"darkgoldenrod","tile":"lightcoral"}
                

    max_y = 1/2*500*5**2
    min_y = 0
    
    for i in range(total_energy_metric.shape[0]):

        fig, ax = plt.subplots(1,1)
        # Define a colormap from white to blue
        white_to_blue = LinearSegmentedColormap.from_list("WhiteToBlue", ["pink",color_dict[terrain[i]]])

        colors = white_to_blue( np.linspace(0,1,119))
        print(color_dict[terrain[i]])
        ax.scatter(unpredictibility_metric[i,:],total_energy_metric[i,:],color=colors)
        #for j in range(119):
        #    ax.text(unpredictibility_metric[i,j],total_energy_metric[i,j],f"{j}")
        
        

        ax.set_ylabel("Danger zone [J]")
        ax.set_xlabel("Unpredictability [SI]")
        ax.set_ylim(min_y,max_y)
        ax.set_xlim(0,1)
        #plt.show()
        fig.savefig(PATH_TO_SAVE+f"/{i}_{terrain[i]}.png")
        plt.close("all")
        
#iter_scatter(df)



