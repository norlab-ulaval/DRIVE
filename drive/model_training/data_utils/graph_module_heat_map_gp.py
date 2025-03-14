import matplotlib.pyplot as plt
import numpy as np 
import pandas as pd
from drive.model_training.data_utils.extractors import *
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, Product, ConstantKernel
import matplotlib.cm as cm 
import pickle 
import shapely
import pathlib
import argparse

DATASET_PICKLE = "./drive_datasets/results_multiple_terrain_dataframe/filtered_cleared_path_warthog_max_lin_speed_all_speed_all_terrain_steady_state_dataset.pkl"
GEOM_PICKLE = "./drive_datasets/results_multiple_terrain_dataframe/geom_limits_by_terrain_for_filtered_cleared_path_warthog_max_lin_speed_all_speed_all_terrain_steady_state_dataset.pkl"

def filtered_mesgrid_cmd_based_on_geom(geom_by_terrain,terrain,x_mesh,y_mesh ):

    geom = geom_by_terrain[terrain]

    list_fitler = []

    x_ravel = np.ravel(x_mesh)
    y_ravel = np.ravel(y_mesh)

    for x,y in zip(x_ravel,y_ravel):
        pt = shapely.geometry.Point(x,y)
        if shapely.within(pt,geom):
            list_fitler.append(True)
        else:
            list_fitler.append(False)

    filter = np.array(list_fitler).reshape(x_mesh.shape)

    return filter


def find_the_surface(df,col_interest,terrain,geom_to_filter ={},ax=None,debug=False,norm= {"normalize":False},cline=False,colormap="seismic",to_plot="mean",col_x_y = ["cmd_right_wheels","cmd_left_wheels"],x_lim=(-6,6),y_lim=(-6,6),alpha = 0.7):
    
    # Extract the values 
    vx = np.ravel(column_type_extractor(df,col_x_y[1])) # y
    vyaw = np.ravel(column_type_extractor(df,col_x_y[0])) # x

    y = np.ravel(column_type_extractor(df,col_interest))
    X = np.array((vyaw,vx)).T


    
    # Normalize the mean_prediction for color mapping
    

    X_train = X
    y_train = y # /norm
    
    

    # Lets pick the kernel 
    kernel = 1 * RBF(length_scale=1.0, length_scale_bounds=(0.001, 100000))
    #print(kernel)
    # Now let's train the Kernel 
    gaussian_process = GaussianProcessRegressor(kernel=kernel, n_restarts_optimizer=15,alpha=alpha)
    gaussian_process.fit(X_train, y_train)
    print(gaussian_process.kernel_)

    # Predict the grid 


    # Define the ranges for x and y
    x_lim_to_plot = np.linspace(x_lim[0], x_lim[1], 100)  # 100 points from -5 to 5
    y_lim_to_plot = np.linspace(y_lim[0], y_lim[1], 100)  # 100 points from -5 to 5

    # Create the meshgrid
    X_2do, Y_2do = np.meshgrid(x_lim_to_plot, y_lim_to_plot)

    if geom_to_filter != {}:
        filter = filtered_mesgrid_cmd_based_on_geom(geom_to_filter,terrain,X_2do,Y_2do )
    else:
        filter = {} 

    x_2_eval = np.array((np.ravel(X_2do),np.ravel(Y_2do))).T

    
    mean_prediction, std_prediction = gaussian_process.predict(x_2_eval, return_std=True)

    if to_plot == "mean":
        
        data_to_plot = mean_prediction
    if to_plot == "std":
        data_to_plot = std_prediction 
    if debug:
        #graph_scatter_valid(vx,vyaw,X,y)
        im = plot_image(ax,X_train,data_to_plot,col_interest,y,terrain,x_2_eval,
                    normalize=norm,cline=cline,filter=filter,shape=X_2do.shape,
                    colormap=colormap,x_lim=x_lim,y_lim=y_lim)

    return im


def graph_scatter_valid(vx,vyaw,X,y):
    r = 12
    theta = np.linspace(0,2*np.pi,101)

    x_test = r*np.cos(theta)
    y_test = r*np.sin(theta)
    print(1/r)

    fig, ax = plt.subplots(1,1)
    ax.set_aspect("equal")
    ax.scatter(vyaw,vx)
    ax.scatter(x_test,y_test)

    print("var vyaw",np.var(vyaw))
    print("var vx",np.var(vx))

    print("covariance matrix \n",np.cov(X.T))

    alpha = np.cov(X.T)
    print(np.var(y))
    plt.show()

def plot_image(ax,X_train,mean_prediction,col,y,terrain,x_2_eval,normalize={"normalize":False},cline=False,filter={},shape=(100,100),colormap="PuOr",x_lim=(-6,6),y_lim=(-6,6)):
    
    if ax == None:
        fig, ax = plt.subplots(1,1)

    

    if normalize["normalize"]:
        normalizer = normalize[col]
        #mean_prediction = normalizer(mean_prediction)
        #y = normalizer(y)
        
        if isinstance(filter,np.ndarray): 
            filtered_prediction = np.where(filter,mean_prediction.reshape(shape),0)
            
            #filtered_prediction_2d = filtered_prediction.reshape((filtered_shape,filtered_shape))

            im = ax.imshow(filtered_prediction,extent=(x_lim[0], x_lim[1], y_lim[0], y_lim[1]), origin='lower', cmap=colormap, vmin=normalizer.vmin, vmax=normalizer.vmax)
            scatter = ax.scatter(X_train[:,0],X_train[:,1],c=y,cmap=colormap,edgecolor='black',vmin=normalizer.vmin, vmax=normalizer.vmax)
            final_shape = int(np.sqrt(mean_prediction.shape[0]))

        else:
            im = ax.imshow(mean_prediction.reshape(shape),extent=(x_lim[0], x_lim[1], y_lim[0], y_lim[1]), origin='lower', cmap=colormap, vmin=normalizer.vmin, vmax=normalizer.vmax)
            scatter = ax.scatter(X_train[:,0],X_train[:,1],c=y,cmap=colormap,edgecolor='black',vmin=normalizer.vmin, vmax=normalizer.vmax)
            final_shape = int(np.sqrt(mean_prediction.shape[0]))

        
        if cline:
            CS = ax.contour(x_2_eval[:,0].reshape((final_shape,final_shape)),
                    x_2_eval[:,1].reshape((final_shape,final_shape)),
                    mean_prediction.reshape((final_shape,final_shape)),colors="black")
            
            ax.clabel(CS, inline=True, fontsize=10)
    else:
        if filter != {}: 
            im = ax.imshow(mean_prediction.reshape(shape)[filter],extent=(x_lim[0], x_lim[1], y_lim[0], y_lim[1]), origin='lower', cmap=colormap, vmin=normalizer.vmin, vmax=normalizer.vmax)
            scatter = ax.scatter(X_train[:,0],X_train[:,1],c=y,cmap=colormap,edgecolor='black',vmin=normalizer.vmin, vmax=normalizer.vmax)
            final_shape = int(np.sqrt(mean_prediction.shape[0]))

        else:
            im = ax.imshow(mean_prediction.reshape(shape),extent=(x_lim[0], x_lim[1], y_lim[0], y_lim[1]), origin='lower', cmap=colormap, vmin=normalizer.vmin, vmax=normalizer.vmax)
            scatter = ax.scatter(X_train[:,0],X_train[:,1],c=y,cmap=colormap,edgecolor='black',vmin=normalizer.vmin, vmax=normalizer.vmax)
            final_shape = int(np.sqrt(mean_prediction.shape[0]))


        if cline:
            CS = ax.contour(x_2_eval[:,0].reshape((final_shape,final_shape)),
                       x_2_eval[:,1].reshape((final_shape,final_shape)),
                       mean_prediction.reshape((final_shape,final_shape)),colors="black")
            ax.clabel(CS, inline=True, fontsize=10)
   
    return im

    
    
    # Add the second colorbar
    #cbar2 = plt.colorbar(scatter, ax=ax)
    #cbar2.set_label('Scatter') 




def create_figure_all_slip():
    fig, ax = plt.subplots(1,1,figsize=(8, 6))

def extract_percentiles(data):
    """
    Extracts the 2.5% and 97.5% percentiles from the given array.

    Parameters:
    data (array-like): Input array of values.

    Returns:
    tuple: A tuple containing the 2.5th and 97.5th percentiles.
    """
    # Convert input to a NumPy array if it's not already
    data_array = np.asarray(data)

    # Calculate the 2.5th and 97.5th percentiles
    p2_5 = np.percentile(data_array, 2.5)
    p97_5 = np.percentile(data_array, 97.5)

    return p2_5, p97_5

def plot_losange_limits(ax,geom):
    x,y = geom.exterior.xy
    ax.plot(x,y,color="black")


def graph_of_future_body(path,path_to_geom,to_plot="mean",prefix="mean",relative_slip=False):

    
    with open(path_to_geom, 'rb') as file:
        geom_by_terrain = pickle.load(file)["body"]

    df = pd.read_pickle(path)

    normalize = True
    debug = True
    cline = True
    list_terrain = list(df.terrain.unique())
    size = len(list_terrain)
    fig, axs = plt.subplots(3,size)
    plt.subplots_adjust(wspace=0.25, hspace=0.25)
    fig.set_figwidth(3.5*size)
    fig.set_figheight(3*3)
    
    
    norm_slip_x_ss = plt.Normalize(vmin=np.min(df.slip_body_x_ss), vmax=np.max(df.slip_body_x_ss))
    norm_slip_y_ss = plt.Normalize(vmin=np.min(df.slip_body_y_ss), vmax=np.max(df.slip_body_y_ss))
    norm_slip_yaw_ss = plt.Normalize(vmin=np.min(df.slip_body_y_ss), vmax=np.max(df.slip_body_yaw_ss))
    
    if to_plot == "mean":
        max = 3
        max_yaw = 3
        min = -max
        min_yaw = -max_yaw
    if to_plot =="std":
        min = 0.0
        min_yaw = 0.0
        max = 0.2
        max_yaw = 0.1


    norm_slip_x_ss = plt.Normalize(vmin=min, vmax=max)
    norm_slip_y_ss = plt.Normalize(vmin=min, vmax=max)
    norm_slip_yaw_ss = plt.Normalize(vmin=min_yaw, vmax=max_yaw)
    
    print(norm_slip_x_ss)
    
    norm_global_dict = {"normalize":normalize,
                        "slip_body_x_ss":norm_slip_x_ss,
                        "slip_body_y_ss":norm_slip_y_ss,
                        "slip_body_yaw_ss":norm_slip_yaw_ss}
    
    for i in range(size):  
        terrain = list_terrain[i]
        
        df_terrain = df.loc[df["terrain"]==terrain]
        
        if size == 1:
            ax_to_plot = axs[0]
            ax_to_plot_2 = axs[1]
            ax_to_plot_3 = axs[2]
        else:
            ax_to_plot = axs[0,i]
            ax_to_plot_2 = axs[1,i]
            ax_to_plot_3 = axs[2,i]
            
            geom = geom_by_terrain[terrain]
            plot_losange_limits(ax_to_plot,geom)
            plot_losange_limits(ax_to_plot_2,geom)
            plot_losange_limits(ax_to_plot_3,geom)
        
        
        
        col_x_y = ["cmd_body_yaw_lwmean","cmd_body_x_lwmean" ]
        im1 = find_the_surface(df_terrain,"slip_body_x_ss",terrain,geom_to_filter =geom_by_terrain, ax= ax_to_plot, debug=debug,norm= norm_global_dict,cline=cline,colormap="PuOr",to_plot=to_plot,col_x_y=col_x_y)
        im2 = find_the_surface(df_terrain,"slip_body_y_ss",terrain,geom_to_filter =geom_by_terrain,ax= ax_to_plot_2,debug=debug,norm= norm_global_dict,cline=cline,colormap="PuOr",to_plot=to_plot,col_x_y=col_x_y)
        im3 = find_the_surface(df_terrain,"slip_body_yaw_ss",terrain,geom_to_filter =geom_by_terrain,ax=ax_to_plot_3,debug=debug,norm= norm_global_dict,cline=cline,colormap="PiYG",to_plot=to_plot,col_x_y=col_x_y)

        ax_to_plot.set_title(f"{terrain}")
        #ax.set_title(f"{col} on {terrain} ")
        ax_to_plot_3.set_xlabel("Angular velocity [rad/s]")

        if i ==0:
            ax_to_plot.set_ylabel("Linear velocity [m/s]")
            ax_to_plot_2.set_ylabel("Linear velocity [m/s]")
            ax_to_plot_3.set_ylabel("Linear velocity [m/s]")

        
    if size == 1:
        # Add a colorbar
        cbar = plt.colorbar(im1, ax=axs[0])
        cbar.set_label("slip_body_x_ss")  
        cbar = plt.colorbar(im2, ax=axs[1])
        cbar.set_label("slip_body_y_ss")
        cbar = plt.colorbar(im3, ax=axs[2])
        cbar.set_label("slip_body_yaw_ss") 
    else:
        # Add a colorbar
        cbar = plt.colorbar(im1, ax=axs[0,axs.shape[1]-1])
        cbar.set_label("slip_body_x_ss")  
        cbar = plt.colorbar(im2, ax=axs[1,axs.shape[1]-1])
        cbar.set_label("slip_body_y_ss")
        cbar = plt.colorbar(im3, ax=axs[2,axs.shape[1]-1])
        cbar.set_label("slip_body_yaw_ss")  
        
    for ax in np.ravel(axs):

        ax.set_facecolor("black")
    # Optional label for the colorbar
    fig.savefig(path.parent/(prefix + path.parts[-1]+".pdf"),format="pdf")
    plt.show()

def graph_of_future_body_general(path,column,path_to_geom,to_plot="mean",prefix="mean",relative_slip=False,abs=False):

    
    
    with open(path_to_geom, 'rb') as file:
        geom_by_terrain = pickle.load(file)["body"]

    df = pd.read_pickle(path)
    if abs:
        df[column] = df[column].abs()
    if column not in df.columns:
        print_column_unique_column(df)

        raise ValueError(f"{column} is not in column")
    normalize = True
    debug = True
    cline = True
    list_terrain = list(df.terrain.unique())
    size = len(list_terrain)
    fig, axs = plt.subplots(2,size)
    plt.subplots_adjust(wspace=0.25, hspace=0.25)
    fig.set_figwidth(3.5*size)
    fig.set_figheight(3*3)
    
    
    norm_slip_x_ss = plt.Normalize(vmin=np.min(df[column]), vmax=np.max(df[column]))

    if to_plot == "mean":
        max = 0.5
        max_yaw = 3
        min = -max
        min_yaw = -max_yaw
    if to_plot =="std":
        min = -0.2
        min_yaw = 0.0
        max = 0.2
        max_yaw = 0.1


    norm_slip_x_ss = plt.Normalize(vmin=min, vmax=max)
    
    
    norm_global_dict = {"normalize":normalize,
                        column:norm_slip_x_ss}
    
    for i in range(size):  
        terrain = list_terrain[i]
        
        df_terrain = df.loc[df["terrain"]==terrain]
        
        if size == 1:
            ax_to_plot = axs[0]
            
        else:
            ax_to_plot = axs[0,i]
            
            
            geom = geom_by_terrain[terrain]
            plot_losange_limits(ax_to_plot,geom)
            
        
        
        col_x_y = ["cmd_body_yaw_lwmean","cmd_body_x_lwmean" ]
        im1 = find_the_surface(df_terrain,column,terrain,geom_to_filter =geom_by_terrain, ax= ax_to_plot, debug=debug,norm= norm_global_dict,cline=cline,colormap="PuOr",to_plot=to_plot,col_x_y=col_x_y)
        
        ax_to_plot.set_title(f"{terrain}")
        #ax.set_title(f"{col} on {terrain} ")
        ax_to_plot.set_xlabel("Angular velocity [rad/s]")

        if i ==0:
            ax_to_plot.set_ylabel("Linear velocity [m/s]")
            

        
    if size == 1:
        # Add a colorbar
        cbar = plt.colorbar(im1, ax=axs[0])
        cbar.set_label(column)  
        
    else:
        # Add a colorbar
        cbar = plt.colorbar(im1, ax=axs[0,axs.shape[1]-1])
        cbar.set_label(column)  
        
    for ax in np.ravel(axs):

        ax.set_facecolor("black")
    # Optional label for the colorbar
    fig.savefig(path.parent/(column+prefix + path.parts[-1]+".pdf"),format="pdf")
    plt.show()


def graph_of_future_wheel(path,path_to_geom,to_plot="mean",prefix="wheel_mean"):

    
    with open(path_to_geom, 'rb') as file:
        geom_by_terrain = pickle.load(file)["wheel"]

    df = pd.read_pickle(path)

    normalize = True
    debug = True
    cline = True
    list_terrain = list(df.terrain.unique())
    size = len(list_terrain)
    fig, axs = plt.subplots(2,size)
    plt.subplots_adjust(wspace=0.25, hspace=0.25)
    fig.set_figwidth(3.5*size)
    fig.set_figheight(3*2)
    
    
    
    norm_slip_x_ss = plt.Normalize(vmin=np.min(df.slip_wheel_left_ss), vmax=np.max(df.slip_body_x_ss))
    norm_slip_y_ss = plt.Normalize(vmin=np.min(df.slip_wheel_right_ss), vmax=np.max(df.slip_body_y_ss))
    
    
    if to_plot == "mean":
        max = 6
        max_yaw = 6
        min = -max
        min_yaw = -max_yaw
    if to_plot =="std":
        min = 0.0
        min_yaw = 0.0
        max = 1
        max_yaw = 0.1


    norm_slip_x_ss = plt.Normalize(vmin=min, vmax=max)
    norm_slip_y_ss = plt.Normalize(vmin=min, vmax=max)
    
    
    
    norm_global_dict = {"normalize":normalize,
                        "slip_wheel_left_ss":norm_slip_x_ss,
                        "slip_wheel_right_ss":norm_slip_y_ss}
    default_alpha = 1
    alpha_dict = {
        "ice": 1,
        "asphalt":default_alpha,
        "grass":3,
        "sand":default_alpha,
        "gravel":default_alpha,

    }
    for i in range(size):  
        terrain = list_terrain[i]
        
        df_terrain = df.loc[df["terrain"]==terrain]
        
        if size == 1:
            ax_to_plot = axs[0]
            ax_to_plot_2 = axs[1]
            
        else:
            ax_to_plot = axs[0,i]
            ax_to_plot_2 = axs[1,i]
            
            
            geom = geom_by_terrain[terrain]
            plot_losange_limits(ax_to_plot,geom)
            plot_losange_limits(ax_to_plot_2,geom)
        alpha = alpha_dict[terrain]
        
        col_x_y = ["cmd_right_wheels","cmd_left_wheels"]
        im1 = find_the_surface(df_terrain,"slip_wheel_left_ss",terrain,
                            geom_to_filter =geom_by_terrain, ax= ax_to_plot,
                            debug=debug,norm= norm_global_dict,cline=cline,
                            colormap="PuOr",to_plot=to_plot,col_x_y=col_x_y,
                            x_lim=(-17,17),y_lim=(-17,17),alpha = alpha)
        im2 = find_the_surface(df_terrain,"slip_wheel_right_ss",terrain,
                            geom_to_filter =geom_by_terrain,ax= ax_to_plot_2,
                            debug=debug,norm= norm_global_dict,cline=cline,
                            colormap="PuOr",to_plot=to_plot,col_x_y=col_x_y,
                            x_lim=(-17,17),y_lim=(-17,17),alpha = alpha)
        
        ax_to_plot.set_title(f"{terrain}")
        #ax.set_title(f"{col} on {terrain} ")
        ax_to_plot_2.set_xlabel("Right wheel speed [rad/s]")

        if i ==0:
            ax_to_plot.set_ylabel("Left wheel speed [rad/s]")
            ax_to_plot_2.set_ylabel("Left wheel speed [rad/s]")
            

        
    if size == 1:
        # Add a colorbar
        cbar = plt.colorbar(im1, ax=axs[0])
        cbar.set_label("slip_wheel_left_ss")  
        cbar = plt.colorbar(im2, ax=axs[1])
        cbar.set_label("slip_wheel_right_ss")
        
    else:
        # Add a colorbar
        cbar = plt.colorbar(im1, ax=axs[0,axs.shape[1]-1])
        cbar.set_label("slip_wheel_left_ss")  
        cbar = plt.colorbar(im2, ax=axs[1,axs.shape[1]-1])
        cbar.set_label("slip_wheel_right_ss")
        
        
    for ax in np.ravel(axs):

        ax.set_facecolor("black")
    # Optional label for the colorbar
    fig.savefig(path.parent/(prefix + path.parts[-1]+".pdf"),format="pdf")
    plt.show()

if __name__=="__main__":
    # Arg parser for the pickle file
    parser = argparse.ArgumentParser(description="Process some arguments.")
    parser.add_argument("--dataset",type=str,help="Path to the pickle file", default=DATASET_PICKLE)
    parser.add_argument("--geom",type=str,help="Path to the pickle file containing the geom", default=GEOM_PICKLE)

    path = pathlib.Path(parser.parse_args().dataset)
    path_to_geom = pathlib.Path(parser.parse_args().geom)
    
    to_plot = "mean"
    #graph_of_future_wheel(path,path_to_geom,to_plot,prefix="wheel_mean")
    to_plot = "std"
    #graph_of_future_wheel(path,path_to_geom,to_plot,prefix="wheel_std")
    to_plot = "mean"
    #graph_of_future_body(path,path_to_geom,to_plot,prefix="mean")
    to_plot = "std"
    #graph_of_future_body(path,path_to_geom,to_plot,prefix="std") 

    graph_of_future_body_general(path,"slip_angle_ss",path_to_geom,to_plot,prefix="mean")
    