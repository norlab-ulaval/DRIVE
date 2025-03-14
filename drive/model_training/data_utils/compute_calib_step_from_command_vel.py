import pandas as pd 
import numpy as np 
import matplotlib.pyplot as plt
import extractors

def compute_calib_step_from_cmd_vel_x(df,path_2_save,nb_points_by_window=6*20+1,prefix=""):


    cmd_vel_x = df["cmd_vel_x"].astype(float).to_numpy()
    cmd_vel_yaw = df["cmd_vel_omega"].astype(float).to_numpy()
    cmd_vel_yaw = df["cmd_vel_omega"].astype(float).to_numpy()
    ros_time = df.ros_time.astype(float)
    index =  df["cmd_vel_x"].index
    calib_state = "calib"
    calib_step = 0

    list_array = []


    new_calib_step = ["0"]
    new_calib_state = ["0"]

    calib_step = 0
    i = 1

    list_index = []
    list_cmd = []

    first_step = False
    while i <= (cmd_vel_x.shape[0] - nb_points_by_window ):
        
        
        delta_t =  0 
        j = i + 1
        while delta_t < 6.0:
            
            delta_t  = (ros_time.loc[j] - ros_time.loc[i]) * 10**(-9)
            j += 1
        
        #print(i,j)
        #print(j-i, delta_t)
        extract_120_cmd_vel_x = cmd_vel_x[i:j]
        extract_120_cmd_vel_yaw = cmd_vel_yaw[i:j]
        

        #print(np.unique(extract_120_cmd_vel_yaw).shape)
        #if (np.unique(extract_120_cmd_vel_x).shape[0] ==  1) or (np.unique(extract_120_cmd_vel_yaw).shape[0] == 1) \
        #    and (extract_120_cmd_vel_yaw[0] != 0.0 or extract_120_cmd_vel_x[0] != 0.0):

        treshold = 10**-5
        treshold_2 = 10**-15
        if ((np.std(extract_120_cmd_vel_x) <=  treshold_2) or (np.std(extract_120_cmd_vel_yaw) <=  treshold_2)) \
            and (np.abs(extract_120_cmd_vel_yaw[0]) >= treshold or np.abs(extract_120_cmd_vel_x[0]) >= treshold):
            first_step = True    
            
            new_calib_step.extend([f"{calib_step}"]*(j-i))
            new_calib_state.extend(["calib"]*(j-i)) 
            
            calib_step += 1


            test =2
            list_index.append(i)
            list_cmd.append(calib_state)
            
            
            

            fig, axs = plt.subplots(1,1)

            axs.scatter(np.linspace(0,5.95,j-i),extract_120_cmd_vel_x)
            axs.scatter(np.linspace(0,5.95,j-i),extract_120_cmd_vel_yaw)

            i = j

            fig.savefig(f"drive/model_training/data_utils/test_reverse_calib_step/{prefix}_{calib_step}_calib_step_ice.png")
            plt.close("all")

        else:
            

            if first_step:
                
                new_calib_state.append("idle") 
            else:
                new_calib_state.append("") 
            
            new_calib_step.append(f"{calib_step}")
            i += 1 
    
    # Add useless idle to complete the shape 

    shape = df.shape[0]
    nb_2_add = shape - len(new_calib_state)

    new_calib_state.extend(["idle"]*nb_2_add)
    new_calib_step.extend([f"{calib_step}"]*nb_2_add)
    
    # Replace the column 
    new_calib_state = np.array(new_calib_state)
    new_calib_step = np.array(new_calib_step)
    df.calib_state = new_calib_state
    df.calib_step = new_calib_step
    
    df.to_pickle(path_2_save)
    

    test2 = df['calib_state'].to_numpy().astype('str')
    test= df['calib_step'].to_numpy().astype('int')
    print("calib_step", calib_step)
    start = 0
    end = -1

    fig,axs = plt.subplots(2,sharex=True)
    axs[0].plot(np.arange(len(new_calib_step))*0.05,np.array(new_calib_step).astype(float))
    axs[0].vlines(np.array(list_index)*0.05,ymin=-5,ymax=60,color="red")
    axs[0].scatter(index[start:end]*0.05,cmd_vel_x[start:end],s=1.0)
    axs[1].vlines(np.array(list_index)*0.05,ymin=-5,ymax=5,color="red")
    axs[1].scatter(index[start:end]*0.05,cmd_vel_yaw[start:end],s=1.0)
    

def valide_cmd(df,prefix="",path_to_save = "drive/model_training/data_utils/test_reverse_calib_step/test_calib_torch_df"):

    
    cmd_wheel_l = np.array(list((extractors.extract_df_colummn_into_6_sec_dico(df,"cmd_left").values()))).T
    cmd_wheel_r = np.array(list((extractors.extract_df_colummn_into_6_sec_dico(df,"cmd_right").values()))).T
    
    for i in range(cmd_wheel_r.shape[0]):
        
        fig, axs = plt.subplots(1,1)
        axs.scatter(np.linspace(0,5.95,120),cmd_wheel_l[i,:])
        axs.scatter(np.linspace(0,5.95,120),cmd_wheel_r[i,:])
        fig.savefig(path_to_save+f"{prefix}_{i}_calib_step_ice.png")
        plt.close("all")

        


if __name__ == "__main__":


    df = pd.read_pickle("drive_datasets/data/warthog/wheels/grass/warthog_wheels_grass_2024_9_20_9h27s52/model_training_datasets/torch_ready_dataframe.pkl")

    df2 = pd.read_pickle("drive_datasets/data/warthog/wheels/grass/warthog_wheels_grass_2024_9_20_9h27s52/model_training_datasets/raw_dataframe.pkl")
    
    #path_2_dataraw = "drive_datasets/data/warthog/wheels/ice/warthog_wheels_ice/model_training_datasets/raw_dataframe_wronged_calib_0_1.pkl"
    path_2_dataraw = "drive_datasets/data/warthog/wheels/ice/warthog_wheels_ice/model_training_datasets/raw_dataframe_wronged_calib.pkl"
    path_to_save = "/home/nicolassamson/ros2_ws/src/DRIVE/drive_datasets/data/warthog/wheels/ice/warthog_wheels_ice/model_training_datasets/raw_dataframe.pkl"
    df_raw = pd.read_pickle(path_2_dataraw)
    compute_calib_step_from_cmd_vel_x(df_raw,path_to_save,prefix="warthog_wheels_ice_")
    print(df_raw.shape)
    

    path_2_dataraw2 = "/home/nicolassamson/ros2_ws/src/DRIVE/drive_datasets/data/warthog/wheels/ice/warthog_wheels_ice_2024_7_30_16h30s21_ice_param/model_training_datasets/raw_dataframe_wronged_calib.pkl"
    path_to_save_2 = "/home/nicolassamson/ros2_ws/src/DRIVE/drive_datasets/data/warthog/wheels/ice/warthog_wheels_ice_2024_7_30_16h30s21_ice_param/model_training_datasets/raw_dataframe.pkl"
    
    df_raw_2 = pd.read_pickle(path_2_dataraw2)
    print(df_raw_2.shape)
    compute_calib_step_from_cmd_vel_x(df_raw_2[:int(1127/0.05)],path_to_save_2,prefix="warthog_wheels_ice_2024_7_30_16h30s21_ice_param")

    #plt.show()

    df_steady_state = pd.read_pickle("drive_datasets/data/warthog/wheels/ice/warthog_wheels_ice/model_training_datasets/torch_ready_dataframe.pkl")
    df_ss_2 = pd.read_pickle("drive_datasets/data/warthog/wheels/ice/warthog_wheels_ice_2024_7_30_16h30s21_ice_param/model_training_datasets/torch_ready_dataframe.pkl")
    extractors.print_column_unique_column(df_steady_state)

    valide_cmd(df_steady_state,prefix="warthog_wheels_ice_")
    valide_cmd(df_ss_2,prefix="warthog_wheels_warthog_wheels_ice_2024_7_30_16h30s21")
    
   # path_to_example = "/home/nicolassamson/ros2_ws/src/DRIVE/drive_datasets/data/warthog/wheels/ice/warthog_wheels_ice_ral2023/model_training_datasets/raw_dataframe.pkl"
#
   # df_ex = pd.read_pickle(path_to_example)
#
   # df_ex.calib_step = df_ex.calib_step.astype(float).to_numpy()
#
   # calib_value = df_ex.loc[df_ex.calib_state == "calib"]
   # 
   # 
   # #df_ex.to_csv("/home/nicolassamson/ros2_ws/src/DRIVE/drive_datasets/data/warthog/wheels/ice/warthog_wheels_ice_2024_7_30_16h30s21_ice_param/model_training_datasets/test.csv")
    #df_raw.to_csv("/home/nicolassamson/ros2_ws/src/DRIVE/drive_datasets/data/warthog/wheels/ice/warthog_wheels_ice_2024_7_30_16h30s21_ice_param/model_training_datasets/test_raw.csv")
    #fig,axs = plt.subplots(2,sharex=True)
    #
    #plt.show()
#
#
    #path_2_dataraw3 = "/home/nicolassamson/ros2_ws/src/DRIVE/drive_datasets/data/warthog/wheels/asphalt/warthog_wheels_asphalt_2024_9_20_8h21s48/model_training_datasets/raw_dataframe_without_calib_step.pkl"
    #path_to_save_3 = "/home/nicolassamson/ros2_ws/src/DRIVE/drive_datasets/data/warthog/wheels/asphalt/warthog_wheels_asphalt_2024_9_20_8h21s48/model_training_datasets/raw_dataframe.pkl"
    #
    #df_raw_3 = pd.read_pickle(path_2_dataraw3)
    #print(df_raw_3.shape)
    #compute_calib_step_from_cmd_vel_x(df_raw_3,path_to_save_3)
    #plt.show()
#
#
