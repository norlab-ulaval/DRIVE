import numpy as np 
import pandas as pd
from drive.model_training.models.kinematic.ideal_diff_drive import Ideal_diff_drive



def print_column_unique_column(df):
    df_columns = list(df.columns)
    possible_number = ["1","2","3","4","5","6","7","8","9","0"]
    for i,column in enumerate(df_columns):

        if column[-1] in possible_number:

            if column[-2] in possible_number:
                df_columns[i] = column[:-3]

                if column[-3] in possible_number:
                    df_columns[i] = column[:-4]
            else:
                df_columns[i] = column[:-2]

    df_columns_name = pd.Series(df_columns)
    unique_name = df_columns_name.unique()
    unique_name.sort()
    
    print(unique_name)
    return unique_name





def extracts_appropriate_columns(df,commun_name):

    df_columns = list(df.columns)
    possible_number = ["1","2","3","4","5","6","7","8","9","0"]

    for i,column in enumerate(df_columns):

        if column[-1] in possible_number:

            if column[-2] in possible_number:
                df_columns[i] = column[:-3]

                if column[-3] in possible_number:
                    df_columns[i] = column[:-4]
            else:
                df_columns[i] = column[:-2]

    df_columns_name = pd.Series(df_columns)

    
    mask = [name == commun_name for name in df_columns_name ]
    kept_column = df.columns[mask]

    return df[kept_column]

def column_type_extractor(df, common_type,
                        transient_state=True,steady_state=True, verbose=False):
    """ Extract the np.matrix that represent the 40 columns of all steps of the 
    specific type. 
    column_type_extractor
    For example, icp_velx_40 is of the type icp_velx 
    
    """
    
    
    
    #if len(column_to_take)>1:

    
    local_df = df.copy()
    if transient_state==False and steady_state==True:
        mask = local_df.steady_state_mask == 1
        local_df = local_df.loc[mask]
    elif steady_state == False and transient_state == True:
        mask = local_df.steady_state_mask == 0
        local_df = local_df.loc[mask]
    elif steady_state == False and transient_state == False:
        raise ValueError("Both steady state and transient can not be at false")
    
    
        
    local_df = extracts_appropriate_columns(local_df,common_type)

    np_results = local_df.to_numpy().astype('float')


    if verbose == True:
        message = "_"*8+f"{common_type}"+"_"*8
        print(message)
        print(f"The column type: {common_type}")
        print(f"The resulting dataframe_shape: {np_results.shape}")
        print(f"Number of calibrating steps:{np_results.shape[0]}")
        print(f"Number of measurement by step: {np_results.shape[1]}")
        print(f"Maximum {np.max(np_results)}")
        print(f"Minimum {np.min(np_results)}")
        print("_"*len(message))


    return np_results

def create_time_axe(rate,n):
    
    return np.array(range(0,n)) * rate

def compute_body_vel_IDD( u, robot='warthog-wheel'):
    if robot == 'warthog-wheel':
        wheel_radius = 0.3
        baseline = 1.1652
        
        rate = 0.05

        model = Ideal_diff_drive(wheel_radius,baseline,rate)
    
    
    body_vel = model.compute_body_vel(u)

    return body_vel

def compute_operation_points_and_step(res_2d_array,cmd_2d_array):
    #print(cmd_2d_array.shape)
    reshaped_res_2d_array = reshape_into_6sec_windows(res_2d_array)
    reshaped_cmd = reshape_into_6sec_windows(cmd_2d_array)

    operation_point = (reshaped_res_2d_array[:,0:5].mean(axis=1)).reshape((reshaped_res_2d_array.shape[0],1))
    command_abso = (reshaped_cmd[:,-5:].mean(axis=1)).reshape((reshaped_res_2d_array.shape[0],1)) 
    
    
    
    steps = command_abso - operation_point
    return operation_point,steps

def compute_operation_points_and_step_using_mask(res_2d_array,cmd_2d_array,mask,nb_points_per_window):
    #print(cmd_2d_array.shape)
    reshaped_mask = reshape_into_6sec_windows(mask) # array

    # created the array_shift_to_extract_the_value
    reshaped_res_2d_array = reshape_into_6sec_windows(res_2d_array)
    reshaped_cmd = reshape_into_6sec_windows(cmd_2d_array)
    # Assuming that we always start at zero 
    first_line_of_zeros = np.zeros((1,nb_points_per_window))
    shifted_array = np.vstack((first_line_of_zeros,reshaped_res_2d_array[:-1,-nb_points_per_window:])) 
    shifted_array_mean = np.mean(shifted_array,axis=1)


    operation_point = np.where(reshaped_mask[:,0], shifted_array_mean,np.zeros(shifted_array_mean.shape[0])).reshape((reshaped_cmd.shape[0],1))
    
    command_abso = reshaped_cmd[:,-5:].mean(axis=1).reshape((reshaped_res_2d_array.shape[0],1)) 
    
    
    
    steps = command_abso - operation_point
    return operation_point,steps



def normalizer_2d_array(res_2d_array,cmd_2d_array):
    """To normalize the step answer by the first column of the command vector
    Args:
        res_2d_array (_type_): the nb_calibration_stepxhorizon_length(40 or 120) AKA 2 or 6second
        cmd_2d_array (_type_): _description_
    Returns:
        _type_: _description_
    """
    operation_point,steps = compute_operation_points_and_step(res_2d_array,cmd_2d_array)

    normalizer_coefficient = 1/steps

    centered_2d_array = res_2d_array - operation_point
    normalize_2d_array = centered_2d_array * normalizer_coefficient
    
    normalized_cmd_2d_array = np.ones(normalize_2d_array.shape)
    return normalize_2d_array,normalized_cmd_2d_array

def reshape_into_2sec_windows(array_to_reshape, n_window=3):

    first_index = int(array_to_reshape.shape[0]*n_window)
    total_info = array_to_reshape.shape[0]* array_to_reshape.shape[1]
    second_size = int(total_info//first_index)
    reshape_size = (first_index, second_size)

    array_to_reshape = array_to_reshape.reshape(reshape_size)

    return array_to_reshape
def reshape_into_6sec_windows(array_to_reshape, n_window=3):
    """_summary_

    Args:
        array_to_reshape (_type_): _description_
        n_window (int, optional): _description_. Defaults to 3.

    Returns:
        _type_: _description_
    """

    first_index = array_to_reshape.shape[0]//n_window
    total_info = array_to_reshape.shape[0]* array_to_reshape.shape[1]
    second_size = total_info//first_index
    reshape_size = (first_index, second_size)

    array_to_reshape = array_to_reshape.reshape(reshape_size)

    return array_to_reshape

