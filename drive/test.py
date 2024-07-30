import yaml 
import pandas as pd


path_to_calibration_node_config_file = "calib_data/test_warthog_reassemble/config_file_used/_warthog.config.yaml"


# Load param of the calibration node
with open(path_to_calibration_node_config_file, 'r') as file:
    prime_service = yaml.safe_load(file)
    param_dict = prime_service["/drive/calibration_node"]["ros__parameters"]
    rate = param_dict["cmd_rate"]

    print(rate)


df = pd.read_pickle("/home/nicolassamson/ros2_ws/src/DRIVE/calib_data/test_warthog_reassemble/input_space_data.pkl")
df.to_pickle("/home/nicolassamson/ros2_ws/src/DRIVE/calib_data/test_warthog_reassemble/input_space_data.pkl",
            )
print(df.columns)