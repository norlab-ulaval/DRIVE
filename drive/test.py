import yaml 
import pandas as pd
import pathlib 
import shutil
import os


path_src = pathlib.Path("/home/nicolassamson/ros2_ws/src/DRIVE/drive/test_folder_1")

path_dest = pathlib.Path("/home/nicolassamson/ros2_ws/src/DRIVE/drive/test_folder_2")

print(os.listdir   (path_src))