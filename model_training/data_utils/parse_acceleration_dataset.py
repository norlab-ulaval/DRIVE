import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


from data_utils.acceleration_dataset_parser import AccelerationDatasetParser

slip_dataset_path = '/home/dominic/repos/norlab_WMRD/data/ral2023_dataset/husky/boreal_snow/slip_dataset_all.pkl'
export_dataset_path = '/home/dominic/repos/norlab_WMRD/data/ral2023_dataset/husky/boreal_snow/acceleration_dataset.pkl'
robot = "husky"
# robot = "marmotte"
# robot = "warthog-track"

acceleration_dataset_parser = AccelerationDatasetParser(slip_dataset_path=slip_dataset_path,
                                        export_dataset_path=export_dataset_path,
                                        robot=robot)

acceleration_dataset_parser.append_acceleration_elements_to_dataset()