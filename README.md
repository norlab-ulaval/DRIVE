# DRIVE - Data-driven Robot Input Vector Exploration

[![ROS2](https://img.shields.io/badge/ROS2-humble-blue?labelColor=blue&logo=ROS)](https://docs.ros.org/en/humble)

DRIVE is an open-source uncrewed ground vehicle (UGV) training dataset gathering protocol.
This protocol automates the task of driving the UGV to gather a training dataset, then used to train a motion model. The resulting model can then be used for controllers.


[![DRIVE video](https://img.youtube.com/vi/tBCtC7WolL4/0.jpg)](https://www.youtube.com/watch?v=tBCtC7WolL4)

## Citing

If you use DRIVE in an academic context, please cite [our preprint](https://www.researchgate.net/publication/374023495_DRIVE_Data-driven_Robot_Input_Vector_Exploration):

```bibtex
@misc{baril2023drive,
      title={DRIVE: Data-driven Robot Input Vector Exploration}, 
      author={Dominic Baril and Simon-Pierre Deschênes and Luc Coupal and Cyril Goffin and Julien Lépine and Philippe Giguère and François Pomerleau},
      year={2023},
      eprint={2309.10718},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}
```

## Installation

DRIVE works as a standard ROS 2 package. 
You simply need to clone this repository in your ROS 2 workspace and compile it.
The package has dependencies to the following standard Python libraries : 

* [Numpy](https://numpy.org/);
* [Pandas](https://pandas.pydata.org/);
* [Scipy](https://scipy.org/);
* [Matplotlib](https://matplotlib.org/).

```sh
git clone 
cd ..
rosdep install --from-paths src -y --ignore-src
colcon build
```

## Usage

This section is under construction.

## Datasets

The datasets used in our paper are publicly available. 
[Follow this link to download them in Rosbag or Pandas Dataframe format](https://github.com/norlab-ulaval/Norlab_wiki/wiki/DRIVE-datasets).

