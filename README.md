# DRIVE - Data-driven Robot Input Vector Exploration


[![DOI](https://zenodo.org/badge/DOI/10.48550/arxiv.2309.110718.svg)](https://doi.org/10.48550/arXiv.2309.10718)
[![ROS2](https://img.shields.io/badge/ROS2-humble-blue?labelColor=blue&logo=ROS)](https://docs.ros.org/en/humble)

DRIVE is an open-source uncrewed ground vehicle (UGV) training dataset gathering protocol.
This protocol automates the task of driving the UGV to gather a training dataset, then used to train a motion model. The resulting model can then be used for controllers.

[![SNOW WILN deployment](https://img.youtube.com/vi/tBCtC7WolL4/0.jpg)](https://www.youtube.com/watch?v=tBCtC7WolL4)

👉 [See on Youtube](https://www.youtube.com/watch?v=tBCtC7WolL4)

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

## Datasets

The datasets used in our paper are publicly available.
[Follow this link to download them in Pandas Dataframe format](https://github.com/norlab-ulaval/Norlab_wiki/wiki/DRIVE-datasets).
## Installation

DRIVE works as a standard ROS 2 package.
You simply need to clone this repository in your ROS 2 workspace and compile it.
The package has dependencies to the following standard Python libraries and custom message package :

* [Numpy](https://numpy.org/)
* [Pandas](https://pandas.pydata.org/)
* [Scipy](https://scipy.org/)
* [Matplotlib](https://matplotlib.org/)
* [norlab_controllers_msgs](https://github.com/norlab-ulaval/norlab_controllers_msgs)

```sh
git clone git@github.com:norlab-ulaval/DRIVE.git && git clone git@github.com:norlab-ulaval/norlab_controllers_msgs.git
cd ..
rosdep install --from-paths src -y --ignore-src
colcon build
```

## Usage

To execute the protocol, the first step is to create a YAML configuration file.
Examples are located in the [`/config`](https://github.com/norlab-ulaval/DRIVE/tree/humble/config) folder.
The table below provides parameter description:

| Parameter name                  | Type    | Default              | Note                                                                                                                                                                                               |
| :------------------------------ | :------ | :------------------- | :------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| command_model                   | String  | 'Differential_drive' | Vehicle command model (currently only Differential Drive is implemented, more to come soon)                                                                                                        |
| auto_max_speed_characterization | Boolean | True                 | Automatic UGV max speed characterization. If set to 'False', will use user-defined maximum linear speed to estimate limits.                                                                        |
| auto_cmd_model_characterization | Boolean | True                 | Automatic UGV command model characterization. If set to 'False', will use command model parameters. For differential-drive, these are set to wheel_radius and wheel_baseline.                      |
| max_lin_speed                   | Double  | 2.0                  | Vehicle maximum linear speed                                                                                                                                                                       |
| max_ang_speed                   | Double  | 2.0                  | Vehicle maximum angular speed                                                                                                                                                                      |
| wheel_radius                    | Double  | 1.0                  | Vehicle wheel radius                                                                                                                                                                               |
| wheel_baseline                  | Double  | 1.0                  | Vehicle wheel baseline                                                                                                                                                                             |
| cmd_rate                        | Integer | 20                   | Vehicle command rate (in Hz)                                                                                                                                                                       |
| encoder_rate                    | Integer | 4                    | Vehicle wheel encoders measurement rate (in Hz)                                                                                                                                                    |
| step_len                        | Double  | 6.0                  | Calibration interval step (in seconds)                                                                                                                                                             |
| n_calib_steps                   | Integer | 20                   | Number of calibration steps. In our paper, we show that 10 6-second steps are enough for our model to converge.                                                                                    |
| dead_man_button                 | Boolean | True                 | Is the dead man switch on the controller set as a button in the [`joy`](https://docs.ros2.org/foxy/api/sensor_msgs/msg/Joy.html) message? If set to "False", it will be looking in the "axes".     |
| dead_man_threshold              | Double  | 0.5                  | Dead man switch activation threshold                                                                                                                                                               |
| calib_trigger_button            | Boolean | True                 | Is the calibration trigger on the controller set as a button in the [`joy`](https://docs.ros2.org/foxy/api/sensor_msgs/msg/Joy.html) message? If set to "False", it will be looking in the "axes". |
| calib_trigger_threshold         | Double  | 0.5                  | Calibration trigger activation threshold                                                                                                                                                           |

Then, make sure that the [DRIVE launch file](https://github.com/norlab-ulaval/DRIVE/blob/humble/launch/drive.launch.xml) is adapted to your configuration and proper topics.
It is possible to record a rosbag of the experiment by uncommenting this line : `<!--  <executable cmd="ros2 bag record -a -o /media/robot/ssd-2nd/rosbag/doughnut" />-->`.
You can then launch the characterization procedure with :

```sh
ros2 launch drive drive.launch.xml
```

Make sure an operator is present during the characretization. The dead man switch allows to stop the UGV and pause the characterization protocol.
The operator can move and re-orient the UGV to allow it to execute the following commands.
When done, you can use the following command to save the raw data in a Pandas Dataframe :

```sh
ros2 service call /logger_node/export_data norlab_controllers_msgs/srv/ExportData "export_path: data: '<EXPORT_DATAFRAME_PATH>'"
```

### DRIVE node graph

```mermaid
flowchart LR

/drive_node[ /drive_node ]:::main

/imu_in([ /imu_in<br>sensor_msgs/msg/Imu ]):::bugged
/joy([ /joy<br>sensor_msgs/msg/Joy ]):::bugged
/left_wheel_vel([ /left_wheel_vel<br>std_msgs/msg/Float64 ]):::bugged
/right_wheel_vel([ /right_wheel_vel<br>std_msgs/msg/Float64 ]):::bugged
/calib_state([ /calib_state<br>std_msgs/msg/String ]):::bugged
/calib_step([ /calib_step<br>std_msgs/msg/Int32 ]):::bugged
/doughnut_cmd_vel([ /doughnut_cmd_vel<br>geometry_msgs/msg/Twist ]):::bugged
/good_calib_step([ /good_calib_step<br>std_msgs/msg/Bool ]):::bugged
/joy_switch([ /joy_switch<br>std_msgs/msg/Bool ]):::bugged


/imu_in --> /drive_node
/joy --> /drive_node
/left_wheel_vel --> /drive_node
/right_wheel_vel --> /drive_node
/drive_node --> /calib_state
/drive_node --> /calib_step
/drive_node --> /doughnut_cmd_vel
/drive_node --> /good_calib_step
/drive_node --> /joy_switch




subgraph keys[<b>Keys<b/>]
subgraph nodes[<b><b/>]
topicb((No connected)):::bugged
main_node[main]:::main
end
subgraph connection[<b><b/>]
node1[node1]:::node
node2[node2]:::node
node1 o-.-o|to server| service[/Service<br>service/Type\]:::service
service <-.->|to client| node2
node1 -->|publish| topic([Topic<br>topic/Type]):::topic
topic -->|subscribe| node2
node1 o==o|to server| action{{/Action<br>action/Type/}}:::action
action <==>|to client| node2
end
end
classDef node opacity:0.9,fill:#2A0,stroke:#391,stroke-width:4px,color:#fff
classDef action opacity:0.9,fill:#66A,stroke:#225,stroke-width:2px,color:#fff
classDef service opacity:0.9,fill:#3B8062,stroke:#3B6062,stroke-width:2px,color:#fff
classDef topic opacity:0.9,fill:#852,stroke:#CCC,stroke-width:2px,color:#fff
classDef main opacity:0.9,fill:#059,stroke:#09F,stroke-width:4px,color:#fff
classDef bugged opacity:0.9,fill:#933,stroke:#800,stroke-width:2px,color:#fff
style keys opacity:0.15,fill:#FFF
style nodes opacity:0.15,fill:#FFF
style connection opacity:0.15,fill:#FFF

```

