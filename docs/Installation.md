# Installation

## On your computer (Test with a simulated differential drive robot)

If you want to test DRIVE on your computer before using it on a real robot, follow these steps:

1. Clone the repo

```bash
git clone git@github.com:GLO-3002-Norlab/DRIVE.git
```

2. Build the docker images

```bash
cd DRIVE
docker compose build
```

3. Launch the application

```bash
docker compose up -d
```

4. Go on [http://localhost:5000](http://localhost:5000), you should see the user interface
5. Launch a teleoperation node to control the virtual robot. When focused on the terminal, you can control the robot with wasd

```bash
docker exec -it drive_ros bash -ic "ros2 run drive_ros keyboard_teleop"
```

6. Play around on the interface and control the robot on the terminal you launched the last command with

## On your robot

We assume that your robot runs on ROS2. Since every robot is different, you will have some topic configuration to do before being able to run a DRIVE experiment. Here is what you will need to know before configuring:

- How do you actuate your robot to execute a command (from linear and angular velocity)?
- What is your localization topic and message?
- What is you deadman switch topic and message?
- From a goal pose, how do you instruct your robot to go autonomously to that pose? Then, how do you know if the goal is reached?

Once you have the answer to all these questions. See the [Configuration Section](Architecture.md) to start configuring your installation.
