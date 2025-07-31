# Installation

On your computer (With a simulated robot), see : [On your computer](#on-your-computer)

On your robot, see : [On your robot](#on-your-robot)

## On your computer

Before running on your robot, you can test DRIVE on your computer using a simulated differential drive robot.

### Prerequisites

- Docker
- VSCode
- Foxglove

### Steps

1. Clone the repo

```bash
git clone git@github.com:norlab-ulaval/DRIVE.git -b drive_again
```

2. Open the repo in VSCode

```bash
cd DRIVE
code .
```

3. Download the following extensions for VSCode: "Dev Containers" and "Remote - SSH"

4. Restart VSCode

5. Do `Ctrl+Shift+P` and select "Dev Containers: Rebuild and Reopen in Container". This should build the docker container and open the VSCode inside of it.

6. Once inside, do `Ctrl+Shift+P` and select "Tasks: Run Task" > "launch: Foxglove Bridge"

7. Then, start the DRIVE protocol with `Ctrl+Shift+P` and select "Tasks: Run Tasks" > "launch: Drive ROS Simulation Demo"

8. Open Foxglove on `localhost:8765` with the [DRIVE layout](../foxglove/DRIVE.json) available in the repo

9. Then, you can teleop the robot from Foxglove and test the DRIVE protocol.

## On your robot

We assume that your robot runs on ROS2. Since every robot is different, you will have some topic configuration to do before being able to run a DRIVE experiment. Here is what you will need to know before configuring:

- What is your command topic (linear and angular velocity in the body frame) and message?
- What is your localization topic and message?
- What is you deadman switch topic and message?

Once you have the answer to all these questions, we can start installing DRIVE.

### Prerequisites

- Docker
- Foxglove

### Steps

1. Clone the repo

```bash
git clone git@github.com:norlab-ulaval/DRIVE.git -b drive_again
```

2. Copy the `.env.template` file to `.env` and change the variables according to your setup. Make sure to select the right ROS version and the right middleware or else the container will not be able to communicate with the host.

```bash
cd DRIVE
cp .env.template .env
nano .env
```

3. Now, we will create a folder for your robot config and copy the default config files inside it.

```bash
ROBOT_NAME=<insert your robot name>
mkdir ros/config/$ROBOT_NAME
cp ros/config/*.yaml ros/config/$ROBOT_NAME/
```

4. Create the launch file for your robot.

```bash
cp ros/launch/robot.launch.py ros/launch/$ROBOT_NAME.launch.py
```

5. Edit the launch file for your robot (Check the TODO and set your robot name and the topic configuration)

```bash
nano ros/launch/$ROBOT_NAME.launch.py
```

6. Edit the parameters files for your experiment.

```bash
cd ros/config/$ROBOT_NAME
ls -la
# Check each config file and edit them with your favorite editor
```

7. Build the docker container

```bash
cd ../../../ # Go back to the root of the repo
docker compose build
```

8. Start the docker container

```bash
docker compose up -d
```

9. Check that the communication with your robot is working

```bash
docker exec -it drive_ros bash
ros2 topic list
ros2 topic echo <Any topic of your robot>
exit
```

10. Start the Foxglove Bridge on your robot.

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

11. Open Foxglove from your computer on `<your robot IP>:8765` with the [DRIVE layout](../foxglove/DRIVE.json) available in the repo

12. Good job! You are now ready to execute the DRIVE protocol!
