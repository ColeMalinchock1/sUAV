# sUAV Project
This project is to design a ROS2 system that allows a Pixhawk 4 to communicate with an on-board NVIDIA Jetson Orin Nano to control the flight of a small UAV. The vehicle will also include a lidar that is capable of detecting objects in the way of its current path and avoid them.

## Current Status
Using a system developed by [SOURCE] to control the yaw and x velocity, y velocity, and altitude of the drone. It should be a instantaneous control that will adjust the variables without delay. Currently being used to simply move forward and rotate.

## Logging
Logging is toggled by the LOGGING constant in the lib.constants.py file. This will allow the user to know the commands being completed on-board following the flight. It can be toggled off if there is too much overhead from it.

## Notes for testing
- Consider using ttyUSB0 instead of ttyTHS1.
- Disable logging if there is too much overhead
- Use auto-takeoff like it is used in the example

## Future notes
- Use ROS2 instead of mavutil
- Use PID to control the obstacle avoidance after completing simple maneuvers with current communication

## Flow Diagram
![sUAV Flow Diagram](/Resources/FlowDiagram.png)

## Initial setup process
When starting with a new Jetson, follow the steps below

### 1. NVIDIA Jetpack
Using NVIDIA SDK Manager, follow [these steps](https://www.jetson-ai-lab.com/initial_setup_jon_sdkm.html) to flash the Jetson with the up to date Jetpack (Currently tested on Jetpack 6.2)

### 2. Virtual Environment
Setup a virtual environment in root with the following command

```
python -m venv .venv
```

Have it activate automatically by adding these lines to the .bashrc file

```
# Goes back to root
cd

# Checks if it is in a virtual environment and deactivates it
if [[ -n "$VIRTUAL_ENV" ]]; then
    deactivate
fi

# Sources the .venv and ros dependencies
source .venv/bin/activate
```

### 3. Setup ROS2
Visit [this website](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) to follow the instructions on how to install and setup the ROS2 environment. (Tested with Humble Hawksbill)

Install colcon and other ROS2 dependencies with the below commands
```
sudo apt-get install python3-colcon-common-extensions

sudo apt-get install python3-rosdep
```

Follow [these instructions](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html) to setup a ROS2 workspace

### 4. ZED Package
Install the latest ZED SDK version from [these steps](https://www.stereolabs.com/docs/installation/linux)

Follow [these steps](https://www.stereolabs.com/docs/ros2) to create a ZED pacakage in the ROS2 workspace

Follow [these steps](https://index.ros.org/p/cv_bridge/) to install the OpenCV Bridge

Test the ZED node with the following line in terminal

```
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
```

Add the following lines to the .bashrc so it should look like this at the bottom

```
# Goes back to root
cd

# Checks if it is in a virtual environment and deactivates it
if [[ -n "$VIRTUAL_ENV" ]]; then
    deactivate
fi

# Sources the .venv and ros dependencies
source .venv/bin/activate
source /opt/ros/humble/setup.bash
source /home/emrl-3172/ros2_ws/install/local_setup.bash

# Echo messages to the user
echo ZED Node: 
echo ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
echo
```

### 5. Clone the Repository
Clone the repository with the following line in terminal while in ~/ros2_ws/src

```
git clone <repo>
```

### 6. Setup the Pixhawk Communication
TODO

### 7. Setup the Depth Sensing
TODO

## Field Testing
When in the field, proceed with the following instructions to fly the drone with the current capabilities

### 1. Start the Jetson
Make the sure the Jetson and the router are both powered then SSH into the Jetson with VS Code by using the following information.

```
Jetson Name: emrl-3172
IP Address: TODO
```

### 2. Start the UDP Ground Station
With a laptop, run the GroundStation > UDP_Reciever.py

### 3. Start Mission Planner
With a laptop, run Mission Planner, send an updated mission to the drone, and connect the controller

### 4. Begin the Process
Begin main.py and the tmux session below
```
tmux

python main.py
```

Once you see the checks complete on the Ground Station detach the tmux session with Ctrl+B, then D and safely disconnect from the SSH with ```exit```

If you need to reattach to the tmux session, SSH into the Jetson again and run ```tmux attach```

### 5. Takeoff and Obstacle Avoidance
Manually takeoff the drone and once it is stable in flight, switch it to GUIDED mode in Mission Planner

You can now monitor all the logged messages with the Ground Station while the drone is connected to the router