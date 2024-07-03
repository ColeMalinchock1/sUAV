# sUAV Project
This project is to design a ROS2 system that allows a Pixhawk 4 to communicate with an on-board NVIDIA Jetson Orin Nano to control the flight of a small UAV. The vehicle will also include a lidar that is capable of detecting objects in the way of its current path and avoid them.

## ROS2 Nodes
The current ROS2 nodes used with this project are:
- MicroXRCE Agent Node
- Pixhawk Node
- User Interface Node

Each node is described in detail below along with necessary commands to run each node

### MicroXRCE Agent Node
This node is intended to communicate with the Pixhawk via UART connection from the NVIDIA Jetson Orin Nano. It outputs a series of topics that can be found [here](https://github.com/PX4/px4_msgs)

```script
sudo MicroXRCEAgent serial --dev /dev/ttyTHS1 -b 921600
```

### Pixhawk Node
This node is used to subscribe to select topics from the MicroXRCE Agent Node to know the current position and then publishes commands to the MicroXRCE Agent Node regarding flight path and changes.

```script
python pixhawk_sim_node.py
```

### User Interface Node
This node is used to publish a setpoint for the vehicle to go to and is then received by the pixhawk node to then go to that point. It also receives the current position from the pixhawk node to monitor it.

```script
python ui_node.py
```

## Pixhawk Setup
The pixhawk parameters need to be setup to allow for a ROS2 communication with the on board NVIDIA Jetson Orin Nano

### Changing Parameters
These parameters will be changed to the following values to allow the Pixhawk to communicate with the NVIDIA Jetson Orin Nano. These were sourced from [here](https://docs.px4.io/main/en/companion_computer/pixhawk_rpi.html)

```script
MAV_1_CONFIG = TELEM2
UXRCE_DDS_CFG = 0 (Disabled)
SER_TEL2_BAUD = 57600
```

### Starting the client
Only needs to be done once for each Pixhawk but will initialize the client to communicate via UART. This is to be done in QGroundControl or Mission Planner terminal.

```script
uxrce_dds_client start -t serial -d /dev/ttyS2 -b 921600
```

In the case of any issues, you can use this to debug the current state of the client:

```script
uxrce_dds_client status
```
