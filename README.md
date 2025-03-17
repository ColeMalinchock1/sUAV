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
- Use TCP Socket for ground station logging with vehicle
- Use ROS2 instead of mavutil
- Find a lidar that will work better
- Use PID to control the obstacle avoidance after completing simple maneuvers with current communication

