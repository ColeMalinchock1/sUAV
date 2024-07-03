#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Int64
from px4_msgs.msg import VehicleCommand, TrajectorySetpoint, TimesyncStatus

import time
import threading
import math

class PixhawkController(Node):
    def __init__(self):
        super().__init__('pixhawk_controller')

        # Setting up the quality of service for the publishers and subscribers
        qos_profile = QoSProfile(
        reliability = QoSReliabilityPolicy.BEST_EFFORT,
        history = QoSHistoryPolicy.KEEP_LAST,
        depth = 5
        )

        # Publishers for sending messages to the PX
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.offboard_setpoint_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)

        # Subscribers for receiving messages from the PX
        self.timesync_subscriber = self.create_subscription(TimesyncStatus, '/fmu/out/timesync_status', self.timesync_callback, qos_profile)

        # Initializing the time sync between the PX and Orin
        self.timestamp = 0
        self.armed = False
        self.offboard_mode = False
        self.get_logger().info('Waiting for timesync...')

        # Looping until timestamp is received
        while self.timestamp == 0:
            rclpy.spin_once(self)

        # Arm the vehicle on initialization
        self.arm_vehicle()

        # Make the vehicle takeoff
        self.takeoff()

        # Set it so it is offboard
        self.set_offboard_mode()

        # Rotate the vehicle 360 degrees
        self.rotate_360()

        # Land the vehicle
        self.land()

        # Final message on completion
        self.get_logger().info('Complete')

    # Callback to receive the timestamp from the PX
    def timesync_callback(self, msg):
        self.timestamp = msg.timestamp

    # Send a command to the PX so that it arms the vehicle
    def arm_vehicle(self):
        self.get_logger().info('Arming vehicle')

        # Create and format the Vehicle Command message
        arm_command = VehicleCommand()
        arm_command.timestamp = self.timestamp
        arm_command.param1 = 1.0
        arm_command.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        arm_command.target_system = 1
        arm_command.target_component = 1

        # Publish the command and then wait
        self.vehicle_command_publisher.publish(arm_command)
        self.armed = True
        time.sleep(2)

    # Send a command to make the vehicle take off to a target location
    def takeoff(self):
        self.get_logger().info('Take off')
        
        # Create and format the Vehicle Command message
        takeoff_command = VehicleCommand()
        takeoff_command.timestamp = self.timestamp
        takeoff_command.command = VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF
        takeoff_command.param5 = 47.39 # Target Latitude
        takeoff_command.param6 = 8.4324 # Target Longitude
        takeoff_command.param7 = 10.0 # Target Altitude
        takeoff_command.target_system = 1
        takeoff_command.target_component = 1

        # Publish the command then wait
        self.vehicle_command_publisher.publish(takeoff_command)
        time.sleep(10)

    # Sets the offboard mode to on
    # This allows for the vehicle to do more manual movements
    def set_offboard_mode(self):
        self.get_logger().info('Setting offboard mode')

        # Create and format the Vehicle Command message
        offboard_command = VehicleCommand()
        offboard_command.timestamp = self.timestamp
        offboard_command.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        offboard_command.param1 = 1.0 # Custom mode
        offboard_command.param2 = 6.0 # Offboard mode
        offboard_command.target_system = 1
        offboard_command.target_component = 1
        
        # Publish the command then wait
        self.vehicle_command_publisher.publish(offboard_command)
        self.offboard_mode = True
        time.sleep(2)

    # Rotates the yaw of the vehicle 360 degrees
    def rotate_360(self):
        self.get_logger().info('Rotating 360 degrees')

        # Initialize values
        yaw_angle = 0
        rotation_rate = 30 # Degrees per second
        duration = 360 / rotation_rate

        # Loop through the duration turning the vehicle to a new setpoint
        for i in range(int(duration * 10)):

            # Increment the setpoint of the yaw
            yaw_angle += rotation_rate * 0.1
            yaw_angle_rad = math.radians(yaw_angle)

            # Create and format Trajectory Setpoint message
            setpoint = TrajectorySetpoint()
            setpoint.timestamp = self.timestamp
            setpoint.yaw = yaw_angle_rad
            setpoint.position = [0.0, 0.0, -10.0]

            # Publish the new setpoint then wait
            self.offboard_setpoint_publisher.publish(setpoint)
            self.get_logger().info(f'Setting yaw: {yaw_angle} degrees')
            time.sleep(0.1)

    # Lands the vehicle
    def land(self):
        self.get_logger().info('Landing')

        # Create and format Vehicle Command message
        land_command = VehicleCommand()
        land_command.timestamp = self.timestamp
        land_command.command = VehicleCommand.VEHICLE_CMD_NAV_LAND
        land_command.target_system = 1
        land_command.target_component = 1

        # Publish the command then wait
        self.vehicle_command_publisher.publish(land_command)
        time.sleep(10)
 

def main(args=None):
    rclpy.init(args=args)
    pixhawk_controller = PixhawkController()
    rclpy.spin(pixhawk_controller)
    pixhawk_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# def callback(data):
#     global test_data
#     test_data = data
                
# def main(args=None):
#     global test_data

#     rclpy.init(args=args)



#     px_node = Node("pixhawk_node")
#     px_node.create_subscription(VehicleStatus, "/fmu/out/vehicle_status", callback, qos_profile)


#     thread = threading.Thread(target=rclpy.spin, args=(px_node, ), daemon=True)
#     thread.start()

#     rate = px_node.create_rate(20, px_node.get_clock())
#     state = 0
#     while rclpy.ok():
#         if state == 0:
#             initialize()
#         elif state == 1:
#             takeoff()
#         elif state == 2:
#             rotate_360()
#         elif state == 3:
#             land()
#         rate.sleep()

#     rclpy.spin(px_node)
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()