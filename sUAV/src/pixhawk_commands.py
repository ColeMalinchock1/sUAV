
# Import necessary libraries
from sUAV.lib.constants import *

# If not in debug mode then import pixhawk communication libraries 
if not DEBUG_MODE:
    from dronekit import connect, VehicleMode, LocationGlobal
    from pymavlink import mavutil

class PixhawkCommands():
    """Class to handle the pixhawk commands and communication between the on-board Jetson and pixhawk"""

    def __init__(self, logger):
        """
        Init function when the pixhawk commands is started
        
        logger as the logger being passed to handle messages about the system
        """

        self.address = PIXHAWK_ADDRESS
        self.baud_rate = PIXHAWK_BAUD
        self.wait_ready = PIXHAWK_WAIT_READY
        self.vehicle = None
        self.logger = logger
        self._setup_pixhawk() # Runs the setup function to create the vehicle
        
    def _setup_pixhawk(self):
        """Sets up the vehicle to be connected with the address and baud rate"""

        self.logger.info("Connecting to vehicle...")

        # Connects the vehicle
        self.vehicle = connect(self.address, baud=self.baud_rate, wait_ready=self.wait_ready)

        # Checks if the connection was made correctly to the pixhawk
        if self.vehicle:
            self.logger.info("Connected to vehicle")
            # Checks if there is a heartbeat with the pixhawk
            if self.check_satellites() and self.check_battery():
                self.logger.info("Vehicle health validated")
                return True
            else:
                self.close_vehicle()
                self.logger.critical("Poor vehicle health")
        else:
            self.logger.critical("Unable to setup vehicle")

        return False
    
    def check_satellites(self):
        """Checks the number of satellites seen by the gps"""

        # Gets the number of satellites from the gps
        num_satellites = self._get_satellites()

        # Checks if the number of satellites is sufficient
        if num_satellites >= MINIMUM_SATELLITES:
            self.logger.info(f"Sufficient number of satellites - Counted: {num_satellites}, Required: {MINIMUM_SATELLITES}")
            return True
        else:
            self.logger.critical(f"Insufficient number of satellites - Counted: {num_satellites}, Required: {MINIMUM_SATELLITES}")
            return False
        
    def check_battery(self):
        """Checks that the battery is at a safe level"""

        battery_level = self._get_battery_level()
        if battery_level >= MINIMUM_BATTERY_LEVEL:
            self.logger.info(f"Sufficient battery level - Measured: {battery_level}, Required: {MINIMUM_BATTERY_LEVEL}")
            return True
        else:
            self.logger.info(f"Insufficient battery level - Measured: {battery_level}, Required: {MINIMUM_BATTERY_LEVEL}")
            return False
        
    def _get_satellites(self):
        """Gets the number of satellites"""
        return self.vehicle.gps_0.fix_type

    def _get_battery_level(self):
        """Gets the battery level"""
        return self.vehicle.battery.level

    def close_vehicle(self):
        """Closes the vehicle connection"""
        self.vehicle.close()

    def command_YAW(self, degrees):
        """
        Rotates the drone a specified amount
        
        degrees ranging from [-180, 180]
        """

        current_yaw = self.vehicle.heading

        self.logger.info(f"Yaw command - Current yaw: {current_yaw}, Final yaw: {current_yaw + degrees}")

        # Checks if the rotation is to the left or right
        if (degrees < 0):
            degrees = abs(degrees)
            direction = -1
        else:
            direction = 1

        # Formats the message to be sent to the pixhawk
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, # Yaw command
            0,
            degrees, #
            YAW_SPEED, # The speed of the rotation [deg/s]
            direction, # The direction of the rotation [1 = right, -1 = left]
            1,
            0, 0, 0
        )

        # Sends the message to the pixhawk
        self._send_command(msg)

    def command_XYA(self, velocity_x, velocity_y, altitude):
        """
        Moves the drone in a specified direction as specified velocities
        """

        self.logger.info(f"Adjusting velocity - Velocity X: {velocity_x}, Velocity Y: {velocity_y}, Altitude: {altitude}")

        # Formats the message to be sent to the pixhawk
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            MOVEMENT_MASK,
            0, 0, altitude,
            velocity_x, velocity_y, 0,
            0, 0, 0,
            0, 0
        )

        # Sends the message to the pixhawk
        self._send_command(msg)

    def _send_command(self, msg):
        """
        Sends the message to the pixhawk

        msg as the formatted message to be sent to the drone
        """

        self.vehicle.send_mavlink(msg)

        self.logger.info("Command sent")

    def get_mode(self):
        return self.vehicle.mode.name
