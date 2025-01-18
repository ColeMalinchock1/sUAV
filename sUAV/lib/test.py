#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
from dronekit import connect, VehicleMode
import time
import argparse
import logging
from datetime import datetime

"""
This program is for testing that the drone is able to do an auto take off to then complete a mission and then land
To run use the command python test.py --connect /dev/ttyTHS1

Author: Cole Malinchock
Last Revision: 1/18/25
"""

# Set up logging
def setup_logging():
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    logging.basicConfig(
        filename=f'drone_log_{timestamp}.log',
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    # Add a stream handler if you want to see logs in terminal as well
    console = logging.StreamHandler()
    console.setLevel(logging.INFO)
    logging.getLogger('').addHandler(console)

def arm_and_takeoff(vehicle, target_altitude):
    """
    Arms vehicle and fly to targetAltitude.
    """

    # Completes a pre arm check
    logging.info("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        logging.info(" Waiting for vehicle to initialise...")
        time.sleep(1)
    
    # Arms the motors in guided mode
    logging.info("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Waits for the GPS to lock with enough satellites
    while vehicle.gps_0.fix_type < 3:
        logging.info("Waiting for GPS lock...")
        time.sleep(1)

    # Waits for the vehicle to arm
    while not vehicle.armed:      
        logging.info(" Waiting for arming...")
        time.sleep(1)

    # Take off to target altitude
    logging.info("Taking off!")
    vehicle.simple_takeoff(target_altitude)
    
    # Wait until the vehicle reaches a safe height
    while True:
        logging.info(f" Altitude: {vehicle.location.global_relative_frame.alt}")      
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
            logging.info("Reached target altitude")
            break
        time.sleep(1)

def run_mission(vehicle):
    """
    Executes the pre-loaded mission on the Pixhawk
    """
    logging.info("Starting pre-loaded mission")
    
    # Switch to AUTO mode to execute the mission
    vehicle.mode = VehicleMode("AUTO")
    
    # Wait for mode switch
    while not vehicle.mode.name == "AUTO":
        logging.info("Waiting for AUTO mode...")
        time.sleep(1)
    
    logging.info("AUTO mode confirmed - executing mission")
    
    # Monitor mission progress
    while True:
        # If mission is complete, vehicle will switch to RTL or LAND
        if vehicle.mode.name != "AUTO":
            logging.info("Mission completed - vehicle has left AUTO mode")
            break
            
        logging.info(f"Current altitude: {vehicle.location.global_relative_frame.alt}")
        time.sleep(2)

def main():
    """
    Main function ran at start up to go through the necessary sequences
    """
    # Set up logging
    setup_logging()

    # Set up argument parsing to make sure that a pixhawk is connected to a specific /dev/
    parser = argparse.ArgumentParser(description='Connect to drone, takeoff, and complete a mission.')
    parser.add_argument('--connect', 
                       help="Vehicle connection target string. If not specified, SITL automatically started and used.")
    parser.add_argument('--mission',
                   action='store_true',  # This makes it a boolean flag
                   help="Enable mission mode for the drone")
    args = parser.parse_args()

    # Gets the port from the command line argument
    connection_string = args.connect
    # Gets the boolean if the mission is enabled
    mission_enabled = args.mission

    # Start SITL if no connection string specified
    if not connection_string:
        import dronekit_sitl
        sitl = dronekit_sitl.start_default()
        connection_string = sitl.connection_string()

    # Connect to the Vehicle
    logging.info("\nConnecting to vehicle on: %s" % connection_string)
    vehicle = connect(connection_string, wait_ready=True, baud=57600, timeout=60)

    # Log some basic vehicle information
    logging.info("\nVehicle parameters:")
    logging.info(f" GPS: {vehicle.gps_0}")
    logging.info(f" Battery: {vehicle.battery}")
    logging.info(f" Last Heartbeat: {vehicle.last_heartbeat}")
    logging.info(f" Is Armable?: {vehicle.is_armable}")
    logging.info(f" System status: {vehicle.system_status.state}")
    logging.info(f" Mode: {vehicle.mode.name}")

    # Arm and take off
    target_altitude = 2.0  # meters
    arm_and_takeoff(vehicle, target_altitude)

    # Switch to STABILIZE mode
    logging.info("\nSwitching to STABILIZE mode")
    vehicle.mode = VehicleMode("STABILIZE")
    time.sleep(2)  # Give time for mode switch

    # Complete the mission if enabled
    if mission_enabled:
        run_mission(vehicle)

    # Switching to LAND mode
    logging.info("\nSwitching to LAND mode")
    vehicle.mode = VehicleMode("LAND")
    time.sleep(2) # Give time for mode switch

    # Keep the script running to maintain vehicle connection
    try:
        while True:
            logging.info("\nCurrent Status:")
            logging.info(f" Mode: {vehicle.mode.name}")
            logging.info(f" Altitude: {vehicle.location.global_relative_frame.alt}")
            logging.info(f" Battery: {vehicle.battery}")
            time.sleep(2)
    except KeyboardInterrupt:
        logging.info("\nUser interrupted")
    finally:
        # Close vehicle object
        logging.info("\nClosing vehicle connection")
        vehicle.close()

if __name__ == '__main__':
    # Waits 20 seconds before starting up to give time to back away
    time.sleep(20)

    # Runs main on start up
    main()