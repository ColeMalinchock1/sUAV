#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
from dronekit import connect, VehicleMode
import time
import argparse

def arm_and_takeoff(vehicle, aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """
    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)
        
    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:      
        print(" Waiting for arming...")
        time.sleep(1)

    vehicle.mode = VehicleMode("STABILIZE")
    vehicle.armed = True

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude
    count = 0
    # Wait until the vehicle reaches a safe height
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)      
        count += 1
        if count == 30:
            vehicle.mode = VehicleMode("LOITER")
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def main():
    # Set up argument parsing
    parser = argparse.ArgumentParser(description='Connect to drone, takeoff, and enter stabilize mode.')
    parser.add_argument('--connect', 
                       help="Vehicle connection target string. If not specified, SITL automatically started and used.")
    args = parser.parse_args()

    connection_string = args.connect

    # Start SITL if no connection string specified
    if not connection_string:
        import dronekit_sitl
        sitl = dronekit_sitl.start_default()
        connection_string = sitl.connection_string()

    # Connect to the Vehicle
    print("\nConnecting to vehicle on: %s" % connection_string)
    vehicle = connect(connection_string, wait_ready=True, baud=57600, timeout=60)

    # Print some basic vehicle information
    print("\nVehicle parameters:")
    print(" GPS: %s" % vehicle.gps_0)
    print(" Battery: %s" % vehicle.battery)
    print(" Last Heartbeat: %s" % vehicle.last_heartbeat)
    print(" Is Armable?: %s" % vehicle.is_armable)
    print(" System status: %s" % vehicle.system_status.state)
    print(" Mode: %s" % vehicle.mode.name)

    # Arm and take off
    target_altitude = 3.5  # meters
    arm_and_takeoff(vehicle, target_altitude)

    # Switch to STABILIZE mode
    print("\nSwitching to STABILIZE mode")
    vehicle.mode = VehicleMode("STABILIZE")
    time.sleep(2)  # Give time for mode switch

    # Keep the script running to maintain vehicle connection
    try:
        while True:
            print("\nCurrent Status:")
            print(" Mode: %s" % vehicle.mode.name)
            print(" Altitude: %s" % vehicle.location.global_relative_frame.alt)
            print(" Battery: %s" % vehicle.battery)
            time.sleep(2)
    except KeyboardInterrupt:
        print("\nUser interrupted")
    finally:
        # Close vehicle object
        print("\nClosing vehicle connection")
        vehicle.close()

if __name__ == '__main__':
    main()