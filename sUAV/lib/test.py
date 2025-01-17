from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time
import threading

# Connect to the vehicle
print("Connecting to vehicle...")
vehicle = connect('/dev/ttyTHS1', baud=57600, wait_ready=True)

# Global variables
mission_count = 0
last_update_time = 0
UPDATE_INTERVAL = 2  # Minimum seconds between updates
in_auto_mode = False  # Track if we're in AUTO mode

def update_mission_count():
    """Update mission count in a separate thread"""
    global mission_count, last_update_time
    
    current_time = time.time()
    # Only update if enough time has passed since last update
    if current_time - last_update_time > UPDATE_INTERVAL:
        try:
            cmds = vehicle.commands
            cmds.download()
            cmds.wait_ready()
            mission_count = cmds.count
            last_update_time = current_time
            print(f"Updated mission count: {mission_count} waypoints")
        except Exception as e:
            print(f"Failed to update mission count: {e}")

def check_mission_available():
    """Check if there are any mission waypoints loaded"""
    # Start update in separate thread
    update_thread = threading.Thread(target=update_mission_count)
    update_thread.daemon = True
    update_thread.start()
    return mission_count > 0

def switch_to_mission_mode():
    """Switch vehicle to AUTO mode to start the mission"""
    global in_auto_mode
    vehicle.mode = VehicleMode("AUTO")
    in_auto_mode = True
    print("Switched to AUTO mode for mission execution")

def switch_to_manual_mode():
    """Switch vehicle back to STABILIZE mode"""
    global in_auto_mode
    vehicle.mode = VehicleMode("STABILIZE")
    in_auto_mode = False
    print("Switched back to STABILIZE mode - manual control")

try:
    print("Monitoring RC channels...")
    # Initial mission download
    update_mission_count()
    
    @vehicle.on_message('RC_CHANNELS')
    def rc_listener(self, name, message):
        if message.chan12_raw > 1000:
            print("Channel 12 pressed!")
            
            if not in_auto_mode:
                # Check if mission is available
                if check_mission_available():
                    print(f"Mission available with {mission_count} waypoints")
                    switch_to_mission_mode()
                else:
                    print("No mission waypoints available!")
            else:
                # If already in AUTO mode, switch back to manual
                switch_to_manual_mode()
                
    while True:
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nStopping...")
finally:
    vehicle.close()