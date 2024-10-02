from pymavlink import mavutil
from lib.coords_to_cartesian import CoordsToCartesian as c2c

def connect_pixhawk(serial_port='/dev/ttyTHS1', baudrate=57600):
    # Connect to the Pixhawk via the specified serial port and baudrate
    master = mavutil.mavlink_connection(serial_port, baud=baudrate)
    master.wait_heartbeat()  # Wait for the heartbeat signal from the Pixhawk
    print("Connected to Pixhawk")
    return master

def get_current_location(master, timeout=10):
    # Wait for the GLOBAL_POSITION_INT message
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=timeout)
    if msg is None:
        print("Failed to receive GLOBAL_POSITION_INT")
        return None
    
    # Extract latitude, longitude, and altitude from the message
    lat = msg.lat
    lon = msg.lon
    alt = msg.alt
    hdg = msg.hdg

    location = {
        'latitude': lat,
        'longitude': lon,
        'altitude': alt,
        'heading': hdg
    }

    return location

def request_waypoints(master, timeout=10):
    # Request the total number of waypoints
    master.waypoint_request_list_send()

    # Wait for the MISSION_COUNT message
    msg = master.recv_match(type='MISSION_COUNT', blocking=True, timeout=timeout)
    if msg is None:
        print("Failed to receive MISSION_COUNT")
        return []

    waypoint_count = msg.count
    print(f"Received MISSION_COUNT: {waypoint_count}")

    waypoints = []

    for i in range(waypoint_count):
        # Request individual waypoint
        master.waypoint_request_send(i)
        
        # Wait for the corresponding MISSION_ITEM message
        msg = master.recv_match(type='MISSION_ITEM', blocking=True, timeout=timeout)
        if msg is None:
            print(f"Failed to receive MISSION_ITEM for waypoint {i}")
            break

        waypoint = {
            'seq': msg.seq,
            'lat': msg.x,  # Convert to degrees
            'lon': msg.y,  # Convert to degrees
            'alt': msg.z,  # Convert to meters
        }

        waypoints.append(waypoint)
        print(f"Received waypoint: {waypoint}")

    return waypoints


def send_waypoints(master, waypoints):
    # Send the updated list of waypoints to the Pixhawk
    master.waypoint_count_send(len(waypoints))
    
    for wp in waypoints:
        print(f"Sending waypoint: seq={wp['seq']}, lat={wp['lat']}, lon={wp['lon']}, alt={wp['alt']}")
        master.mav.mission_item_send(master.target_system,
                                     master.target_component,
                                     wp['seq'],
                                     mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                     mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                                     0, 1, 0, 0, 0, 0, wp['lat'], wp['lon'], wp['alt'])


def main():
    # Connect to the Pixhawk
    pixhawk = connect_pixhawk()

    # Request the current waypoints
    waypoints = request_waypoints(pixhawk)

    # Converts the waypoint format into a basic 2d list
    waypoint_list = [[wp['lat'], wp['lon'], wp['alt']] for wp in waypoints]

    print("Current waypoints: ", waypoint_list)

    current_position = get_current_location(pixhawk)
    
    if current_position is None:
        current_position = (35.770759, -78.674728, 100.0)
        print(f"Setting current position as {current_position}")
        converter = c2c(current_position[0], current_position[1])
    else:
        converter = c2c(current_position[0], current_position[1])


    incomplete = True

    while incomplete:
        case = input("What do you want to do?\n1. Add Waypoint\n2. Remove Waypoint\n3. Remove All Waypoints\n4. Nothing\n")

        if case == "1":
            
            print(f"Current waypoints: {waypoint_list}")
            idx = int(input("Enter position where you want to add a waypoint: \n"))

            if len(waypoint_list) == 0:

                print(f"Adding point to new list")

            elif idx == 0:

                print(f"Adding point before {waypoint_list[idx]}")

            elif idx == len(waypoint_list) - 1:

                print(f"Adding point after {waypoint_list[idx]}")
                
            else:

                print(f"Adding point between {waypoint_list[idx - 1]} and {waypoint_list[idx]}")

            x = float(input("Enter relative x (meters): "))
            y = float(input("Enter relative y (meters): "))

            if len(waypoint_list) == 0 or idx == 0:
                previous_position = converter.latlon_to_xy(current_position[0], current_position[1])
            else:
                previous_position = converter.latlon_to_xy(waypoint_list[idx - 1][0], waypoint_list[idx - 1][1])

            print(previous_position)

            previous_position[0] = previous_position[0] + x
            previous_position[1] = previous_position[1] + y

            new_waypoint = converter.xy_to_latlon(previous_position[0], previous_position[1])

            new_waypoint.append(current_position[2])

            temp_waypoints = waypoint_list

            temp_waypoints.insert(idx, new_waypoint)

            print("New waypoints: ", temp_waypoints)

            confirmation = input("Press ENTER to confirm or X to cancel change\n")

            if confirmation != "x" or confirmation != "X":
                print("Waypoint added successfully")
                waypoint_list = temp_waypoints

        elif case == "2":

            incomplete = True

            while incomplete:

                print(f"Current waypoints: {waypoint_list}")
                idx = int(input("Enter position where you want to remove a waypoint: \n"))
                
                if idx >= 0 and idx < len(waypoint_list) - 1:

                    print(f"Removing this point {waypoint_list[idx]}")

                else:
                    print("Invalid point")

                if idx == 0:
                    temp_waypoints = waypoint_list[1, len(waypoint_list) - 1]
                elif idx == len(waypoint_list) - 1:
                    temp_waypoints = waypoint_list[0, len(waypoint_list) - 2]
                else:
                    temp_waypoints = waypoint_list[0:idx-1,idx + 1:len(waypoint_list)]

                print("New waypoints: ", temp_waypoints)

                confirmation = input("Press enter to confirm or x to cancel change")

                if confirmation != "x" or confirmation != "X":
                    print("Removed waypoint successfully")
                    waypoint_list = temp_waypoints
        
        elif case == "3":

            confirmation = input("Press enter to confirm or x to cancel change")

            if confirmation != "x" or confirmation != "X":
                print("Waypoints cleared successfully")
                waypoint_list = []
            
        loop = input("Would you like to make anymore changes? (y/n)")

        if loop == "n":
            incomplete = False
        else:
            incomplete = True


    # Create a new waypoint and add it at the front
    #new_wp = {'seq': 0, 'lat': 1.0, 'lon': 2.0, 'alt': 3.0}
    #waypoints = add_waypoint(waypoints, new_wp)
    print(waypoint_list)
    waypoints = [{'seq': i, 'lat': wp[0], 'lon': wp[1], 'alt': wp[2]} for i, wp in enumerate(waypoint_list)]
    # Send the updated waypoints back to the Pixhawk
    print(waypoints)
    send_waypoints(pixhawk, waypoints)

    print("All waypoints sent successfully")

if __name__ == '__main__':
    main()
