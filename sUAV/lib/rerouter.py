import math

THRESHOLD = 0.2
MAX_THRESHOLD = 1
STOP_COMMAND = "STOP"
VEHICLE_SIZE = 5

class Rerouter():

    def __init__(self):
        self.scan = self.y_distances = None

    def obstacle_detected(self, scan, y_distances):
        
        self.scan = scan
        self.y_distances = y_distances

        opening = self.find_opening(self.scan)

        if opening == -1 or len(opening) < VEHICLE_SIZE:
            return STOP_COMMAND

        steer = self.steer_adjuster(self.scan, opening, self.y_distances)

        return steer

        

    def find_opening(self, scan):
        """Finds the opening in the scan if there is one"""

        # Initialize all items tracked in the for loop
        opening = -1
        longest_streak = None
        new_streak = [0, 0]
        start_new_streak = True

        # Loops through all depths on the scan
        for i in range(len(scan)):
            
            # Checks if the scan at the position is above the max threshold and then sets it as the max threshold and updates the streak
            # Else updates the streaks
            if scan[i] >= MAX_THRESHOLD:
                scan[i] = MAX_THRESHOLD

                # Checks if a new streak is started and starts it at that point
                # Else extends the streak to the current i
                if start_new_streak:
                    start_new_streak = False
                    new_streak = [i, i]
                else:
                    new_streak[1] = i
            else:
                start_new_streak = True
                if longest_streak is not None:
                    if longest_streak[1] - longest_streak[0] < new_streak[1] - new_streak[0]:
                        longest_streak = new_streak
                else:
                    longest_streak = new_streak

        if longest_streak is not None:
            opening = list(range(longest_streak[0], longest_streak[1] + 1))

        return opening
            

    def steer_adjuster(self, scan, opening, y_distances):
        
        # Checks if the vehicle has enough space on its left or right and can continue going straight
        # Else if opening is to the left of the scan set k as -1
        # Else it is assumed the opening is to the right of the center of the scan set k as 1
        if int(len(scan) / 2 - VEHICLE_SIZE / 2) in opening or int(len(scan) / 2 + VEHICLE_SIZE / 2) in opening:
            return 0
        elif opening[int(len(opening) / 2)] < int(len(scan) / 2):
            k = 1
        else:
            k = -1
        
        closest_point = None

        # If the opening is to the left get the closest point on the right side
        if k < 0:
            for i in range(len(scan)):
                if i > len(scan) / 2:
                    if closest_point is None:
                        closest_point = i
                    elif scan[closest_point] > scan[i]:
                        closest_point = i
        else:
            for i in range(len(scan)):
                if i < len(scan) / 2:
                    if closest_point is None:
                        closest_point = i
                    elif scan[closest_point] > scan[i]:
                        closest_point = i

        # Steering in degrees
        steer = k * math.degrees(math.atan(y_distances[closest_point]) / MAX_THRESHOLD - scan[closest_point])

        return steer