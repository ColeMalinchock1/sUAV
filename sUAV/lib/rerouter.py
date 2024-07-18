THRESHOLD = 0.2
MAX_THRESHOLD = 3
STOP_COMMAND = "STOP"

class Rerouter():

    def __init__(self):
        self.current_position = self.scan = None

    def obstacle_detected(self, current_position, scan):
        self.current_position = current_position
        self.scan = scan

        opening = self.find_opening(scan)

        if opening == -1:
            return STOP_COMMAND

        self.vectorize(scan)

    def find_opening(self, scan):
        """Finds the opening in the scan if there is one"""

        # Initialize all items tracked in the for loop
        opening = -1
        longest_streak = None
        new_streak = (0, 0)
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
                    new_streak = (i, i)
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
            opening = (longest_streak[0] + longest_streak[1]) / 2
    
        return opening
            
        


    def vectorize(self, scan):
        
        vectorized_scan = []

        previous_z = None

        for z in scan:
            if previous_z is not None:
                delta_z = z - previous_z
                if delta_z < THRESHOLD:
                    delta_z = 0
            
                vectorized_scan.append(delta_z)
                
            else:
                previous_z = z

