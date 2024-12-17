from geopy import distance
import math

class CoordsToCartesian():
    # The radius of Earth in meters
    R = 6378137

    def __init__(self, lat_origin, lon_origin):
        # Sets the origin of the coordinate system
        self.origin = (lat_origin, lon_origin)

    def haversine(self, lat, lon):
        # Finds the distance between the lat-longs and converts it to meters
        d_lat = math.radians(lat - self.origin[0])
        d_lon = math.radians(lon - self.origin[1])

        a = math.sin(d_lat / 2) ** 2 + math.cos(math.radians(self.origin[0])) * math.cos(math.radians(lat)) * math.sin(d_lon / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance_m = self.R * c

        return distance_m

    def latlon_to_xy(self, lat, lon):
        lat = float(lat)
        lon = float(lon)

        x = self.haversine(self.origin[0], lon)
        y = self.haversine(lat, self.origin[1])

        if lon < self.origin[1]:
            x = -x
        if lat < self.origin[0]:
            y = -y

        return [x, y]
    
    def xy_to_latlon(self, x, y):
        # Calculate the latitude from y
        d_lat = (y / self.R) * (180 / math.pi)
        lat = self.origin[0] + d_lat
        
        # Calculate the longitude from x using the calculated latitude
        lat_radians = math.radians(lat)
        d_lon = (x / (self.R * math.cos(lat_radians))) * (180 / math.pi)
        lon = self.origin[1] + d_lon
        
        return [lat, lon]

    def compass_heading_to_yaw(self, heading):
        # Convert compass heading to yaw angle
        yaw = (heading - 90) % 360
        return yaw
    
    def yaw_to_compass_heading(self, yaw):
        # Convert yaw angle to compass heading
        compass_heading = (90 - yaw) % 360
        return compass_heading
