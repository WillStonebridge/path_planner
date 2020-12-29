import json
import matplotlib.pyplot as plt
import matplotlib.path as mpath
import math
import time 
import numpy as np

class Map:
    """
    Class that uses Json file path of boundaries and obstacles
    to create a environment space
    """
    def __init__(self, resolution, boundarypoints, obstacles, buffer):
        """
        Initialize grid map for a star planning

        resolution: grid resolution [m]
        boundarypoints: list of points that make up a polygonal fence
        obstacles: list of points and respective radiuses 
        buffer: buffer around obstacles
        """ 
        min = np.amin(boundarypoints, axis = 0)
        self.min_lat = min[0]
        self.min_lon = min[1]
        print(self.min_lat, self.min_lon)
        self.map_y_width, self.map_x_width = 0, 0                       # widths of obstacle map
        self.max_x, self.max_y = 0, 0                                   # max lat, max lon
        self.min_x, self.min_y = 0, 0                                   # min lat, min lon
        self.resolution = resolution                                    # resolution
        self.buffer = buffer                                            # buffer
        self.calc_grid_bounds(boundarypoints)                           # finds values for max, min, and widths of lon and lat
        self.obstacle_map = None                                        # map of obstacles (initialized to none)
        self.calc_obstacle_map(obstacles, boundarypoints, buffer)       # creates map of obstacles and boundaries, stores it into a 2D list in obstacle_map

    def calc_bearing(self, lat1, lon1, lat2, lon2):
        lat1 = math.radians(lat1)
        lon1 = math.radians(lon1)
        lat2 = math.radians(lat2)
        lon2 = math.radians(lon2)
        dlat = lat2 - lat1
        dlon = lon2- lon1
        return math.atan2(math.sin(dlon) * math.cos(lat2), math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon))
    def calc_haversine(self, lat1, lon1, lat2, lon2):
        lat1 = math.radians(lat1)
        lon1 = math.radians(lon1)
        lat2 = math.radians(lat2)
        lon2 = math.radians(lon2)
        dlat = lat2 - lat1
        dlon = lon2- lon1
        a = pow(math.sin(dlat / 2),2) + math.cos(lat1) * math.cos(lat2) * pow(math.sin(dlon / 2), 2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        d = 6371e3 * c  #  meters
        return d
    def transform_to_real_position(self, index, min_index):
        """
        transforms index to corresponding "real" position 
        """ 
        position = index * self.resolution + min_index
        return position

    def transform_to_map_position(self, position, min_index):
        """
        transforms "real" position to corresponding index position 
        """ 
        index = (position - min_index) / self.resolution
        return index
    def decimal_to_cartesian(self, coordinates):
        ""          
    def calc_grid_bounds(self, boundarypoints):
        self.min_x = boundarypoints[0][0]
        self.min_y = boundarypoints[0][1]
        for boundarypoint in boundarypoints:
            if boundarypoint[0] > self.max_x:
                self.max_x = boundarypoint[0]
            if boundarypoint[1] > self.max_y:
                self.max_y = boundarypoint[1]
            if boundarypoint[0] < self.min_x:
                self.min_x = boundarypoint[0]
            if boundarypoint[1] < self.min_y:
                self.min_y = boundarypoint[1]
        self.map_y_width = round((self.max_y - self.min_y) / self.resolution)
        self.map_x_width = round((self.max_x - self.min_x) / self.resolution)


    def calc_obstacle_map(self, obstacles, boundarypoints, buffer):
        boundaryPath = mpath.Path(boundarypoints)
        self.obstacle_map = [[False for _ in range(self.map_y_width)] for _ in range(self.map_x_width)]
        for initial_x in range(self.map_x_width):
            x = self.transform_to_real_position(initial_x, self.min_x)
            for initial_y in range(self.map_y_width):
                y = self.transform_to_real_position(initial_y, self.min_y)
                if not boundaryPath.contains_point([x,y]):
                    self.obstacle_map[initial_x][initial_y] = True
                else:
                    for obstacle in obstacles:
                        if math.hypot(obstacle[0] - x, obstacle[1] - y) - self.buffer <= obstacle[2]:
                            self.obstacle_map[initial_x][initial_y] = True
                            break
    
    
    
