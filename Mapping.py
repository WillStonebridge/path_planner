import json
import matplotlib.pyplot as plt
import matplotlib.path as mpath
import math
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
        for point in boundarypoints:
            point[0], point[1] = self.decimal_to_cartesian(point[0], point[1], self.min_lat, self.min_lon)
        for obstacle in obstacles:
            obstacle[0], obstacle[1] = self.decimal_to_cartesian(obstacle[0], obstacle[1], self.min_lat, self.min_lon)
        self.map_y_width, self.map_x_width = 0, 0                       # widths of obstacle map
        self.max_x, self.max_y = 0, 0                                  
        self.min_x, self.min_y = 0, 0                                  
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
    def decimal_to_cartesian(self, lat1, lon1, lat2, lon2):
        d = self.calc_haversine(lat2, lon2, lat1, lon1)
        bearing = self.calc_bearing(lat2, lon2, lat1, lon1)
        x = round(d * math.cos(bearing))
        y = round(d * math.sin(bearing))
        return x, y
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
        # print(f"self.min_x: {self.min_x}\n")
        # print(f"self.min_y: {self.min_y}\n")
        # print(f"self.max_x: {self.max_x}\n")
        # print(f"self.max_y: {self.max_y}\n")
        # print(f"self.map_x_width: {self.map_x_width}\n")
        # print(f"self.map_y_width: {self.map_y_width}\n")

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
    
def main():
    mission_data = "AUVSI2021waypoints.json"
    resolution = 10
    buffer = 5
    file = json.load(open(mission_data, 'rb'))
    waypoints = []
    boundarypoints = []
    obstacles = []
    for waypoint in file["waypoints"]:
            waypoints += [[waypoint["latitude"],
                                waypoint["longitude"], waypoint["altitude"]]]

    for boundarypoint in file["boundaryPoints"]:
        boundarypoints += [[boundarypoint["latitude"],
                            boundarypoint["longitude"]]]
    boundarypoints.append(list(boundarypoints[0]))

    for obstacle in file["obstacles"]:
        obstacles += [[obstacle["latitude"],
                        obstacle["longitude"], obstacle["radius"]]]
    map = Map(resolution, boundarypoints, obstacles, buffer)
    invalid = []
    for x in range(len(map.obstacle_map)):
        for y in range(len(map.obstacle_map[x])):
            if not map.obstacle_map[x][y]:
                invalid.append([map.transform_to_real_position(x, map.min_x), map.transform_to_real_position(y, map.min_y)])
    fig, ax = plt.subplots()

    for node in invalid:
        plt.plot(node[0], node[1], '.k')
    ax.set_xlim(map.min_x - 10, map.max_x + 10)
    ax.set_ylim(map.min_y - 10, map.max_y + 10)
    plt.grid(True)
    plt.axis("equal")
    plt.show()

if __name__ == "__main__":
    main()    
    
