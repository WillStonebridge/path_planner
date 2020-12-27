import json
import matplotlib.pyplot as plt
import matplotlib.path as mpath
import math
import time 

class Map:
    """
    Class that uses Json file path of waypoints, boundary vertexes, and obstacles
    to create a environment space
    """
    def __init__(self, resolution, mission_data, buffer):
        """
        Initialize grid map for a star planning

        resolution: grid resolution [m]
        rr: robot radius[m]
        mission_data: path to json file mission data
        """ 
        file = json.load(open(mission_data, 'rb'))
        self.waypoints = []
        self.boundarypoints = []
        self.obstacles = []

        for waypoint in file["waypoints"]:
            self.waypoints += [[waypoint["latitude"],
                           waypoint["longitude"], waypoint["altitude"]]]

        for boundarypoint in file["boundaryPoints"]:
            self.boundarypoints += [[boundarypoint["latitude"],
                                boundarypoint["longitude"]]]
        self.boundarypoints.append(self.boundarypoints[0])

        for obstacle in file["obstacles"]:
            self.obstacles += [[obstacle["latitude"],
                            obstacle["longitude"], obstacle["radius"]]]
        self.map_lon_width, self.map_lat_width = 0, 0
        self.max_lat, self.max_lon = 0, 0
        self.min_lat, self.min_lon = 0, 0
        self.resolution = resolution                    # resolution
        self.calc_grid_bounds(self.boundarypoints)
        self.buffer = buffer                            # creates a "buffer" around obstacles
        self.obstacle_map = None                        # map of obstacles
        self.calc_obstacle_map(self.obstacles, self.boundarypoints, buffer)       # creates map of obstacles and boundaries


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
        self.min_lat = boundarypoints[0][0]
        self.min_lon = boundarypoints[0][1]
        self.min_lon, self.min_lat = 0, 0 
        for boundarypoint in boundarypoints:
            if boundarypoint[0] > self.max_lat:
                self.max_lat = boundarypoint[0]
            if boundarypoint[1] > self.max_lon:
                self.max_lon = boundarypoint[1]
            if boundarypoint[0] < self.min_lat:
                self.min_lat = boundarypoint[0]
            if boundarypoint[1] < self.min_lon:
                self.min_lon = boundarypoint[1]
        self.map_lon_width = round((self.max_lon - self.min_lon) / self.resolution)
        self.map_lat_width = round((self.max_lat - self.min_lat) / self.resolution)
        # print(f"min_lat: {self.min_lat}\nmin_lon: {self.min_lon}\nmax_lat: {self.max_lat}\nmax_lon: {self.max_lon}\nmap_lat_width: {self.map_lat_width}\nmap_lon_width: {self.map_lon_width}\n")


    def calc_obstacle_map(self, obstacles, boundarypoints, buffer):
        boundaryPath = mpath.Path(boundarypoints)
        self.obstacle_map = [[False for _ in range(self.map_lon_width)] for _ in range(self.map_lat_width)]
        for initial_x in range(self.map_lat_width):
            x = self.transform_to_real_position(initial_x, self.min_lat)
            for initial_y in range(self.map_lon_width):
                y = self.transform_to_real_position(initial_y, self.min_lon)
                if not boundaryPath.contains_point([x,y]):
                    self.obstacle_map[initial_x][initial_y] = True
                else:
                    for obstacle in obstacles:
                        # print(f"x: {x}          y: {y}         hypot: {math.hypot(obstacle[0] - x, obstacle[1] - y)}       radius: {obstacle[2]}     circle center: {obstacle[0]}, {obstacle[1]}")
                        if math.hypot(obstacle[0] - x, obstacle[1] - y) - self.buffer <= obstacle[2]:
                            self.obstacle_map[initial_x][initial_y] = True
                            break
    
    
    
def main():
    start_time = time.time()
    mission_data = "D:/Leonard/Documents/PART/testmap.json"
    resolution = 1
    buffer = 5
    map = Map(resolution, mission_data, buffer)
    print(f"time: {time.time() - start_time}")
    invalid = []
    for x in range(len(map.obstacle_map)):
        for y in range(len(map.obstacle_map[x])):
            if not map.obstacle_map[x][y]:
                invalid.append([map.transform_to_real_position(x, map.min_lat), map.transform_to_real_position(y, map.min_lon)])
    fig, ax = plt.subplots()

    for node in invalid:
        plt.plot(node[0], node[1], '.k')
    ax.set_xlim(map.min_lat - 10, map.max_lat + 10)
    ax.set_ylim(map.min_lon - 10, map.max_lon + 10)
    plt.grid(True)
    plt.axis("equal")
    plt.show()

if __name__ == "__main__":
    main()

