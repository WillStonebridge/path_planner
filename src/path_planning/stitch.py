import json
import sys, os
sys.path.append(
    os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir)))
sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/Dubins")

from Mapping import Map
from rrt_dubins import RRTDubins
from searching import searching
import numpy as np


# Get interop file and object

# SET CONSTANTS
def main(argv):

    # constants
    constAlt = 100
    cameraWidth = 130
    inputFile = "../../mission_plan/interop_example.json"
    outputFile = "../../mission_plan/routepath.json"
    max_angle = 15
    resolution = 10
    buffer = 5

    # interop points    
    interop_file = open(inputFile, "r")
    #runway_start = 
    #runway_end
    
    interopObj = json.load(interop_file)
    routepathObj = interopObj
    interop_file.close()

    waypoints = []
    boundarypoints = []
    obstacles = []

    for waypoint in interopObj["waypoints"]:
            waypoints += [[waypoint["latitude"],
                                waypoint["longitude"], waypoint["altitude"]]]

    for boundarypoint in interopObj['flyZones'][0]['boundaryPoints']:
        boundarypoints += [[boundarypoint["latitude"],
                            boundarypoint["longitude"]]]
    boundarypoints.append(list(boundarypoints[0]))

    for obstacle in interopObj["stationaryObstacles"]:
        obstacles += [[obstacle["latitude"],
                           obstacle["longitude"], obstacle["radius"]]]

    min = np.amin(boundarypoints, axis = 0)
    map = Map(10, boundarypoints, obstacles, 0)
    x_list_bound = []
    y_list_bound = []
    for boundary in boundarypoints:
        x_list_bound.append(boundary[0])
        y_list_bound.append(boundary[1])
    x_list_obs = []
    y_list_obs = []
    radiuses = []

    x_list_way = []
    y_list_way = []
    wa_list_way = []

    search_area = round(max(max(x_list_bound),max(y_list_bound))) + 50 # FIXME All of this could be included in the init
    blank = '[' #FIXME Why use strings when Dict exists?

    for waypoint in waypoints:
        x,y = map.decimal_to_cartesian(waypoint[0],waypoint[1],min[0],min[1])
        x_list_way.append(x)
        y_list_way.append(y)
        wa_list_way.append(waypoint[2])

    for obstacle in obstacles:
        x_list_obs.append(obstacle[0])
        y_list_obs.append(obstacle[1])
        radiuses.append(obstacle[2])
    obstacleList = []  # [x,y,size(radius)]
    for i in range(len(x_list_obs)):
        obstacleList.append((x_list_obs[i],y_list_obs[i],radiuses[i]/10)) # FIXME Redundant code; Mapping already does this; also, what is this 10?
    search_area = round(max(max(x_list_bound),max(y_list_bound))) + 50 # FIXME All of this could be included in the init; also, what is this 50?

    # 1. Run Abhi Coverage Path Plannign on interop example file

    searching_wp = searching.runProgram(constAlt, cameraWidth, inputFile) # List
    searching_list = []
    sx_list_way = []
    sy_list_way = [] 
    sa_list_way = []
    for waypoint in searching_wp:
            searching_list += [[waypoint["latitude"],
                                waypoint["longitude"], waypoint["altitude"]]]
    for waypoint in searching_list:
        x,y = map.decimal_to_cartesian(waypoint[0],waypoint[1],min[0],min[1])
        sx_list_way.append(x)
        sy_list_way.append(y)
        sa_list_way.append(waypoint[2])
    print("Searching Generated")

    # 2. Run Paul Landing
    landing_wp = [] # List
    print("Landing Generated")
    
    # Run rrt_dubins

# In-route Waypoints
    for i in range(len(x_list_way)-1):
        if i == 0:
            start = [x_list_way[i], y_list_way[i], np.deg2rad(0.0)]
            goal = [x_list_way[i+1], y_list_way[i+1], np.arctan2(y_list_way[i+1]-y_list_way[i],x_list_way[i+1]-x_list_way[i])]
        else:
            start = [x_list_way[i], y_list_way[i], np.arctan2(y_list_way[i]-y_list_way[i-1],x_list_way[i]-x_list_way[i-1])]
            goal = [x_list_way[i+1], y_list_way[i+1], np.arctan2(y_list_way[i+1]-y_list_way[i],x_list_way[i+1]-x_list_way[i])]

        rrt_dubins = RRTDubins(start, goal, obstacleList, [0, search_area],map, max_iter=3000) # FIXME RRTDUubins object should only be creaed once and ran once
        #make area the max of the max y/x
        path = rrt_dubins.planning()

        index_list = [] # FIXME Variable not used
        j = 0
        for point in path: # FIXME again, this could be simply included in rrt_dubins
            index_x = map.transform_to_map_index(point[0])# FIXME Variable not used
            index_y = map.transform_to_map_index(point[1])# FIXME Variable not used
            index_list.append([index_x,index_y]) # FIXME Variable not used
            lat, long = map.cartesian_to_decimal(point[0],point[1],min[0],min[1])
            new_point = {'latitude': str(lat), 'longtitude':str(long), 'altitude':str(wa_list_way[j])}
            blank += json.JSONEncoder().encode(new_point)
            blank += ','
    print("In Route RRT Dubins Found")

# Searching Waypoints
    for i in range(len(sx_list_way)-1):
        if i == 0:
            start = [sx_list_way[i], sy_list_way[i], np.deg2rad(0.0)]
            goal = [sx_list_way[i+1], sy_list_way[i+1], np.arctan2(sy_list_way[i+1]-sy_list_way[i],sx_list_way[i+1]-sx_list_way[i])]
        else:
            start = [sx_list_way[i], sy_list_way[i], np.arctan2(sy_list_way[i]-sy_list_way[i-1],sx_list_way[i]-sx_list_way[i-1])]
            goal = [sx_list_way[i+1], sy_list_way[i+1], np.arctan2(sy_list_way[i+1]-sy_list_way[i],sx_list_way[i+1]-sx_list_way[i])]

        rrt_dubins = RRTDubins(start, goal, obstacleList, [0, search_area],map, max_iter=3000, max_radius = 5, min_radius = 1) # FIXME RRTDUubins object should only be creaed once and ran once
        #make area the max of the max y/x
        path = rrt_dubins.planning()

        index_list = [] #FIXME Variable not used
        j = 0
        for point in path: # FIXME again, this could be simply included in rrt_dubins
            index_x = map.transform_to_map_index(point[0]) #FIXME Variable not used
            index_y = map.transform_to_map_index(point[1]) #FIXME Variable not used
            index_list.append([index_x,index_y]) #FIXME Variable not used
            lat, long = map.cartesian_to_decimal(point[0],point[1],min[0],min[1])
            new_point = {'latitude': str(lat), 'longtitude':str(long), 'altitude':str(sa_list_way[j])}
            blank += json.JSONEncoder().encode(new_point)
            blank += ','

    print("Searching RRT Dubins Found")

    # Combine Lists
    #outputList = []
    #outputList.extend(searching_wp)
    #outputList.extend(landing_wp)

    # Write dicitonary to JSON
    
    routepathObj["waypoints"].extend(outputList)

    with open(outputFile, "w") as file:
        json.dump(routepathObj, file)
    
if __name__ == "__main__":
    main(sys.argv[1:])
