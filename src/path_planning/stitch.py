import json
import sys
import os
import numpy as np
import matplotlib.pyplot as plt

sys.path.append(
    os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir)))
sys.path.append(
    os.path.dirname(os.path.abspath(__file__)) + "/Dubins")

from Mapping import Map
from rrt_dubins import RRTDubins
from searching import searching
import Landing

# Get interop file and object

# SET CONSTANTS
show_animation = False

def main(argv):

    # constants
    constAlt = 100
    cameraWidth = 130
    inputFile = "../../mission_plan/interop_example.json"
    combinedFile = "../../mission_plan/combinedpath.json"
    outputFile = "../../mission_plan/routepath.json"
    max_angle = 15
    resolution = 10
    buffer = 10
    #runway_start = 
    #runway_end

    # interop points    
    with open(inputFile, "r")as file:
        interopObj = json.load(file)
    routepathObj = dict(interopObj)
    combinedWpObj = dict(routepathObj)

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

    map = Map(resolution, boundarypoints, obstacles, buffer)
    if show_animation:
        x_list_bound = []
        y_list_bound = []
        for boundary in boundarypoints:
            x_list_bound.append(boundary[0])
            y_list_bound.append(boundary[1])  
        x_list_obs = [] 
        y_list_obs = []
        radiuses = []
        for obstacle in obstacles:
            x_list_obs.append(obstacle[0])
            y_list_obs.append(obstacle[1])
            radiuses.append(obstacle[2]) 
        obstacleList = []
        for i in range(len(x_list_obs)):
            obstacleList.append((x_list_obs[i],y_list_obs[i],radiuses[i]/10)) 
   

    # 1. Run Abhi Coverage Path Plannign on interop example file
    searching_wp = searching.runProgram(constAlt, cameraWidth, inputFile, animation=False) # List
    print("Searching Generated")

    # 2. Run Paul Landing
    landing_wp = Landing.calc_landing(map, obstacles, waypoints[len(waypoints)-1], [waypoints[0], waypoints[1]], 8)
    print("Landing Generated")
    # Combining intermediary Waypoints for logging (#TODO ADD Logging)

    combinedWp =  []
    combinedWp.extend(interopObj["waypoints"])
    combinedWp.extend(searching_wp)
    combinedWp.extend(landing_wp)
    combinedWpObj["waypoints"] = combinedWp
    with open(combinedFile, "w") as file:
        json.dump(combinedWpObj, file)


    # 3. Run Dubins
    dubins_wp = []

    # In-route Waypoints

    dubins_planner = RRTDubins(map, max_iter=3000) 
    
    for i in range(len(interopObj["waypoints"])-1):
        dubins_wp.extend(dubins_planner.planning(interopObj["waypoints"][i], interopObj["waypoints"][i+1], animation=show_animation))
    print("In Route RRT Dubins Found")

    # Searching Waypoints
    for i in range(len(searching_wp)-1):
        dubins_wp.extend(dubins_planner.planning(searching_wp[i], searching_wp[i+1], animation=show_animation, min_radius = 1, max_radius = 10))

    print("Searching RRT Dubins Found")
    for i in range(len(landing_wp)-1):
        dubins_wp.extend(dubins_planner.planning(landing_wp[i], landing_wp[i+1], animation=show_animation))

    print("Landing RRT Dubins Found")
    if show_animation:
        x_list_bound = []
        y_list_bound = []
        for boundary in boundarypoints:
            x_list_bound.append(boundary[0])
            y_list_bound.append(boundary[1])  
        x_list_obs = [] 
        y_list_obs = []
        radiuses = []
        for obstacle in obstacles:
            x_list_obs.append(obstacle[0])
            y_list_obs.append(obstacle[1])
            radiuses.append(obstacle[2]) 
        obstacleList = []
        for i in range(len(x_list_obs)):
            obstacleList.append((x_list_obs[i],y_list_obs[i],radiuses[i]/10)) 
        dubins_planner.draw_graph(obstacleList)
        plt.plot([x for x in x_list_bound], [y for y in y_list_bound], 'b')
        plt.grid(True)
        plt.show()
    

    # Write dicitonary to JSON
    routepathObj["waypoints"] = 0
    routepathObj["waypoints"] = dubins_wp
    with open(outputFile, "w") as file:
        json.dump(routepathObj, file)
    
if __name__ == "__main__":
    main(sys.argv[1:])
