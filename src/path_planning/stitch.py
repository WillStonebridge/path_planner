import json
import sys, os
sys.path.append(
    os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir)))
sys.path.append(
    os.path.dirname(os.path.abspath(__file__)) + "/Dubins")

from Mapping import Map
from rrt_dubins import RRTDubins
from searching import searching
import numpy as np
import Landing

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

    map = Map(10, boundarypoints, obstacles, 0)

    # 1. Run Abhi Coverage Path Plannign on interop example file
    searching_wp = searching.runProgram(constAlt, cameraWidth, inputFile) # List
    print("Searching Generated")

    # 2. Run Paul Landing
    landing_wp = Landing.calc_landing(map, obstacles, waypoints[len(waypoints)-1], [waypoints[0], waypoints[1]], 8)
    print("Landing Generated")
    # Run rrt_dubins

    # 3. Combine Lists
    combined_wp = []
    combined_wp.extend(interopObj["waypoints"])
    combined_wp.extend(searching_wp)
    combined_wp.extend(landing_wp)

    #outputList = []
    #outputList.extend(searching_wp)
    #outputList.extend(landing_wp)

    # In-route Waypoints


    print("In Route RRT Dubins Found")

    # Searching Waypoints
    print("Searching RRT Dubins Found")


    # Write dicitonary to JSON
    
    #routepathObj["waypoints"].extend(outputList)

    #with open(outputFile, "w") as file:
    #    json.dump(routepathObj, file)
    
if __name__ == "__main__":
    main(sys.argv[1:])
