import json
import sys, os
sys.path.append(
    os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir)))

from Mapping import Map
from Dubins import *
from searching import searching



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

    map = Map(resolution, boundarypoints, obstacles, buffer)

    # 1. Run Abhi Coverage Path Plannign on interop example file
    searching_wp = searching.runProgram(constAlt, cameraWidth, inputFile) # List
    #searchpoints_file = open("searchpath.json", "r")
    #searchpointsObj = json.load(searchpoints_file)
    #searchpoints_file.close()
    #wayPts = searchpointsObj
    
    # 2. Run Paul Landing
    landing_wp = [] # List
    
    # Combine Lists
    outputList = []
    outputList.extend(searching_wp)
    outputList.extend(landing_wp)

    # Run rrt_dubins

    # Write dicitonary to JSON
    
    routepathObj["waypoints"].extend(outputList)

    with open(outputFile, "w") as file:
        json.dump(routepathObj, file)
    
if __name__ == "__main__":
    main(sys.argv[1:])
