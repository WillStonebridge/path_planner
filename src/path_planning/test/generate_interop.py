import json
import sys, getopt
import os

'''
creates a randomly generated set of:
    Waypoints
   Obstacles
    Searchgrid
from:
    Boundary Waypoints in example.json
''' 

def create_mission_plan(file, numObstacles, numWp)

    boundarypoints = []
    for boundarypoint in file['flyZones'][0]['boundaryPoints']:
        boundarypoints += [[boundarypoint["latitude"],
                            boundarypoint["longitude"]]]
    boundarypoints.append(list(boundarypoints[0]))
    

def main(argv):
    mission_data = "../../../mission_plan/interop_example.json"
    out_file = "../../../mission_plan/random_mission.json"
    
    options = "r"
    long_options = ["file"]
    try:
        # Parsing argument
        arguments, values = getopt.getopt(argv, options, long_options) 
    except getopt.GetoptError:
        print('Options:\n -i <inputfile> \n -r <resolution meters> -b <buffer meters>')
        sys.exit(2)
    for opt, arg in arguments:
        if opt == '-i':
            mission_data = arg
    file = json.load(open(mission_data, 'rb'))
    waypoints = []
    boundarypoints = []
    obstacles = []
    searchGridPoints = []
    for waypoint in file["waypoints"]:
            waypoints += [[waypoint["latitude"],
                                waypoint["longitude"], waypoint["altitude"]]]

    for boundarypoint in file['flyZones'][0]['boundaryPoints']:
        boundarypoints += [[boundarypoint["latitude"],
                            boundarypoint["longitude"]]]
    boundarypoints.append(list(boundarypoints[0]))

    for obstacle in file["stationaryObstacles"]:
        obstacles += [[obstacle["latitude"],
                        obstacle["longitude"], obstacle["radius"]]]

    for search in file["searchGridPoints"]:
        searchGridPoints += [[search["latitude"],
                        search["longitude"]]]
 
    # Creating Map for transformation tool
    map = Map(resolution, boundarypoints, obstacles, buffer)
    out_json = create_mission_plan(map, 6, 20) 
    




if __name__ == "__main__":
    main(sys.argv[1:])
