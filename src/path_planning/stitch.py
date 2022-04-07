import json
import sys
import getopt
import os
import numpy as np
import matplotlib.pyplot as plt
#import mavros_msgs.msg as mavros_msgs

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


def formatPolygon(polygon: dict) -> dict:
    '''
    Accepts boundaryPoints from a flyZone object, returns a formatted polygon object
    '''
    formattedPolygon = {
                        
                         "inclusion": True,
                         "polygon": [],
                         "version":1
                       }
    for polygonItem in polygon:
        formattedPolygon["polygon"].append([polygonItem["latitude"], polygonItem["longitude"]])
    return formattedPolygon

def formatCircle(stationaryObstacle: dict) -> dict:
    '''
    Accepts a circular obstacle, returns a formatted circle object
    '''
    ft2m = 0.3048
    formattedCircle = {
                        "circle" : {
                            "center": [],
                            "radius": 0
                         },
                         "inclusion": False,
                         "version":1
                       }
    formattedCircle["circle"]["center"] = [stationaryObstacle["latitude"], stationaryObstacle["longitude"]]
    formattedCircle["circle"]["radius"] = stationaryObstacle['radius'] * ft2m
    return formattedCircle

def formatMission(waypoints: dict, AMSLAltAboveTerrain=None, AltitudeMode=1) -> dict:
    '''
    Take waypoints, formats into QGC mission format
    '''
    item = []
    wp = {
            "AMSLAltAboveTerrain": AMSLAltAboveTerrain,
            "Altitude": 0,
            "AltitudeMode": 1,
            "autoContinue": True,
            "command": None,
            "doJumpId": 0,
            "frame": 3, # mavros_msgs.Waypoint.NAV_FRAME_GLOBAL_REALTIVE_ALT
            "params": None,
            "type": "SimpleItem"
    }
    i = 0
    for waypoint in waypoints:
        if i == 0:
            wp['command'] = 22 #mavros_msgs.CommandCode.NAV_TAKEOFF
            wp['doJumpId'] = i + 1 
        elif i == len(waypoints) - 2:
            wp['command'] = 189 #mavros_msgs.CommandCode.DO_LAND_START
            wp['doJumpId'] = i + 1
            wp['params'] = [0, 0, 0, 0, 0, 0, 0]
            item.append(wp.copy())
            wp['command'] = 16 #mavros_msgs.CommandCode.NAV_WAYPOINT
            wp['doDumpId'] = i + 2
            wp['Altitude'] = waypoint['altitude']
            wp['params'] = [0, 0, 0, 0, waypoint['latitude'], waypoint['longitude'], waypoint['altitude']]
            wp['frame'] = 2 #mavros_msgs.Waypoint.NAV_FRAME_GLOBAL
            item.append(wp.copy())
            i += 1
            continue
        elif i == len(waypoints) - 1:
            wp['command'] = 21 #mavros_msgs.CommandCode.NAV_LAND
            wp['doJumpId'] = i + 2
        else: 
            wp['command'] = 16 #mavros_msgs.CommandCode.NAV_WAYPOINT
            wp['doJumpId'] = i + 1 
        wp['Altitude'] = waypoint['altitude']
        wp['params'] = [0, 0, 0, 0, waypoint['latitude'], waypoint['longitude'], waypoint['altitude']]
        i += 1
        item.append(wp.copy())
    return item

def formatPlanFile(routepath: dict) -> dict:
    planfile = {
                 "fileType": "Plan",
                 "geoFence": {
                     "circles": [],
                      "polygons":[],
                     "version": 2
                     },
                 "groundStation": "QGroundControl",
                 "mission": {
                     "cruiseSpeed": 15, 
                     "firmwareType": 12, 
                     "globalPlanAltitudeMode": 1,
                     "hoverSpeed": 5,
                     "items": [],
                     "plannedHomePosition": [],
                     "vehicleType": 1,
                     "version":2
                     },
                 "rallyPoints": {
                     "points": [],
                     "version": 2
                     },
                 "version": 1
               }
    for polygon in routepath["flyZones"]:
        planfile["geoFence"]["polygons"].append(formatPolygon(polygon["boundaryPoints"]))
    for stationaryObstacle in routepath["stationaryObstacles"]:
        planfile["geoFence"]["circles"].append(formatCircle(stationaryObstacle))
    planfile["mission"]["items"] = formatMission(routepath['waypoints'])
    planfile["mission"]["plannedHomePosition"] = [
                                                  routepath['waypoints'][0]['latitude'], 
                                                  routepath['waypoints'][0]['longitude'], 
                                                  0
                                                  ]
    return planfile
def main(argv):

    # constants
    constAlt = 100
    cameraWidth = 130
    inputFile = "../../mission_plan/example/interop_example.json"
    combinedFile = "../../mission_plan/combinedpath.json"
    outputFile = "../../mission_plan/mavros_route.json"
    qgcFile = "../../mission_plan/qgc_route.plan"
    resolution = 10
    buffer = 10

    options = "r:b:i:o:"
    long_options = ["file"]
    try:
        # Parsing argument
        arguments, values = getopt.getopt(argv, options, long_options) 
    except getopt.GetoptError:
        print('Options:\n -i <inputfile> \n -r <resolution meters> -b <buffer meters>')
        sys.exit(2)
    for opt, arg in arguments:
        if opt == '-i':
            inputFile = arg
        elif opt == '-r':
            resolution = int(arg)
        elif opt == '-b':
            buffer = int(arg)
        elif opt == '-o':
            outputFile = int(arg)

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
    #with open(combinedFile, "w") as file:
    #    json.dump(combinedWpObj, file)


    # 3. Run Dubins
    dubins_wp = []

    # In-route Waypoints

    dubins_planner = RRTDubins(map, max_iter=3000) 
    
    for i in range(len(interopObj["waypoints"]) - 1):
        dubins_wp.extend(dubins_planner.planning(interopObj["waypoints"][i], interopObj["waypoints"][i+1], animation=show_animation, min_radius = 1, max_radius=50))
    print("In Route RRT Dubins Found")

    ## Searching Waypoints
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
    mission_json = formatPlanFile(routepathObj)

    with open(outputFile, "w") as file:
        json.dump(routepathObj, file)

    with open(qgcFile, "w") as file:
        json.dump(mission_json, file)
if __name__ == "__main__":
    main(sys.argv[1:])
