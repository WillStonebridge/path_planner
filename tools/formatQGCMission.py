import json
import sys
import getopt
import mavros_msgs.msg as mavros_msgs

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
            "frame": None,
            "params": None,
            "type": "SimpleItem"
    }
    i = 0
    for waypoint in waypoints:
        if i == 0:
            wp['command'] = mavros_msgs.CommandCode.NAV_TAKEOFF
            wp['doJumpId'] = i + 1 
        elif i == len(waypoints) - 2:
            wp['command'] = mavros_msgs.CommandCode.DO_LAND_START
            wp['doJumpId'] = i + 1
            wp['params'] = [0, 0, 0, 0, 0, 0, 0]
            item.append(wp.copy())
            wp['command'] = mavros_msgs.CommandCode.NAV_WAYPOINT
            wp['doDumpId'] = i + 2
            wp['Altitude'] = waypoint['altitude']
            wp['params'] = [0, 0, 0, 0, waypoint['latitude'], waypoint['longitude'], waypoint['altitude']]
            item.append(wp.copy())
            i += 1
            continue
        elif i == len(waypoints) - 1:
            wp['command'] = mavros_msgs.CommandCode.NAV_LAND
            wp['doJumpId'] = i + 2
        else: 
            wp['command'] = mavros_msgs.CommandCode.NAV_WAYPOINT
            wp['doJumpId'] = i + 1 
        wp['Altitude'] = waypoint['altitude']
        wp['params'] = [0, 0, 0, 0, waypoint['latitude'], waypoint['longitude'], waypoint['altitude']]
        i += 1
        item.append(wp.copy())
    print(item)
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
    routepath_file = "../mission_plan/a_star.json"
    outputFile = "qgc_formatted.plan"
        
    options = "i:"
    long_options = ["file"]
    try:
        # Parsing argument
        arguments, values = getopt.getopt(argv, options, long_options) 
    except getopt.GetoptError:
        print('Options:\n -i <inputfile> ')
        sys.exit(2)
    for opt, arg in arguments:
        if opt == '-i':
            routepath_file = arg 
    with open(routepath_file, 'r') as file:
        routepathObj = json.load(file)

    mission_json = formatPlanFile(routepathObj)
    with open(outputFile, "w") as file:
        json.dump(mission_json, file)

if __name__ == "__main__":
    main(sys.argv[1:])
