mapping_directory = "../src/path_planning"
interop_path = "../mission_plan/testfield_example.json"
min_obstacles = 5
max_obstacles = 8
min_obstacle_radius = 50
max_obstacle_radius = 100
min_waypoints = 13
max_waypoints = 17
min_waypoint_altitude = 200
max_waypoint_altitude = 400
min_searchpoints = 10
max_searchpoints = 15
USING_INTEROP_BOUNDARY = True
boundary_min_lat = 38.14
boundary_max_lat = 38.15
boundary_min_lon = -76.435
boundary_max_lon = -76.428
min_boundarypoints = 12
max_boundarypoints = 16

resolution = 10
buffer = 30
    

# Inputs
# string mapping_directory - path to Leonard's mapping directory
# string interop_path - path to file of interop example
# int min_obstacles - minimum number of obstacles
# int max_obstacles - maximum nmber of obstacles
# int min_radius - minimum radius of obstacles
# int max_radius - maximum radius of obstacles
# int min_waypoints - minimum number of waypoints
# int max_waypoints - maximum number of waypoints
# int min_waypoint_altitude - minimum altitude of waypoints
# int max_waypoint_altitude - maximum altitude of waypoints
# int min_searchpoints - minimum number of search points
# int max_searchpoitns - maximum number of search points
# bool USING_INTEROP_BOUNDARY - set to true if using original boundary

# Outputs
# dictionary interop_output - Same format as interop_example but with new points

import sys
import json
import math
import matplotlib.pyplot as plt
import random

sys.path.insert(1, mapping_directory)

from Mapping import Map



def calcAngle(x1, y1, x2, y2):
    
    if (x2 - x1) == 0:
        x2 += 0.0001
        
    slope = (y2 - y1) / (x2 - x1)
    
    ang = math.atan(slope)
    
    
    if (x2 < x1):
        ang += math.pi
    elif (math.fabs(x1 - x2) < 0.0000001 and y2 < y1):
        ang += math.pi
    
    
    ang %= 2 * math.pi
    
    return round(ang, 3)


def orderPoints(searchPts):
        
        # Using mean distance in each dimension
        
        midLat = 0
        midLon = 0
        for i in searchPts:
            midLat += i[0]
            midLon += i[1]
            
        midLat /= len(searchPts)
        midLon /= len(searchPts)
        
        # Using center of bounding box
        
        '''
        minLat = 99999
        maxLat = -99999
        minLon = 99999
        maxLon = -99999
        
        for i in self.searchPts:
            if (i["latitude"] < minLat):
                minLat = i["latitude"]
            if (i["latitude"] > maxLat):
                maxLat = i["latitude"]
            if (i["longitude"] < minLon):
                minLon = i["longitude"]
            if (i["longitude"] > maxLon):
                maxLon = i["longitude"]
        
        midLat = (minLat + maxLat) / 2
        midLon = (minLat + maxLat) / 2
        '''

        angles = []
        for i in searchPts:
            angles.append({"point" : i, "angle" : calcAngle(midLon, midLat, i[1], i[0])})
        
        angles = sorted(angles, key=lambda x:x["angle"])
        
        orderedPts = []
        
        for i in angles:
            orderedPts.append(i["point"])
            
        return orderedPts
        
        

# Get interop_example object

interop_file = open(interop_path, "r")
interop_example_obj = json.load(interop_file)
interop_file.close()



if (not USING_INTEROP_BOUNDARY):
    num_new_boundarypoints = random.randrange(min_boundarypoints, max_boundarypoints, 1)
    new_boundarypoints = []
    new_boundarypoints_dict = []
    
    while(len(new_boundarypoints) < num_new_boundarypoints):
        rand_x = random.randrange(1, 127, 1)
        rand_y = random.randrange(1, 127, 1)
        
        new_boundarypoint_position_x = boundary_min_lat + (boundary_max_lat - boundary_min_lat) / 127 * rand_x
        new_boundarypoint_position_y = boundary_min_lon + (boundary_max_lon - boundary_min_lon) / 127 * rand_y
        
        new_boundarypoints.append([new_boundarypoint_position_x, new_boundarypoint_position_y])
        
    new_boundarypoints = orderPoints(new_boundarypoints)
    new_boundarypoints.append(new_boundarypoints[0])

    for pt in new_boundarypoints:
        new_boundarypoints_dict.append({"latitude" : pt[0], "longitude" : pt[1]})
        
    interop_example_obj["flyZones"][0]['boundaryPoints'] = new_boundarypoints_dict


# Get dictionaries needed to be changed
old_obstacles = interop_example_obj["stationaryObstacles"]
old_waypoints = interop_example_obj["waypoints"]
old_searchgrid = interop_example_obj["searchGridPoints"]

# Create map

boundary_points= []
obstacles = []

# Get boundary points for empty map
for boundarypoint in interop_example_obj['flyZones'][0]['boundaryPoints']:
    boundary_points += [[boundarypoint["latitude"],
                        boundarypoint["longitude"]]]
boundary_points.append(list(boundary_points[0]))

# Create empty map
emptyMap = Map(resolution, boundary_points, [], buffer)
minGrid = 0
maxGrid = len(emptyMap.obstacle_map)

print(str(emptyMap.map_x_width))
print(str(emptyMap.map_y_width))
print(str(maxGrid))

# Create new obstacles
num_new_obstacles = random.randrange(min_obstacles, max_obstacles, 1)
new_obstacles = []

while (len(new_obstacles) < num_new_obstacles):
    new_obstacle_position_x = random.randrange(1, emptyMap.map_x_width, 1)
    new_obstacle_position_y = random.randrange(1, emptyMap.map_y_width, 1)
    if not emptyMap.obstacle_map[new_obstacle_position_x][new_obstacle_position_y]:
        new_obstacles.append([new_obstacle_position_x, new_obstacle_position_y])
            

new_obstacle_dict = []
for obstacle in new_obstacles:
    map_to_x = emptyMap.transform_to_cart_position(obstacle[0])
    map_to_y = emptyMap.transform_to_cart_position(obstacle[1])
    
    ob_lat, ob_lon = emptyMap.cartesian_to_decimal(map_to_x, map_to_y, emptyMap.min_lat, emptyMap.min_lon)
    
    new_obstacle_dict.append({"latitude" : ob_lat, "longitude" : ob_lon, "radius" : random.randrange(min_obstacle_radius, max_obstacle_radius, 10), "height" : 750})
    

# Create new search grid


# Create map
boundary_points= []
obstacles = []


for obstacle in new_obstacle_dict:
    obstacles += [[obstacle["latitude"],
                    obstacle["longitude"], obstacle["radius"]]]
    
for boundarypoint in interop_example_obj['flyZones'][0]['boundaryPoints']:
    boundary_points += [[boundarypoint["latitude"],
                        boundarypoint["longitude"]]]
boundary_points.append(list(boundary_points[0]))


newMap = Map(resolution, boundary_points, obstacles, buffer)
minGrid = 0
maxGrid = newMap.map_x_width - 1



# Create new waypoints
num_new_waypoints = random.randrange(min_waypoints, max_waypoints, 1)
new_waypoints = []

while(len(new_waypoints) < num_new_waypoints):
    new_waypoint_position_x = random.randrange(1, newMap.map_x_width, 1)
    new_waypoint_position_y = random.randrange(1, newMap.map_y_width, 1)
    if not newMap.obstacle_map[new_waypoint_position_x][new_waypoint_position_y]:
        new_waypoints.append([new_waypoint_position_x, new_waypoint_position_y])

new_waypoints = orderPoints(new_waypoints)

# Create new search boundary
num_new_searchpoints = random.randrange(min_searchpoints, max_searchpoints, 1)
new_searchpoints = []

while(len(new_searchpoints) < num_new_searchpoints):
    new_searchpoint_position_x = random.randrange(1, newMap.map_x_width, 1)
    new_searchpoint_position_y = random.randrange(1, newMap.map_y_width, 1)
    if not newMap.obstacle_map[new_searchpoint_position_x][new_searchpoint_position_y]:
        new_searchpoints.append([new_searchpoint_position_x, new_searchpoint_position_y])

new_searchpoints = orderPoints(new_searchpoints)
new_searchpoints.append(new_searchpoints[0])


valid = []
for x in range(len(newMap.obstacle_map)):
    for y in range(len(newMap.obstacle_map[x])):
        if newMap.obstacle_map[x][y]:
            valid.append([x, y])

fig, ax = plt.subplots()

for node in valid:
    plt.plot(node[0], node[1], '.k')

wp_xi = []
wp_yi = []
i = 1
for waypoint in new_waypoints:
    waypoint_x = waypoint[0] #* newMap.map_x_width / maxGrid
    waypoint_y = waypoint[1] #* newMap.map_y_width / maxGrid
    wp_xi.append(waypoint_x)
    wp_yi.append(waypoint_y)
    plt.text(waypoint_x, waypoint_y, str(i), fontsize=10)
    i += 1

plt.plot(wp_xi,wp_yi)

sp_xi = []
sp_yi = []
i = 1

for searchpoint in new_searchpoints:
    searchpoint_x = searchpoint[0] #* newMap.map_x_width / maxGrid
    searchpoint_y = searchpoint[1] #* newMap.map_y_width / maxGrid
    sp_xi.append(searchpoint_x)
    sp_yi.append(searchpoint_y)
    plt.text(searchpoint_x, searchpoint_y, str(i), fontsize=10)
    i += 1

plt.plot(sp_xi,sp_yi)

new_waypoint_dict = []
for waypoint in new_waypoints:
    map_to_x = emptyMap.transform_to_cart_position(waypoint[0])
    map_to_y = emptyMap.transform_to_cart_position(waypoint[1])
    
    wp_lat, wp_lon = emptyMap.cartesian_to_decimal(map_to_x, map_to_y, emptyMap.min_lat, emptyMap.min_lon)
    
    new_waypoint_dict.append({"latitude" : wp_lat, "longitude" : wp_lon, "altitude" : random.randrange(min_waypoint_altitude, max_waypoint_altitude)})

new_searchpoint_dict = []
for obstacle in new_searchpoints:
    map_to_x = emptyMap.transform_to_cart_position(searchpoint[0])
    map_to_y = emptyMap.transform_to_cart_position(searchpoint[1])
    
    sp_lat, sp_lon = emptyMap.cartesian_to_decimal(map_to_x, map_to_y, emptyMap.min_lat, emptyMap.min_lon)
    
    new_searchpoint_dict.append({"latitude" : sp_lat, "longitude" : sp_lon})


plt.grid(True)
plt.axis("equal")
plt.show()

# Export to json file
new_interop_example_obj = interop_example_obj
interop_example_obj["stationaryObstacles"] = new_obstacle_dict
interop_example_obj["searchGridPoints"] = new_searchpoint_dict
interop_example_obj["waypoints"] = new_waypoint_dict

interop_output_file = open("new_interop_example.json", "w")
json.dump(new_interop_example_obj, interop_output_file)
interop_output_file.close()