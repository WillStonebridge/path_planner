import json
import matplotlib.pyplot as plt
import numpy as np
import math
from Mapping import Map
import time
class PathPlanner:
    def __init__(self, resolution, mission_data, buffer):
        start_time = time.time()
        file = json.load(open(mission_data, 'rb'))
        self.waypoints = []
        boundarypoints = []
        obstacles = []
        for waypoint in file["waypoints"]:
                self.waypoints += [[waypoint["latitude"],
                                    waypoint["longitude"], waypoint["altitude"]]]

        for boundarypoint in file["boundaryPoints"]:
            boundarypoints += [[boundarypoint["latitude"],
                                boundarypoint["longitude"]]]
        boundarypoints.append(boundarypoints[0])

        for obstacle in file["obstacles"]:
            obstacles += [[obstacle["latitude"],
                            obstacle["longitude"], obstacle["radius"]]]
        self.map = Map(resolution, boundarypoints, obstacles, buffer)
        print(f"mapping time: {time.time() - start_time}")
        self.motion = [[1, 0, 1],                  #[x direction, y direction, cost]
                        [0, 1, 1],
                        [-1, 0, 1],
                        [0, -1, 1],
                        [-1, -1, math.sqrt(2)],
                        [-1, 1, math.sqrt(2)],
                        [1, -1, math.sqrt(2)],
                        [1, 1, math.sqrt(2)]]

        
    class Node:
        def __init__(self, lat, lon, cost, parent_index):
            self.lat = lat
            self.lon = lon
            self.cost = cost
            self.parent_index = parent_index

    def calc_heurestic(self, gnode, snode):
        distance1 = math.hypot(gnode.lat - snode.lat, gnode.lon - snode.lon)
        return distance1

    def calc_grid_index(self, node):
        return (node.lon - self.map.min_lon) * self.map.map_lat_width + (node.lat - self.map.min_lat)

    def a_star(self, snode, gnode):
        start_node = snode
        goal_node = gnode
        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(snode)] = snode
        rough_path_lat = []
        rough_path_lon = []
        while 1:
            if len(open_set) == 0:
                print("something went wrong")
                break
            current_id = min(open_set, key = lambda o: open_set[o].cost + self.calc_heurestic(gnode, open_set[o])) # finds the node with lowest cost
            current = open_set[current_id]
            # if True:  # pragma: no cover
            #     plt.plot(self.map.transform_to_map_position(current.lat, self.map.min_lat),
            #              self.map.transform_to_map_position(current.lon, self.map.min_lon), "xc")
            #     # for stopping simulation with the esc key.
            #     plt.gcf().canvas.mpl_connect('key_release_event',
            #                                  lambda event: [exit(
            #                                      0) if event.key == 'escape' else None])
            #     if len(closed_set.keys()) % 10 == 0:
            #         plt.pause(0.001)
            if current.lat == goal_node.lat  and current.lon == goal_node.lon:
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break
                
            del open_set[current_id]
            closed_set[current_id] = current
            for i in range(len(self.motion)):
                node = self.Node(current.lat + self.motion[i][0],
                                 current.lon + self.motion[i][1],
                                 current.cost + self.motion[i][2], current_id)
                new_id = self.calc_grid_index(node)

                if not self.verify_node(node):
                    continue
                if new_id in closed_set:
                    continue
                if new_id not in open_set:
                    open_set[new_id] = node
                else:
                    if open_set[new_id].cost > node.cost:
                        open_set[new_id] = node
        rough_path_lat.append(self.map.transform_to_map_position(gnode.lat, self.map.min_lat))
        rough_path_lon.append(self.map.transform_to_map_position(gnode.lon, self.map.min_lon))
        parent_index = gnode.parent_index
        while parent_index != -1:
            node = closed_set[parent_index]
            rough_path_lat.append(self.map.transform_to_map_position(node.lat, self.map.min_lat))
            rough_path_lon.append(self.map.transform_to_map_position(node.lon, self.map.min_lon))
            parent_index = node.parent_index
        return rough_path_lat, rough_path_lon
    
    def calc_path(self, swaypoint, gwaypoint):
        finalpathx = []
        finalpathy = []
        start_time = time.time()
        start_node = self.Node(self.map.transform_to_map_position(swaypoint[0], self.map.min_lat), self.map.transform_to_map_position(swaypoint[1], self.map.min_lon), 0.0, -1)
        goal_node = self.Node(self.map.transform_to_map_position(gwaypoint[0], self.map.min_lat), self.map.transform_to_map_position(gwaypoint[1], self.map.min_lon), 0.0, -1)
        x, y = self.a_star(start_node, goal_node)
        ex, ey = self.calc_critical_nodes(x,y)
        fx, fy = self.calc_smooth_line(ex, ey)
        finalpathx += (fx) 
        finalpathy += (fy)
        print(f"time for leg: {time.time() - start_time}")
        plt.plot(x,y, '-r')
        plt.plot(ex,ey, '.b')
        plt.plot(fx, fy, 'g-s')
        return finalpathx, finalpathy
    # def calc_path(self):
    #     finalpathx = []
    #     finalpathy = []
    #     start_node = self.Node(self.map.transform_to_map_position(self.waypoints[0][0], self.map.min_lat), self.map.transform_to_map_position(self.waypoints[0][1], self.map.min_lon), 0.0, -1)
    #     goal_node = self.Node(self.map.transform_to_map_position(self.waypoints[1][0], self.map.min_lat), self.map.transform_to_map_position(self.waypoints[1][1], self.map.min_lon), 0.0, -1)
    #     x, y = self.a_star(start_node, goal_node)
    #     ex, ey = self.calc_critical_nodes(x,y)
    #     fx, fy = self.calc_smooth_line(ex, ey)
    #     finalpathx += (fx) 
    #     finalpathy += (fy)

    #     # plt.plot(x,y, '-r')
    #     # plt.plot(ex,ey, '.b')
    #     # plt.plot(fx, fy, 'g-s')
    #     return finalpathx, finalpathy        

    def verify_node(self, node):
        x = round(self.map.transform_to_map_position(node.lat, self.map.min_lat))
        y = round(self.map.transform_to_map_position(node.lon, self.map.min_lon))
        if self.map.obstacle_map[x][y]:
            return False
        return True

    def calc_critical_nodes(self, rx, ry):
        criticalx = []
        criticaly = []
        x = np.array(rx)
        y = np.array(ry)
        m = np.diff(y)/np.diff(x)    
        for i in range(m.size):
            if(i  == 0):
                criticalx.append(x[i])
                criticaly.append(y[i])
                continue
            if(m[i] != m[i-1]):
                criticalx.append(x[i])
                criticaly.append(y[i])
        criticalx.append(x[x.size - 1])
        criticaly.append(y[y.size - 1])                
        return criticalx, criticaly

    def verify_line_obstacle(self, x1, y1, x2, y2):
        mx = float((x2 - x1) / 100)
        my = float((y2 - y1) / 100)

        for t in range(0,100):
            # print(round(float((x2 - x1) / 100) * t))
            # print(round(float((y2 - y1) / 100) * t))
            x = int(self.map.transform_to_map_position(int(round(mx * t)) + x1, self.map.min_lat))
            y = int(self.map.transform_to_map_position(int(round(my * t)) + y1, self.map.min_lon))

            if self.map.obstacle_map[x][y] == True:
                return True
        return False
    def calc_smooth_line(self, criticalx, criticaly):
        finalx = []
        finaly = []
        current = 0
        while current < len(criticalx):
            # print(current)
            finalx.append(criticalx[current])
            finaly.append(criticaly[current])
            for check in range(current + 1, len(criticalx)):
                if(self.verify_line_obstacle(criticalx[current], criticaly[current], criticalx[check], criticaly[check])):
                    current = check - 1 
                    break
                elif(check == len(criticalx) - 1):
                    current = len(criticalx)
        finalx.append(criticalx[len(criticalx) - 1])
        finaly.append(criticaly[len(criticaly) - 1])
        return finalx, finaly

def main():
    mission_data = "D:/Leonard/Documents/PART/PathFinding/testmap.json"
    resolution = 1
    buffer = 3
    plan = PathPlanner(resolution, mission_data, buffer)
    for i in range(len(plan.waypoints) - 1):
        fx, fy = plan.calc_path(plan.waypoints[i], plan.waypoints[i+1])
    invalid = []
    for x in range(len(plan.map.obstacle_map)):
        for y in range(len(plan.map.obstacle_map[x])):
            if plan.map.obstacle_map[x][y]:
                invalid.append([plan.map.transform_to_real_position(x, plan.map.min_lat), plan.map.transform_to_real_position(y, plan.map.min_lon)])
    

    for node in invalid:
        plt.plot(node[0], node[1], '.k')

 
    plt.grid(True)
    plt.axis("equal")
    plt.show()

if __name__ == "__main__":
    main()

            

    