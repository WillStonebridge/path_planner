import copy
from importlib.resources import path
import math
from operator import index
import os
from pickle import TRUE
import random
import sys, getopt
import json
import time
from tkinter import N
import matplotlib.pyplot as plt
import numpy as np

sys.path.append(
    os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir)))

from Mapping import Map

try:
    from rrt import RRT
    import dubins_path_planning
except ImportError:
    raise

show_animation = True


class RRTDubins(RRT):
    """
    Class for RRT planning with Dubins path
    """

    class Node(RRT.Node):
        """
        RRT Node
        """

        def __init__(self, x, y, yaw):
            super().__init__(x, y)
            self.cost = 0
            self.yaw = yaw
            self.path_yaw = []

    def __init__(self, waypoint_dict, map,
                 goal_sample_rate=90,
                 max_iter=1000, altitude = 200
                 ):

        self.goal_sample_rate = goal_sample_rate
        self.altitude = altitude
        self.obstacle_map = map.obstacle_map
        self.map = map
        x,y = map.decimal_to_cartesian(map.max_lat,map.max_lon,map.min_lat,map.min_lon)
        self.waypoint_dict = waypoint_dict
        self.start = self.Node(0,0,0)
        self.goal = self.Node(0,0,0)
        self.radius = 0
        self.radii = []
        self.curvature = 0
        self.min_rand = 0
        self.max_rand = round(max(x,y))
        self.goal_yaw_th = np.deg2rad(1)
        self.goal_xy_th = 0.5
        self.max_iter = max_iter
        self.angle = 0


    def planning(self, start, goal, max_radius = 70, min_radius = 50, animation=True, search_until_max_iter=True):
        """
        execute planning

        animation: flag for animation on or off
        """
        x_start,y_start = self.map.decimal_to_cartesian(start['latitude'], start['longitude'],self.map.min_lat,self.map.min_lon)
        x_end,y_end = self.map.decimal_to_cartesian(goal['latitude'], goal['longitude'],self.map.min_lat,self.map.min_lon)
        
        start_angle = self.angle

        end_angle = np.arctan2(y_end-y_start,x_end-x_start)
        self.angle = end_angle

        self.start = self.Node(x_start, y_start, start_angle)
        self.end = self.Node(x_end, y_end, end_angle)

        iteration = round(self.max_iter / (round(((max_radius - min_radius)/5 + 1))))

        last_index = False
        self.node_list = [self.start]
        self.radius = max_radius
        self.curvature = 1/self.radius
        new_node = None
        final_path = None
        path_dict_list = []
        
        for i in range(self.max_iter):
            if i != 0 and i % iteration == 0:
                
                self.radius = max(self.radius-5,min_radius)
                self.curvature = 1/self.radius
            
            rnd = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd)
            temp_node = self.steer(self.node_list[nearest_ind], rnd)
            
            if temp_node:
                index_x = self.map.transform_to_map_index(temp_node.x)
                index_y = self.map.transform_to_map_index(temp_node.y)
  
                if self.check_path(temp_node) and self.check_boundary([round(index_x),round(index_y)]):
                    self.node_list.append(temp_node)
                    self.radii.append(self.radius)
                    new_node = temp_node

                    if new_node:
                        last_index = self.search_best_goal_node()
                        
                        if last_index: 
                            final_path = self.generate_final_course(last_index)
                            break
                            
            
        if new_node:
            last_index = self.search_best_goal_node()
            
            if last_index: 
                final_path = self.generate_final_course(last_index)
        
        for point in final_path: 
            lat, long = self.map.cartesian_to_decimal(point[0],point[1],self.map.min_lat,self.map.min_lon)
            new_point = {'latitude': str(lat), 'longtitude':str(long), 'altitude':str(self.altitude)}
            path_dict_list.append(new_point)

            if show_animation:
                plt.plot([x for (x, y,_) in final_path], [y for (x, y,_) in final_path], '-r')
            
        return path_dict_list
      
    def draw_graph(self, obs, rnd=None):  # pragma: no cover
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")

        for (ox, oy, size) in obs:
            plt.plot(ox, oy, "ok", ms= size)

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        
        plt.axis([0, 2000, 0, 2000])
        plt.grid(True)
        self.plot_start_goal_arrow()

    def plot_start_goal_arrow(self):  # pragma: no cover
        dubins_path_planning.plot_arrow(
            self.start.x, self.start.y, self.start.yaw)
        dubins_path_planning.plot_arrow(
            self.end.x, self.end.y, self.end.yaw)
    
    def check_path(self, new_node):
       if new_node == None:
            return False
       
       for i in range(len(new_node.path_x)):
            
            index_x = self.map.transform_to_map_index(new_node.path_x[i])
            index_y = self.map.transform_to_map_index(new_node.path_y[i])
            x = (self.check_boundary([round(index_x),round(index_y)]))

            if x == False:
                return False
       return True

    def steer(self, from_node, to_node):
        px, py, pyaw, mode, course_lengths = \
            dubins_path_planning.dubins_path_planning(
                from_node.x, from_node.y, from_node.yaw,
                to_node.x, to_node.y, to_node.yaw, self.curvature)
    
        if len(px) <= 1:  # cannot find a dubins path
            return None

        new_node = copy.deepcopy(from_node)
        new_node.x = px[-1]
        new_node.y = py[-1]
        new_node.yaw = pyaw[-1]

        new_node.path_x = px
        new_node.path_y = py
        new_node.path_yaw = pyaw
        new_node.cost += sum([abs(c) for c in course_lengths])
        new_node.parent = from_node

        return new_node
    
    def check_boundary(self, point):
        try:
            if self.obstacle_map[point[0]][point[1]]:
                return False
        except:
            return False
        
        return True


    def calc_new_cost(self, from_node, to_node):
        _, _, _, _, course_length = dubins_path_planning.dubins_path_planning(
            from_node.x, from_node.y, from_node.yaw,
            to_node.x, to_node.y, to_node.yaw, self.curvature)
        return from_node.cost + course_length

    def get_random_node(self):

        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = self.Node(random.uniform(self.min_rand, self.max_rand),
                            random.uniform(self.min_rand, self.max_rand),
                            random.uniform(-math.pi, math.pi)
                            )
            
        else:  # goal point sampling
            rnd = self.Node(self.end.x, self.end.y, self.end.yaw)

        return rnd

    def search_best_goal_node(self):

        goal_indexes = []
        for (i, node) in enumerate(self.node_list):
            if self.calc_dist_to_goal(node.x, node.y) <= self.goal_xy_th:
                goal_indexes.append(i)

        # angle check
        final_goal_indexes = []
        for i in goal_indexes:
            if abs(self.node_list[i].yaw - self.end.yaw) <= self.goal_yaw_th:
                final_goal_indexes.append(i)

        if not final_goal_indexes:
            return None

        min_cost = min([self.node_list[i].cost for i in final_goal_indexes])
        
        for i in final_goal_indexes:
            if self.node_list[i].cost == min_cost:
                return i

        return None

    def generate_final_course(self, goal_index):
        path = [[self.end.x, self.end.y,self.end.yaw]]
        node = self.node_list[goal_index]
        while node.parent:
            for (ix, iy,iyaw) in zip(reversed(node.path_x), reversed(node.path_y),reversed(node.path_yaw)):
                path.append([ix, iy,iyaw*180/math.pi])
            node = node.parent
        path.append([self.start.x, self.start.y, self.start.yaw*180/math.pi])
        return path


def main(argv):

    altitude = 200
    mission_data = "../../../mission_plan/interop_example.json"

    options = "r:b:i:"
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
        elif opt == '-r':
            resolution = float(arg)
        elif opt == '-b':
            buffer = int(arg)

    file = json.load(open(mission_data, 'rb'))
    boundarypoints = []
    obstacles = []

    waypoint_dict = file["waypoints"] 
  
    for boundarypoint in file['flyZones'][0]['boundaryPoints']:
        
        boundarypoints += [[boundarypoint["latitude"],
                            boundarypoint["longitude"]]]
    
    boundarypoints.append(list(boundarypoints[0]))

    for obstacle in file["stationaryObstacles"]:
        obstacles += [[obstacle["latitude"],
                        obstacle["longitude"], obstacle["radius"]]]


    map = Map(10, boundarypoints, obstacles, 0)
    rrt_dubins = RRTDubins(waypoint_dict, map, max_iter=3000) 
    

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
   
    start_time = time.time()
    complete_path = []

    for i in range(len(waypoint_dict)-1):

        complete_path.extend(rrt_dubins.planning(rrt_dubins.waypoint_dict[i], rrt_dubins.waypoint_dict[i+1], animation=show_animation))  
        
        if show_animation:  # pragma: no cover
            rrt_dubins.draw_graph(obstacleList)
            plt.plot([x for x in x_list_bound], [y for y in y_list_bound], 'b')
            plt.grid(True)
            plt.pause(1)
        
    elapsed_time = time.time() - start_time
    print(elapsed_time)
    plt.show()   
 
if __name__ == '__main__':
    main(sys.argv[1:])
