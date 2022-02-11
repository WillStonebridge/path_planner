import copy
from importlib.resources import path
import math
from operator import index
import os
from pickle import TRUE
import random
import sys
import json
import time
from tkinter.tix import Tree
from xxlimited import new

# point = Point(0.5, 0.5)
# polygon = Polygon([(0, 0), (0, 1), (1, 1), (1, 0)])
# print(polygon.contains(point))

import matplotlib.pyplot as plt
import numpy as np

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../DubinsPath/")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../RRT/")
sys.path.append(os.path.dirname(os.path.abspath(__file__)))


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

    def __init__(self, start, goal, obstacle_list, rand_area, obstacle_map,map,
                 goal_sample_rate=10,
                 max_iter=1000,
                 ):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Random Sampling Area [min,max]

        """
        self.start = self.Node(start[0], start[1], start[2])
        self.end = self.Node(goal[0], goal[1], goal[2])
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.obstacle_list = obstacle_list
        self.obstacle_map = obstacle_map
        self.map = map

        self.curvature = 1/25 # for dubins path
        self.goal_yaw_th = np.deg2rad(1)
        self.goal_xy_th = 0.5

    def planning(self, animation=True, search_until_max_iter=True, boundarypointslist=[],obstacleslist=[]):
        """
        execute planning

        animation: flag for animation on or off
        """
        last_index = False
        self.node_list = [self.start]
        
        
        for i in range(self.max_iter):
            # print(i)
            # print("hi")
            # i += 1
            # print("Iter:", i, ", number of nodes:", len(self.node_list))
            rnd = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd)
            temp_node = self.steer(self.node_list[nearest_ind], rnd)
            # print('hello')
    
            if temp_node:
                index_x = self.map.transform_to_map_index(temp_node.x)
                index_y = self.map.transform_to_map_index(temp_node.y)

                if self.check_path(temp_node) and self.check_collision(temp_node, self.obstacle_list) and self.check_boundary([round(index_x),round(index_y)]):
                    self.node_list.append(temp_node)
                    new_node = temp_node

            # if animation and i % 5 == 0:
            #     self.plot_start_goal_arrow()
            #     self.draw_graph(rnd)

        if new_node:  
                # print('hi')# check reaching the goal
            last_index = self.search_best_goal_node()
            if last_index:
                return self.generate_final_course(last_index)

       # print("reached max iteration")

        # last_index = self.search_best_goal_node()
        # if last_index:
        #     return self.generate_final_course(last_index)
        # else:
        #     print("Cannot find path")

        # return None

    def draw_graph(self, rnd=None):  # pragma: no cover
        # plt.clf()
        # # for stopping simulation with the esc key.
        # plt.gcf().canvas.mpl_connect('key_release_event',
        #         lambda event: [exit(0) if event.key == 'escape' else None])
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
        # for node in self.node_list:
        #     if node.parent:
        #         plt.plot(node.path_x, node.path_y, "-g")

        for (ox, oy, size) in self.obstacle_list:
            plt.plot(ox, oy, "ok", ms= size)

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.axis([0, 2000, 0, 2000])
        plt.grid(True)
        self.plot_start_goal_arrow()
        plt.pause(0.01)

    def plot_start_goal_arrow(self):  # pragma: no cover
        dubins_path_planning.plot_arrow(
            self.start.x, self.start.y, self.start.yaw)
        dubins_path_planning.plot_arrow(
            self.end.x, self.end.y, self.end.yaw)
    
    def check_path(self, new_node):
       if new_node == None:
            return False
       print(len(new_node.path_x))
       for i in range(len(new_node.path_x)):
            # try:
            # print(new_node.path_x[i],new_node.path_y[i])
            index_x = self.map.transform_to_map_index(new_node.path_x[i])
            index_y = self.map.transform_to_map_index(new_node.path_y[i])
            print(new_node.path_x[i],new_node.path_y[i])
            x = (self.check_boundary([round(index_x),round(index_y)]))
            if x == False:
                return False
       return True
        


            #     if self.obstacle_map[round(index_x)][round(index_y)]:
            #         return False
            # except:
            #     return False
            
            # return True
        
        

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
        # print("final")
        path = [[self.end.x, self.end.y,self.end.yaw]]
        node = self.node_list[goal_index]
        while node.parent:
            for (ix, iy,iyaw) in zip(reversed(node.path_x), reversed(node.path_y),reversed(node.path_yaw)):
                path.append([ix, iy,iyaw*180/math.pi])
            node = node.parent
        path.append([self.start.x, self.start.y, self.start.yaw*180/math.pi])
        return path


def main():



    altitude = 200


    mission_data = "interop_example.json"
    file = json.load(open(mission_data, 'rb'))
    
    waypoints = []
    boundarypoints = []
    obstacles = []
    for waypoint in file["waypoints"]:
            waypoints += [[waypoint["latitude"],
                                waypoint["longitude"], waypoint["altitude"]]]

    for boundarypoint in file['flyZones'][0]['boundaryPoints']:
        
        boundarypoints += [[boundarypoint["latitude"],
                            boundarypoint["longitude"]]]
        
    boundarypoints.append(list(boundarypoints[0]))
    
    boundarypoints_same = boundarypoints

    for obstacle in file["stationaryObstacles"]:
        obstacles += [[obstacle["latitude"],
                        obstacle["longitude"], obstacle["radius"]]]

    min = np.amin(boundarypoints, axis = 0)

    # print('\n\n')

    map = Map(10, boundarypoints, obstacles, 0)

    # print(boundarypoints)

    x_list_bound = []
    y_list_bound = []

    for boundary in boundarypoints:

        x_list_bound.append(boundary[0])
        y_list_bound.append(boundary[1])

    x_list_way = []
    y_list_way = []

    for waypoint in waypoints:
        x,y = map.decimal_to_cartesian(waypoint[0],waypoint[1],min[0],min[1])
        x_list_way.append(x)
        y_list_way.append(y)
    
    x_list_obs = []
    y_list_obs = []
    radiuses = []

    for obstacle in obstacles:
        x_list_obs.append(obstacle[0])
        y_list_obs.append(obstacle[1])
        radiuses.append(obstacle[2])
    
    
    # fig, ax = plt.subplots()

    # for i in range(len(x_list_bound)):
    #     plt.plot(x_list_bound[i],y_list_bound[i], '.b')

    
    # for i in range(len(x_list_way)):
    #     plt.plot(x_list_way[i],y_list_way[i], '.k')

    # for i in range(len(x_list_obs)):
    #     plt.plot(x_list_obs[i],y_list_obs[i], '.r')
    
    # plt.grid(True)
    # plt.axis("equal")
    # plt.show()

    obstacleList = []  # [x,y,size(radius)]

    for i in range(len(x_list_obs)):
         #try making this circles
        obstacleList.append((x_list_obs[i],y_list_obs[i],radiuses[i]/10))

    # print(obstacleList)
    # print(boundarypoints)
    obstacle_map = map.calc_obstacle_map([],boundarypoints,0)

    paths = []
    
    final_paths = {}
    start_time = time.time()
    blank = '['
    for i in range(len(x_list_way)-1):
        print("Waypoint " + str(i))
        # print(i)

        new_path = []

    # Set Initial parameters
        if i == 0:
            start = [x_list_way[i], y_list_way[i], np.deg2rad(0.0)]
            goal = [x_list_way[i+1], y_list_way[i+1], np.arctan2(x_list_way[i+1]-x_list_way[i],y_list_way[i+1]-y_list_way[i])]


        else:
            start = [x_list_way[i], y_list_way[i], np.arctan2(x_list_way[i]-x_list_way[i-1],y_list_way[i]-y_list_way[i-1])]

            goal = [x_list_way[i+1], y_list_way[i+1], np.arctan2(x_list_way[i+1]-x_list_way[i],y_list_way[i+1]-y_list_way[i])]

        


        rrt_dubins = RRTDubins(start, goal, obstacleList, [0, 2000],obstacle_map,map)
        path = rrt_dubins.planning(animation=show_animation)

        index_list = []
        # print(path)
        for point in path:

            index_x = map.transform_to_map_index(point[0])
            index_y = map.transform_to_map_index(point[1])
            index_list.append([index_x,index_y])
     
            
            lat, long = map.cartesian_to_decimal(point[0],point[1],min[0],min[1])
            new_point = {'latitude': str(lat), 'longtitude':str(long), 'altitude':str(altitude)}

            

            blank += json.JSONEncoder().encode(new_point)
            blank += ','
        
        if show_animation:  # pragma: no cover
            rrt_dubins.draw_graph()

             

            # for node in valid:
            #     plt.plot(node[0], node[1], '.k')
    

            plt.plot([x for x in x_list_bound], [y for y in y_list_bound], 'b')
            plt.plot([x for (x, y,_) in path], [y for (x, y,_) in path], '-r')
            plt.grid(True)
            plt.pause(0.001)
    plt.show()    
        # if show_animation:  # pragma: no cover
        #         rrt_dubins.draw_graph()

                

        #         # for node in valid:
        #         #     plt.plot(node[0], node[1], '.k')
        

        #         plt.plot([x for x in x_list_bound], [y for y in y_list_bound], '-b')
        #         plt.plot([x for (x, y,_) in path], [y for (x, y,_) in path], '-r')
        #         plt.grid(True)
        #         plt.pause(0.001)
        #         plt.show()

    blank = blank[:-1]
    blank += ']'

    elapsed_time = time.time() - start_time
    print(elapsed_time)
    with open('RRT_out.json', 'w') as file:
        file.write(blank)   
         
        
            
            
            
        
        # print(new_path)

  

        


        

        # Draw final path
    

 

            
    # print(valid)
 
    
    

if __name__ == '__main__':
    main()
