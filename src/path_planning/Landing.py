import numpy as np
import math
import matplotlib.path as mpath
import json

from Mapping import Map

# returns the angle from 1 to 2 in rads
def find_line_angle(line):
    # sin^-1(y2 - y1 / hypotenuse)
    angle = math.atan2((line[1][1] - line[0][1]), (line[1][0] - line[0][0]))

    if angle < 0:
        angle += 2 * math.pi

    return angle


# finds the intersection of two lines, ASSUMES THE LINES INTERSECT
def find_intersection(line1, line2):
    # converts both lines to standard for (ax+by+c = 0)
    l1 = standard_form(line1)
    l2 = standard_form(line2)

    # uses [a, b, c] from l1 and l2 to calculate intercept point
    x_intercept = (l1[1] * l2[2] - l2[1] * l1[2]) / (l1[0] * l2[1] - l2[0] * l1[1])
    y_intercept = (l1[2] * l2[0] - l2[2] * l1[0]) / (l1[0] * l2[1] - l2[0] * l1[1])

    return [x_intercept, y_intercept]


# returns the standard form of a line ay + bx + c = 0
def standard_form(line):
    slope = get_slope(line)

    a = 1
    b = slope
    c = (slope * line[0][0] - line[0][1])

    return [a, b, c]


# returns slope of a line from start point to end point
def get_slope(line):
    return (line[1][1] - line[0][1]) / (line[1][0] - line[0][0])


# finds the point at a given angle (in radians) and distance from another point
def find_endpoint(point, angle, distance):
    x_endpoint = point[0] + distance * math.cos(angle)
    y_endpoint = point[1] + distance * math.sin(angle)

    return [x_endpoint, y_endpoint]


# finds the midpoint of a line
def find_midpoint(line):
    return [(line[0][0] + line[1][0]) / 2, (line[0][1] + line[1][1]) / 2]


# TODO properly ref circle, atm circle = [[x,y], radius]
def find_triangle_area(circle, line):
    # gets the center of the circle
    circle_center = circle[:2]

    # herons formula is used to calculate area
    a = math.dist(line[0], circle_center)
    b = math.dist(line[1], circle_center)
    c = math.dist(line[0], line[1])

    s = (a + b + c) / 2

    return (s * (s - a) * (s - b) * (s - c)) ** 0.5

def check_path_intersection(map, obstacles, line):
    line_0_x = int(map.transform_to_map_index(line[0][0]))
    line_0_y = int(map.transform_to_map_index(line[0][1]))
    
    print(line_0_x)
    print(line_0_y)

    if line_0_x < 0 or line_0_x > len(map.obstacle_map)-1 or line_0_y < 0 or line_0_y > len(map.obstacle_map)-1: 
        print("case 1")
        return True

    if map.obstacle_map[line_0_x][line_0_y]:
        print("case 2")
        return True
    '''
    for circle in obstacles:  # iterates through every obstacle

        # gets the parameters of the circle
        circle_center = circle[:2]
        radius = circle[2]

        # gets the parameters of the line
        line_angle = find_line_angle(line)
        line_length = math.dist(line[0], line[1])
        line_midpoint = find_midpoint(line)

        # Extra check to see if either point of the line is within the circle
        for point in line:
            if math.dist(point, circle_center) < radius:
                return True

        # creates a line from the circle center to the line's midpoint
        center_midpoint_line = [circle_center, line_midpoint]
        angleCML = find_line_angle(center_midpoint_line)

        # creates a projection of the line with the midpoint at the circle center
        projection_pt_1 = find_endpoint(circle_center, line_angle, line_length / 2)
        projection_pt_2 = find_endpoint(circle_center, line_angle * -1, line_length / 2)

        # Creates a line with a midpoint that intercepts the circle boundary, will be used to calculate a triangle area
        intercept_line_pt_1 = find_endpoint(projection_pt_1, angleCML, radius)
        intercept_line_pt_2 = find_endpoint(projection_pt_2, angleCML, radius)
        intercept_line = [intercept_line_pt_1, intercept_line_pt_2]

        """
        calculates the area formed by two triangles, both of which contain the circle center as a point. The first contains
        the given line and the second contains the calculated intercept line. If the second has a larger area, then the 
        given line must intercept the circle
        """
        if find_triangle_area(circle, intercept_line) > find_triangle_area(circle, line):
            return True
    ''' 
    # returns false if no intersections are detected
    return False

"""
def convert_obstacles(map):
    obstacles_lat_long = map.obstacles

    obstacles = []

    for x in obstacles_lat_long:
        lat = x["latitude"]
        lon = x["longitude"]
        radius = x["radius"]

        coord = map.decimal_to_cartesian(lat, lon, map.min_lat, map.min_lon)

        obstacle = []
        obstacle.append(coord[0], coord[1], radius)

        obstacles.append(obstacle)

    return obstacles
"""
"""
-inputs-
runway: [startpoint, endpoint], the cartesian coords of the start and end of the runway
glideslope: point, the glideslope point that the robot is flying towards for final approach
radius: the radius at which the robot must approach the glideslope point from

-output-
correction point: the point at which the robot should approach the glideslope from

"""

def find_correction_point(map, runway, glide_slope, radius, obstacles):
    # finds the ideal angle from the glide slope to the correction point, which approaches the glide slope at the same angle as the runway
    ideal_angle = find_line_angle([runway[1], runway[0]])

    potential_c_points = find_possible_approaches(map, ideal_angle, radius, glide_slope, obstacles)

    # TODO add buffer here

    for x in range(len(potential_c_points)):
        if potential_c_points[x]:
            return potential_c_points[x]

    print("No correction points found")
    return None


def find_possible_approaches(map, ideal_angle, radius, glide_slope, obstacles):
    possible_c_points = []

    # tests the first point which is the ideal approach
    ideal_c_point = find_endpoint(glide_slope, ideal_angle, radius)
    if check_path_intersection(map, obstacles, [ideal_c_point, glide_slope]):
        possible_c_points.append(ideal_c_point)
    else:
        possible_c_points.append(False)

    # finds all points at a given radius from the
    for radian in range(0, math.pi, 2 * math.pi / 150):

        # finds two approaches at equal angles from the ideal approach

        approachAngle1 = ideal_angle + radian
        approach1 = [glide_slope, find_endpoint(glide_slope, approachAngle1, radius)]
        approachAngle2 = ideal_angle - radian
        approach2 = [glide_slope, find_endpoint(glide_slope, approachAngle2, radius)]

        # checks if either Cpoint intersects an obstacle on its approach, appends false if it does

        if check_path_intersection(map, obstacles, approach1):
            possible_c_points.append(approach1[1])
        else:
            possible_c_points.append(False)

        if check_path_intersection(map, obstacles, approach2):
            possible_c_points.append(approach2[1])
        else:
            possible_c_points.append(False)

    return possible_c_points


def calc_descent(alt_final, alt_initial=None, dist=None, theta=None):
    if dist is not None and alt_initial is not None:
        return np.arctan(abs(alt_initial - alt_final) / dist) * 180 / np.pi
    elif theta is not None and alt_initial is not None:
        return abs(alt_initial - alt_final) / np.tan(theta * np.pi / 180)
    elif dist is not None and theta is not None:
        return alt_final + dist * np.tan(theta * np.pi / 180)
    else:
        return 0

def calc_landing(map, obstacles, start_pos, runway, max_angle):
    start_alt = start_pos[2]

    run_start = runway[0]
    run_end = runway[1]

    run_start_xy = list(map.decimal_to_cartesian(run_start[0], run_start[1], map.min_lat, map.min_lon))
    run_end_xy = list(map.decimal_to_cartesian(run_end[0], run_end[1], map.min_lat, map.min_lon))

    run_x = run_start_xy[0] - run_end_xy[0]
    run_y = run_start_xy[1] - run_end_xy[1]
    run_axis = [run_x / np.sqrt(run_x**2 + run_y**2), run_y / np.sqrt(run_x**2 + run_y**2)]

    START_GUESS = 1000
    STEP_SIZE = 50

    max_path = [[run_axis[i] * START_GUESS + run_end_xy[i] for i in range(2)], run_end_xy]
    glide_path = max_path

    print(run_start_xy)
    print(glide_path)
    while check_path_intersection(map, obstacles, glide_path):
        glide_path[0][0] -= run_axis[0] * STEP_SIZE
        glide_path[0][1] -= run_axis[1] * STEP_SIZE
        print(glide_path)

    glide_path[0][0] -= run_axis[0] * STEP_SIZE
    glide_path[0][1] -= run_axis[1] * STEP_SIZE
    if check_path_intersection(map, obstacles, glide_path):
        glide_path[0][0] += run_axis[0] * STEP_SIZE
        glide_path[0][1] += run_axis[1] * STEP_SIZE

    glide_angle = calc_descent(alt_final=0, alt_initial=start_alt, dist=math.dist(glide_path[0], glide_path[1]))

    correction_point = None
    glide_alt = start_alt
    if (glide_angle > max_angle):
        glide_alt = calc_descent(alt_final=0, dist=math.dist(glide_path[0], glide_path[1]), theta=max_angle)
        radius = calc_descent(alt_final=glide_alt, alt_initial=start_alt, theta=max_angle)
        correction_point = find_correction_point(map, runway, glide_path, radius, obstacles)

    landing_coords = []
    if correction_point is not None:
        correct_dict = dict()
        correct_coords = map.cartesian_to_decimal(correction_point[0], correction_point[1], map.min_lat, map.min_lon)
        correct_dict["latitude"] = correct_coords[0]
        correct_dict["longitude"] = correct_coords[1]
        correct_dict["altitude"] = start_alt
        landing_coords.append(correct_dict)
    
    glide_dict = dict()
    glide_coords = map.cartesian_to_decimal(glide_path[0][0], glide_path[0][1], map.min_lat, map.min_lon)
    glide_dict["latitude"] = glide_coords[0]
    glide_dict["longitude"] = glide_coords[1]
    glide_dict["altitude"] = glide_alt
    landing_coords.append(glide_dict)

    td_dict = dict()
    td_dict["latitude"] = run_end[0]
    td_dict["longitude"] = run_end[1]
    td_dict["altitude"] = 0
    landing_coords.append(td_dict)
    
    return landing_coords

if __name__ == "__main__":
    file = json.load(open("mission_plan\interop_example.json", 'rb'))
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
    
    for obstacle in file["stationaryObstacles"]:
        obstacles += [[obstacle["latitude"],
                        obstacle["longitude"], obstacle["radius"]]]

    map = Map(10, boundarypoints, obstacles, 0)
    print(len(map.obstacle_map))
    print(calc_landing(map, obstacles, waypoints[len(waypoints)-1], [waypoints[0], waypoints[1]], 15))