#!/usr/bin/env python

import copy
import actionlib
import rospy
import numpy as np
from turtle import *
from math import sqrt

from geometry_msgs.msg import PoseStamped
from waiter import MoveBaseClient
from waiter import FollowTrajectoryClient
from waiter import PointHeadClient
from waiter import GraspingClient

EPSILON = 0.00001
INFINITY = 10000000000
DRAW_SCALE = 60

class Table(object):
    def __init__(self, x, y, width, length):
        # "grow" the obstacle hull by increasing width and length
        # so voronoi path planning can be collision-free
        grown_width = width + 1.0
        grown_length = length + 1.0
        self.center = (x, y)
        self.corners = []
        self.grown_corners = []

        x = self.center[0] + width / 2.0
        y = self.center[1] + length / 2.0
        self.corners.append((x, y))
        x = self.center[0] + width / 2.0
        y = self.center[1] - length / 2.0
        self.corners.append((x, y))
        x = self.center[0] - width / 2.0
        y = self.center[1] - length / 2.0
        self.corners.append((x, y))
        x = self.center[0] - width / 2.0
        y = self.center[1] + length / 2.0
        self.corners.append((x, y))

        x = self.center[0] + grown_width / 2.0
        y = self.center[1] + grown_length / 2.0
        self.grown_corners.append((x, y))
        x = self.center[0] + grown_width / 2.0
        y = self.center[1] - grown_length / 2.0
        self.grown_corners.append((x, y))
        x = self.center[0] - grown_width / 2.0
        y = self.center[1] - grown_length / 2.0
        self.grown_corners.append((x, y))
        x = self.center[0] - grown_width / 2.0
        y = self.center[1] + grown_length / 2.0
        self.grown_corners.append((x, y))

        self.top_left = self.grown_corners[3]
        self.bottom_right = self.grown_corners[1]

        self.left_grown_edge = (self.grown_corners[3], self.grown_corners[2])
        self.right_grown_edge = (self.grown_corners[0], self.grown_corners[1])
        self.bot_grown_edge = (self.grown_corners[1], self.grown_corners[2])
        self.top_grown_edge = (self.grown_corners[0], self.grown_corners[3])

# handle precomputed grasp vectors
class WaiterClient():

    def __init__(self, move_base, table, obstacles, start_point, distance_vector_file=None):

        self.move_base = move_base
        self.position = start_point
        self.table = table
        self.obstacles = obstacles

        self.distance_vectors = list()
        self.distance_costs = list()

        try:
            with open(distance_vector_file, "r") as fin:
                # read in the distance vectors and distance costs
                for line in fin:
                    parse = [float(x) for x in line.split(",")]
                    self.distance_vectors.append(np.array([parse[0],parse[1]]))
                    self.distance_costs.append(parse[2])
        except TypeError:
            rospy.logwarn("Distance vector file not provided, aborting.")
            exit()
        except IOError:
            rospy.logwarn("Distance vector file not found, aborting.")
            exit()

    def in_table(self, point):
        x = point[0]
        y = point[1]
        # returns True if the coordinates are inside the table
        if x >= self.table.bottom_right[0] or x <= self.table.top_left[0] or y >= self.table.top_left[1] or y <= self.table.bottom_right[1]:
            return False
        return True

    def get_best_candidate(self, robotx, roboty, blockx, blocky):
        candidates = self.calculate_candidates(robotx, roboty, blockx, blocky)
        best_cost = None
        best_candidate = None
        if len(candidates) == 0:
            rospy.logwarn("Object is too far into the table to be grasped, aborting.")
            exit()
        for candidate_tuple in candidates:
            if best_cost is None or candidate_tuple[1] < best_cost:
                best_candidate = candidate_tuple[0]
                best_cost = candidate_tuple[1]
        return (best_candidate[0], best_candidate[1], 0.0)

    def _find_vector_intersect(self, point, slope, regime):
        if regime == 0:
            edge = self.table.left_grown_edge
        elif regime == 1:
            edge = self.table.top_grown_edge
        elif regime == 2:
            edge = self.table.right_grown_edge
        elif regime == 3:
            edge = self.table.bot_grown_edge

        corner1 = edge[0]
        corner2 = edge[1]

        if regime == 0 or regime == 2:
            x = corner1[0]
            y = slope * (x - point[0]) + point[1]
        elif regime == 1 or regime == 3:
            y = corner1[1]
            x = (y - point[1])/slope + point[0]
        return (x,y)

    def calculate_candidates(self, robotx, roboty, blockx, blocky):
        # These are the table 'regimes'
        """
                  1
            -------------
            |           |
          0 |           | 2
            |           |
            -------------
                  3
        """

        candidates = list() # tuple of (coord, cost)

        for index in range(len(self.distance_vectors)):
            distance_vector = self.distance_vectors[index]
            perp = distance_vector[0]
            trans = distance_vector[1]
            #regime = self.get_regime(robotx, roboty)

            regimes = [0, 1, 2, 3]
            for regime in regimes:
                if regime == 0 or regime == 2:
                    candidate = self._find_vector_intersect((blockx, blocky), -1.0*(trans/perp), regime)
                    if not self.in_table(candidate):
                        candidates.append((candidate, self.distance_costs[index]))
                elif regime == 1 or regime == 3:
                    candidate = self._find_vector_intersect((blockx, blocky), -1.0*(perp/trans), regime)
                    if not self.in_table(candidate):
                        candidates.append((candidate, self.distance_costs[index]))

        return candidates

    def delivery_route(self, start, goal):

        hull_verts = []
        for obs in self.obstacles:
            hull_verts.append(obs.grown_corners)
        verts, boundaries, edges = get_visibility_graph(start, goal, hull_verts)

        # instantiate weights of each edge based on euclidean distance between vertices
        weights = []
        default_row_weights = [INFINITY for i in range(len(verts))]
        for i in range(len(verts)):
            weights.append(list(default_row_weights))
        for i in range(len(edges)):
            for j in range(len(edges[0])):
                if edges[i][j]:
                    weights[i][j] = distance(verts[i], verts[j])

        # create adjacency list for each vertex
        adjacency = []
        for i in range(len(verts)):
            adjacency.append([])
        for i in range(len(edges)):
            for j in range(len(edges[0])):
                if edges[i][j]:
                    adjacency[i].append(j)
                    adjacency[j].append(i)

        final_path_indices = dijkstra(len(verts), adjacency, weights, verts.index(start), verts.index(goal))
        draw_path(final_path_indices, verts)
        final_path = []
        for index in final_path_indices:
            final_path.append(verts[index])
        return final_path

    def goto(self, dest):
        draw_map(self.position, dest, (10,10), self.obstacles)
        points = self.delivery_route(self.position, dest)
        for point in points:
            if point == dest:
                self.move_base.goto(dest[0], dest[1], dest[2])
            else:
                self.move_base.goto(point[0], point[1], 0.0)
        self.position = dest


# euclidean distance between two points
def distance(a, b):
    return sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

def dijkstra(num_verts, adjacency, weights, start, goal):
    path = {}
    dist = {}
    unvisited = [i for i in range(num_verts)]
    for v in unvisited:
        dist[v] = INFINITY
    dist[start] = 0
    while len(unvisited) > 0:
        v = extract_min(unvisited, dist)
        for w in adjacency[v]:
            if dist[w] > dist[v] + weights[v][w]:
                dist[w] = dist[v] + weights[v][w]
                path[w] = v
    path_vert = goal
    final_verts = [goal]
    while path_vert in path:
        path_vert = path[path_vert]
        final_verts.append(path_vert)
    return final_verts[::-1]

def extract_min(unvisited, dist):
    min_dist = INFINITY
    for v in unvisited:
        if dist[v] < min_dist:
            min_dist = dist[v]
            min_v = v
    unvisited.remove(min_v)
    return min_v

def get_visibility_graph(start, goal, true_hull_verts):

    # Get graph vertices.
    verts = []
    for obs_hull in true_hull_verts:
        verts.extend(obs_hull)
    for x,y in verts:
        setposition(x*DRAW_SCALE,y*DRAW_SCALE)
        dot(3, 'green')
    verts.append(start)
    verts.append(goal)

    # Get graph boundaries.
    boundaries = []
    for obs_hull in true_hull_verts:
        prev = obs_hull[0]
        for v in obs_hull[1:]:
            boundaries.append((verts.index(prev),verts.index(v)))
            prev = v

        boundaries.append((verts.index(prev),verts.index(obs_hull[0])))

    # Populate edges based on intersections.
    edges = []
    row = [False for i in range(len(verts))]
    for i in range(len(verts)):
        edges.append(list(row))

    for i in range(len(edges)):
        for j in range(len(edges[0])):
            if(not intersects(i,j, boundaries, verts, true_hull_verts)):
                edges[i][j] = True
    for i,j in boundaries:
        edges[i][j] = True

    # Draw edges in red.
    color('red')
    for i in range(len(edges)):
        for j in range(len(edges[0])):
            if(edges[i][j] and i>=j):
                v1 = verts[i]
                v2 = verts[j]
                setposition(v1[0]*DRAW_SCALE, v1[1]*DRAW_SCALE)
                pendown()
                goto(v2[0]*DRAW_SCALE, v2[1]*DRAW_SCALE)
                penup()

    return verts, boundaries, edges

def intersects(i, j, boundaries, verts, hulls):
    # Ensure they're not from same hull first.
    v1 = verts[i]
    v2 = verts[j]
    for hull in hulls:
        try:
            i1 = hull.index(v1)
            i2 = hull.index(v2)
            return True
        except:
            pass

    x1 = verts[i][0]
    y1 = verts[i][1]

    x2 = verts[j][0]
    y2 = verts[j][1]

    a1 = y2 - y1
    b1 = x1 - x2
    c1 = a1*x1 + b1*y1

    for m,n in boundaries:
        x3 = verts[m][0]
        y3 = verts[m][1]

        x4 = verts[n][0]
        y4 = verts[n][1]

        a2 = y4-y3
        b2 = x3-x4
        c2 = a2*x3 + b2*y3

        det = a1*b2 - a2*b1

        try:
            x = (b2 * c1 - b1 * c2) / det
            y = (a1 * c2 - a2 * c1) / det
        except:
            continue

        # If they're touching an endpoint.
        if( (abs(x1-x3) < EPSILON and abs(y1-y3) < EPSILON) or
            (abs(x1-x4) < EPSILON and abs(y1-y4) < EPSILON) or
            (abs(x2-x3) < EPSILON and abs(y2-y3) < EPSILON) or
            (abs(x2-x4) < EPSILON and abs(y2-y4) < EPSILON)):
            continue

        elif (min(x1, x2) - EPSILON <= x <= max(x1,x2) + EPSILON) \
            and (min(y1, y2) - EPSILON <= y <= max(y1,y2) + EPSILON) \
            and (min(x3, x4) - EPSILON <= x <= max(x3,x4) + EPSILON) \
            and (min(y3, y4) - EPSILON <= y <= max(y3,y4) + EPSILON):

            return True

    return False

def draw_map(start, goal, dims, obs):
    setup(width=dims[0]*3*DRAW_SCALE, height=dims[1]*2*DRAW_SCALE, startx=0, starty=0)
    tracer(0,0) # so turtle graphics doesn't update the screen after every call
    hideturtle()
    draw_x_y(start)
    draw_x_y(goal)
    for ob in obs:
        draw_obs(ob.corners)
    color('blue')
    for ob in obs:
        draw_obs(ob.grown_corners)
    update()

def draw_obs(verts):
    setposition(verts[0][0]*DRAW_SCALE, verts[0][1]*DRAW_SCALE)
    pendown()
    for v in verts:
        goto(v[0]*DRAW_SCALE, v[1]*DRAW_SCALE)
    goto(verts[0][0]*DRAW_SCALE, verts[0][1]*DRAW_SCALE)
    penup()

def draw_x_y(tup):
    penup()
    setposition(tup[0]*DRAW_SCALE, tup[1]*DRAW_SCALE)
    dot(5, 'blue')

def draw_path(path, vertices):
    tracer(0,0) # so turtle graphics doesn't update the screen after every call
    color('green')
    first_vert = vertices[path[0]]
    setposition(first_vert[0]*DRAW_SCALE, first_vert[1]*DRAW_SCALE)
    pendown()
    for i in range(len(path)):
        v = vertices[path[i]]
        goto(v[0]*DRAW_SCALE, v[1]*DRAW_SCALE)
    update()

if __name__ == "__main__":
    # Create a node
    rospy.init_node("better_waiter")

    # Make sure sim time is working
    while not rospy.Time.now():
        pass

    # Setup clients
    move_base = MoveBaseClient()
    torso_action = FollowTrajectoryClient("torso_controller", ["torso_lift_joint"])
    head_action = PointHeadClient()
    grasping_client = GraspingClient()

    tables = []
    table = Table(5.05, 6, 0.913, 2)
    tables.append(table)
    tables.append(Table(2.5, 2.5, 0.913, 0.913))
    tables.append(Table(2, 5, 0.913, 0.913))
    tables.append(Table(2, 0, 0.913, 2))
    tables.append(Table(5, 2, 2, 0.913))

    cube_loc = (4.8, 6.15)
    dropoff_goal = (0.8,0,0)
    start = (0,0)

    waiter_client = WaiterClient(move_base, table, tables, start, "vectors.txt")
    pickup_goal = waiter_client.get_best_candidate(start[0], start[1], cube_loc[0], cube_loc[1])
    rospy.loginfo("Navigating to pickup location...")
    waiter_client.goto(pickup_goal)

    # Raise the torso using just a controller
    rospy.loginfo("Raising torso...")
    torso_action.move_to([0.4, ])

    rospy.loginfo("Looking at object...")
    # Point the head at the cube we want to pick
    head_action.look_at(cube_loc[0], cube_loc[1], 0.0, "map")

    while not rospy.is_shutdown():
        rospy.loginfo("Picking object...")
        grasping_client.updateScene()
        cube, grasps = grasping_client.getGraspableCube()

        if cube == None:
            rospy.logwarn("Perception failed.")
            continue
        # Pick the block
        if grasping_client.pick(cube, grasps):
            break
        rospy.logwarn("Grasping failed.")

    # Tuck the arm
    grasping_client.tuck()

    rospy.loginfo("Navigating to dropoff location...")
    reset() # reset turtle graphics screen
    waiter_client.goto(dropoff_goal)
    # Place the block
    while not rospy.is_shutdown():
        rospy.loginfo("Placing object...")
        pose = PoseStamped()
        pose.pose = cube.primitive_poses[0]
        pose.pose.position.z += 0.05
        #pose.pose.position.x = goal[0]-1
        #pose.pose.position.y = goal[1]
        pose.header.frame_id = cube.header.frame_id
        if grasping_client.place(cube, pose):
            break
        rospy.logwarn("Placing failed.")

    ## Tuck the arm, lower the torso
    grasping_client.tuck()
    torso_action.move_to([0.0, ])
