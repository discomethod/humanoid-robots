#!/usr/bin/env python

import copy
import actionlib
import rospy
import numpy as np

from math import sin, cos
from moveit_python import (MoveGroupInterface,
                           PlanningSceneInterface,
                           PickPlaceInterface)
from moveit_python.geometry import rotate_pose_msg_by_euler_angles

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from control_msgs.msg import PointHeadAction, PointHeadGoal
from grasping_msgs.msg import FindGraspableObjectsAction, FindGraspableObjectsGoal
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# Move base using navigation stack
class MoveBaseClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base...")
        self.client.wait_for_server()

    def goto(self, x, y, theta, frame="map"):
        rospy.loginfo('Moving base to (%f, %f, %f)' % (x, y, theta))
        move_goal = MoveBaseGoal()
        move_goal.target_pose.pose.position.x = x-0.5
        move_goal.target_pose.pose.position.y = y
        move_goal.target_pose.pose.orientation.z = sin(theta/2.0)
        move_goal.target_pose.pose.orientation.w = cos(theta/2.0)
        move_goal.target_pose.header.frame_id = frame
        move_goal.target_pose.header.stamp = rospy.Time.now()

        # TODO wait for things to work
        self.client.send_goal(move_goal)
        self.client.wait_for_result()

# Send a trajectory to controller
class FollowTrajectoryClient(object):

    def __init__(self, name, joint_names):
        self.client = actionlib.SimpleActionClient("%s/follow_joint_trajectory" % name,
                                                   FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for %s..." % name)
        self.client.wait_for_server()
        self.joint_names = joint_names

    def move_to(self, positions, duration=5.0):
        if len(self.joint_names) != len(positions):
            print("Invalid trajectory position")
            return False
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[0].positions = positions
        trajectory.points[0].velocities = [0.0 for _ in positions]
        trajectory.points[0].accelerations = [0.0 for _ in positions]
        trajectory.points[0].time_from_start = rospy.Duration(duration)
        follow_goal = FollowJointTrajectoryGoal()
        follow_goal.trajectory = trajectory

        self.client.send_goal(follow_goal)
        self.client.wait_for_result()

# Point the head using controller
class PointHeadClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("head_controller/point_head", PointHeadAction)
        rospy.loginfo("Waiting for head_controller...")
        self.client.wait_for_server()

    def look_at(self, x, y, z, frame, duration=1.0):
        goal = PointHeadGoal()
        goal.target.header.stamp = rospy.Time.now()
        goal.target.header.frame_id = frame
        goal.target.point.x = x
        goal.target.point.y = y
        goal.target.point.z = z
        goal.min_duration = rospy.Duration(duration)
        self.client.send_goal(goal)
        self.client.wait_for_result()

# Tools for grasping
class GraspingClient(object):

    def __init__(self):
        self.scene = PlanningSceneInterface("base_link")
        self.pickplace = PickPlaceInterface("arm", "gripper", verbose=True)
        self.move_group = MoveGroupInterface("arm", "base_link")

        find_topic = "basic_grasping_perception/find_objects"
        rospy.loginfo("Waiting for %s..." % find_topic)
        self.find_client = actionlib.SimpleActionClient(find_topic, FindGraspableObjectsAction)
        self.find_client.wait_for_server()

    def updateScene(self):
        # find objects
        goal = FindGraspableObjectsGoal()
        goal.plan_grasps = True
        self.find_client.send_goal(goal)
        self.find_client.wait_for_result(rospy.Duration(5.0))
        find_result = self.find_client.get_result()

        # remove previous objects
        for name in self.scene.getKnownCollisionObjects():
            self.scene.removeCollisionObject(name, False)
        for name in self.scene.getKnownAttachedObjects():
            self.scene.removeAttachedObject(name, False)
        self.scene.waitForSync()

        # insert objects to scene
        idx = -1
        for obj in find_result.objects:
            idx += 1
            obj.object.name = "object%d"%idx
            self.scene.addSolidPrimitive(obj.object.name,
                                         obj.object.primitives[0],
                                         obj.object.primitive_poses[0],
                                         wait = False)

        for obj in find_result.support_surfaces:
            # extend surface to floor, and make wider since we have narrow field of view
            height = obj.primitive_poses[0].position.z
            obj.primitives[0].dimensions = [obj.primitives[0].dimensions[0],
                                            1.5,  # wider
                                            obj.primitives[0].dimensions[2] + height]
            obj.primitive_poses[0].position.z += -height/2.0

            # add to scene
            self.scene.addSolidPrimitive(obj.name,
                                         obj.primitives[0],
                                         obj.primitive_poses[0],
                                         wait = False)

        self.scene.waitForSync()

        # store for grasping
        self.objects = find_result.objects
        self.surfaces = find_result.support_surfaces

    def getGraspableCube(self):
        graspable = None
        for obj in self.objects:
            # need grasps
            if len(obj.grasps) < 1:
                continue
            # check size
            if obj.object.primitives[0].dimensions[0] < 0.05 or \
               obj.object.primitives[0].dimensions[0] > 0.07 or \
               obj.object.primitives[0].dimensions[0] < 0.05 or \
               obj.object.primitives[0].dimensions[0] > 0.07 or \
               obj.object.primitives[0].dimensions[0] < 0.05 or \
               obj.object.primitives[0].dimensions[0] > 0.07:
                continue
            # has to be on table
            if obj.object.primitive_poses[0].position.z < 0.5:
                continue
            return obj.object, obj.grasps
        # nothing detected
        return None, None

    def getSupportSurface(self, name):
        for surface in self.support_surfaces:
            if surface.name == name:
                return surface
        return None

    def getPlaceLocation(self):
        pass

    def pick(self, block, grasps):
        success, pick_result = self.pickplace.pick_with_retry(block.name,
                                                              grasps,
                                                              support_name=block.support_surface,
                                                              scene=self.scene)
        self.pick_result = pick_result
        return success

    def place(self, block, pose_stamped):
        places = list()
        l = PlaceLocation()
        l.place_pose.pose = pose_stamped.pose
        l.place_pose.header.frame_id = pose_stamped.header.frame_id

        # copy the posture, approach and retreat from the grasp used
        l.post_place_posture = self.pick_result.grasp.pre_grasp_posture
        l.pre_place_approach = self.pick_result.grasp.pre_grasp_approach
        l.post_place_retreat = self.pick_result.grasp.post_grasp_retreat
        places.append(copy.deepcopy(l))
        # create another several places, rotate each by 360/m degrees in yaw direction
        m = 16 # number of possible place poses
        pi = 3.141592653589
        for i in range(0, m-1):
            l.place_pose.pose = rotate_pose_msg_by_euler_angles(l.place_pose.pose, 0, 0, 2 * pi / m)
            places.append(copy.deepcopy(l))

        success, place_result = self.pickplace.place_with_retry(block.name,
                                                                places,
                                                                scene=self.scene)
        return success

    def tuck(self):
        joints = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        pose = [1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0]
        while not rospy.is_shutdown():
            result = self.move_group.moveToJointPosition(joints, pose, 0.02)
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return

class Table(object):
    def __init__(self, x, y, width, length):
        self.width = width + 1.0
        self.length = length + 1.0
        self.center = (x, y)
        self.corners = []
        self.edge_centers = []
        x = self.center[0] + self.width / 2.0
        y = self.center[1] + self.length / 2.0
        self.corners.append((x, y))
        x = self.center[0] + self.width / 2.0
        y = self.center[1] - self.length / 2.0
        self.corners.append((x, y))
        self.bottom_right = (x, y)
        x = self.center[0] - self.width / 2.0
        y = self.center[1] - self.length / 2.0
        self.corners.append((x, y))
        x = self.center[0] - self.width / 2.0
        y = self.center[1] + self.length / 2.0
        self.corners.append((x, y))
        self.top_left = (x, y)

# handle precomputed grasp vectors
class WaiterClient():

    def __init__(self, move_base, table, start_point, distance_vector_file=None):

        self.move_base = move_base
        self.position = start_point
        self.top_left = np.array([table.top_left[0], table.top_left[1]]) # hardcoded corners of the table
        self.bottom_right = np.array([table.bottom_right[0], table.bottom_right[1]])

        #self.table_as = [self.top_left, self.top_left, self.bottom_right, self.bottom_right]
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

        #xn = np.array([1,0])
        #yn = np.array([0,1])
        #self.table_ns = [xn, yn, xn, yn]

    def get_regime(self, x, y):
        # calculate which regime the coordinates (x,y) are in
        # assumes the coordinates are in a valid regime
        """
                  1
            -------------
            |           |
          0 |           | 2
            |           |
            -------------
                  3  
        """
        if x < self.top_left[0]:
            return 0
        if y > self.top_left[1]:
            return 1
        if x < self.bottom_right[0]:
            return 2
        return 3

    def in_table(self, x, y):
        # returns True if the coordinates are inside the table
        if x > self.bottom_right[0] or x < self.top_left[0] or y > self.top_left[1] or y < self.bottom_right[1]:
            return False
        return True

    def get_best_candidate(self, robotx, roboty, blockx, blocky):
        candidates = self.calculate_candidates(robotx, roboty, blockx, blocky)
        best_cost = None
        best_candidate = None
        for candidate_tuple in candidates:
            if best_cost is None or candidate_tuple[1] < best_cost:
                best_candidate = candidate_tuple[0]
                best_cost = candidate_tuple[1]
        return best_candidate

    def calculate_candidates(self, robotx, roboty, blockx, blocky):
        candidates = list() # tuple of (coord, cost)
        #block = np.array([blockx, blocky])
        """
        # generated the intersections of distance vectors from the block and distance vectors
        for table_a in self.table_as:
            for table_n in self.table_ns:
                for distance_vector in self.distance_vectors:
                    # iterate through the distance vectors
                    distance_mag = np.dot(distance_vector, distance_vector)
                    vector_to_line = table_a - block - np.dot((table_a - block),table_n) * table_n
                    distance_to_line = np.dot(vector_to_line, vector_to_line)
                    if distance_to_line > distance_mag:
                        continue
                    b = table_a - block
                    t_solves = np.roots([np.bot(b,b)-distance_mag,2*np.dot(b,table_n),1])
                    for t_solve in t_solves:
                        if t<0: pass
                        if t>1: pass
        """
        for index in range(len(self.distance_vectors)):
            distance_vector = self.distance_vectors[index]
            perp = distance_vector[0]
            trans = distance_vector[1]
            if robotx < self.top_left[0]:
                # regime 0
                #if not self.in_table(blockx-perp, blocky+trans):
                    #candidates.append((np.array([blockx-perp,blocky+trans]),self.distance_costs[index]))
                if not self.in_table(blockx-perp, blocky-trans):
                    candidates.append((np.array([blockx-perp,blocky-trans]),self.distance_costs[index]))
            elif roboty > self.top_left[1]:
                # regime 1
                #if not self.in_table(blockx+trans, blocky-perp):
                    #candidates.append((np.array([blockx+trans,blocky-perp]),self.distance_costs[index]))
                if not self.in_table(blockx-trans, blocky-perp):
                    candidates.append((np.array([blockx-trans,blocky-perp]),self.distance_costs[index]))
            elif robotx < self.bottom_right[0]:
                # regime 2
                if not self.in_table(blockx+perp, blocky+trans):
                    candidates.append((np.array([blockx+perp,blocky+trans]),self.distance_costs[index]))
                #if not self.in_table(blockx+perp, blocky-trans):
                    #candidates.append((np.array([blockx+perp,blocky-trans]),self.distance_costs[index]))
            else:
                # regime 3
                if not self.in_table(blockx+trans, blocky+perp):
                    candidates.append((np.array([blockx+trans,blocky+perp]),self.distance_costs[index]))
                #if not self.in_table(blockx-trans, blocky+perp):
                    #candidates.append((np.array([blockx-trans,blocky+perp]),self.distance_costs[index]))
        return candidates

    def delivery_route(self, fromx, fromy, tox, toy):
        # returns a list of navigation goals to move from from to to
        #delivery_points.append(np.array([tox, toy]))
        if fromx < self.top_left[0] and tox < self.top_left[0]:
            # both on left side of the table
            #delivery_points.append(np.array([tox, toy]))
            return [ np.array([tox, toy]), ]
        if fromx > self.bottom_right[0] and tox > self.bottom_right[0]:
            # both on right side of the table
            #delivery_points.append(np.array([tox, toy]))
            return [ np.array([tox, toy]), ]
        if fromy < self.bottom_right[1] and toy < self.bottom_right[1]:
            # both on bottom side of the table
            #delivery_points.append(np.array([tox, toy]))
            return [ np.array([tox, toy]), ]
        if fromy > self.top_left[1] and toy > self.top_left[1]:
            # both on bop side of the table
            #delivery_points.append(np.array([tox, toy]))
            return [ np.array([tox, toy]), ]
        delivery_points = []
        from_regime = self.get_regime(fromx, fromy)
        to_regime = self.get_regime(tox, toy)
        midx = (self.top_left[0] + self.bottom_right[0])/2.0
        midy = (self.top_left[1] + self.bottom_right[1])/2.0
        if from_regime%2==0:
            # at left or right side
            if self.top_left[1]+self.bottom_right[1] > fromy+toy:
                # use the bottom of the table
                delivery_points.append(np.array([fromx, self.bottom_right[1]]))
                if abs(tox-fromx) > abs(midx-fromx):
                    delivery_points.append(np.array([midx, self.bottom_right[1]]))
                    if abs(tox-fromx) > abs(2*midx-2*fromx):
                        delivery_points.append(np.array([2*midx-fromx, self.bottom_right[1]]))
            else:
                # use the top of the table
                delivery_points.append(np.array([fromx, self.top_left[1]]))
                if abs(tox-fromx) > abs(midx-fromx):
                    delivery_points.append(np.array([midx, self.top_left[1]]))
                    if abs(tox-fromx) > abs(2*midx-2*fromx):
                        delivery_points.append(np.array([2*midx-fromx, self.top_left[1]]))
        else:
            # at top or bottom
            if self.top_left[0]+self.bottom_right[0] > fromx+tox:
                # use the left side of the table
                delivery_points.append(np.array([self.top_left[0],fromy]))
                if abs(toy-fromy) > abs(midy-fromy):
                    delivery_points.append(np.array([self.top_left[0],midy]))
                    if abs(toy-fromy) > abs(2*midy-2*fromy):
                        delivery_points.append(np.array([self.top_left[0],2*midy-fromy]))
            else:
                # use the right side of the table
                delivery_points.append(np.array([self.bottom_right[0],fromy]))
                if abs(toy-fromy) > abs(midy-fromy):
                    delivery_points.append(np.array([self.bottom_right[0],midy]))
                    if abs(toy-fromy) > abs(2*midy-2*fromy):
                        delivery_points.append(np.array([self.bottom_right[0],2*midy-fromy]))
        """
        if (from_regime+to_regime)%2 == 1:
            # adjacent sides
            del delivery_points[-1] # pop the last corner command
        """
        delivery_points.append(np.array([tox, toy]))
        return delivery_points

    def goto(self, dest):
        points = self.delivery_route(self.position[0], self.position[1], dest[0], dest[1])
        for point in points:
            self.move_base.goto(point[0], point[1], 0.0)
        self.position = dest

if __name__ == "__main__":
    # Create a node
    rospy.init_node("waiter")

    # Make sure sim time is working
    while not rospy.Time.now():
        pass

    # Setup clients
    move_base = MoveBaseClient()
    torso_action = FollowTrajectoryClient("torso_controller", ["torso_lift_joint"])
    head_action = PointHeadClient()
    grasping_client = GraspingClient()

    # Move the base to be in front of the table
    # Demonstrates the use of the navigation stack
    table = Table(4.05, 3, 0.913, 2)
    rospy.loginfo("Moving to table...")
    #move_base.goto(1.5,2,0)
    start_point = (5, 3)

    waiter_client = WaiterClient(move_base, table, start_point)
    move_base.goto(start_point[0], start_point[1], 0.0)
    waiter_client.goto((3, 3))

    #`move_base.goto(5.05, 3, 0.0)
    rospy.loginfo("Here now")
    #move_base.goto(2.250, 3.118, 0.0)
    #move_base.goto(2.750, 3.118, 0.0)

    ## Raise the torso using just a controller
    #rospy.loginfo("Raising torso...")
    #torso_action.move_to([0.4, ])

    ## Point the head at the cube we want to pick
    #head_action.look_at(3.7, 3.18, 0.0, "map")

    ## Get block to pick
    #while not rospy.is_shutdown():
    #    rospy.loginfo("Picking object...")
    #    grasping_client.updateScene()
    #    cube, grasps = grasping_client.getGraspableCube()
    #    if cube == None:
    #        rospy.logwarn("Perception failed.")
    #        continue

    #    # Pick the block
    #    if grasping_client.pick(cube, grasps):
    #        break
    #    rospy.logwarn("Grasping failed.")

    ## Tuck the arm
    #grasping_client.tuck()

    ## Lower torso
    #rospy.loginfo("Lowering torso...")
    #torso_action.move_to([0.0, ])

    ## Move to second table
    #rospy.loginfo("Moving to second table...")
    #move_base.goto(-3.53, 3.75, 1.57)
    #move_base.goto(-3.53, 4.15, 1.57)

    ## Raise the torso using just a controller
    #rospy.loginfo("Raising torso...")
    #torso_action.move_to([0.4, ])

    ## Place the block
    #while not rospy.is_shutdown():
    #    rospy.loginfo("Placing object...")
    #    pose = PoseStamped()
    #    pose.pose = cube.primitive_poses[0]
    #    pose.pose.position.z += 0.05
    #    pose.header.frame_id = cube.header.frame_id
    #    if grasping_client.place(cube, pose):
    #        break
    #    rospy.logwarn("Placing failed.")

    ## Tuck the arm, lower the torso
    #grasping_client.tuck()
    #torso_action.move_to([0.0, ])
