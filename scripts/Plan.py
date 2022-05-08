import os
import IPython
import pb_robot
import numpy

import time
import pybullet
from RRT import RRT
from Grasp import Grasp
from Place import Place


class Plan:
    def __init__(self, robot, objects=None, floor=None):
        self.objects = objects
        self.robot = robot
        self.default = {}
        self.path = {}
        self.relative_grasps = {}
        self.floor = floor

    def set_default(self):
        for obj in self.objects:
            self.default[obj] = self.objects[obj].get_transform()
        self.default['robot'] = self.robot.arm.GetJointValues()

    def reset_env(self):
        for item in self.objects:
            self.objects[item].set_transform(self.default[item])
        self.robot.arm.SetJointValues(self.default['robot'])

    def plan_path(self, obj):
        place = Place(self.robot, self.objects, self.floor)
        grasp = Grasp(self.robot, self.objects)
        moves = {}
        q_start = numpy.array(self.robot.arm.GetJointValues())

        # Pick
        self.robot.arm.hand.Open()
        motion = RRT(self.robot)
        new_path = None
        while new_path is None:
            grasp_pose, q_grasp = grasp.grasp(obj)
            new_path = motion.motion(q_start, q_grasp)
        self.execute_path(new_path)
        moves['pick'] = new_path
        self.robot.arm.hand.Close()

        # Place
        new_path = None
        while new_path is None:
            place_pose = place.samplePlacePose(obj)
            relative_grasp = numpy.dot(numpy.linalg.inv(self.objects[obj].get_transform()), grasp_pose)
            self.relative_grasps[obj] = relative_grasp
            q_goal = self.robot.arm.ComputeIK(numpy.dot(place_pose, self.relative_grasps[obj]))
            while not self.robot.arm.IsCollisionFree(q_goal):
                place_pose = place.place_tsr[obj].sample()

                self.relative_grasps[obj] = numpy.dot(numpy.linalg.inv(self.objects[obj].get_transform()), grasp_pose)
                q_goal = self.robot.arm.ComputeIK(numpy.dot(place_pose, self.relative_grasps[obj]))
            new_path = motion.motion(q_grasp, q_goal)
        self.robot.arm.Grab(self.objects[obj], self.relative_grasps[obj])
        self.execute_path(new_path)
        moves['place'] = new_path
        self.robot.arm.Release(self.objects[obj])
        self.path[obj] = moves

    def execute_path(self, q_list):
        if q_list is None:
            print("No path")
            return
        # input("Execute path?")
        for q in q_list:
            self.robot.arm.SetJointValues(q)
            time.sleep(.1)

    def execute_path_all(self):
        for item in self.objects:
            for q in self.path[item]['pick']:
                self.robot.arm.SetJointValues(q)
                time.sleep(1)
            self.robot.arm.hand.Close()
            self.robot.arm.Grab(self.objects[item], self.relative_grasps[item])
            for q in self.path[item]['place']:
                self.robot.arm.SetJointValues(q)
                time.sleep(1)
            self.robot.arm.Release(self.objects[item])
            self.robot.arm.hand.Open()
        self.robot.arm.SetJointValues(self.default['robot'])

    def plan(self):
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, False)
        print('Calculating Path...')
        for obj in self.objects:
            print(obj)
            self.plan_path(obj)
        self.reset_env()
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, True)
        print("Done")
