#!/usr/bin/env python

import numpy
import random
import math
from tsr.tsr import TSR

class Grasp:
    def __init__(self, robot, objects):
        self.objects = objects
        self.robot = robot
        self.relative = {}
        self.bw_range = {}
        self.utensils = ('fork', 'knife', 'spoon')
        self.grasp_tsr = {}
        self.set_info()

    def set_info(self):
        # bowl

        # Relative rotation of the robot from the bowl
        t_1 = numpy.array([[1, 0, 0, 0],
                           [0, math.cos(math.pi), -math.sin(math.pi), 0],
                           [0, math.sin(math.pi), math.cos(math.pi), 0],
                           [0., 0., 0., 1.]])
        t_2 = numpy.array([[1, 0, 0, 0],
                           [0, math.cos(-math.pi / 7), -math.sin(-math.pi / 7), 0],
                           [0, math.sin(-math.pi / 7), math.cos(-math.pi / 7), 0],
                           [0., 0., 0., 1.]])
        rotation = numpy.dot(t_1, t_2)

        # Relative translation of the robot from the bowl
        translation = numpy.array([[1, 0, 0, 0],
                                   [0, 1, 0, -.039],
                                   [0, 0, 1, -.17],
                                   [0., 0., 0., 1.]])
        rel = numpy.dot(rotation, translation)
        self.relative[('bowl')] = [rel]

        # Rotation 180 degrees of the robot end-effector
        t_3 = numpy.array([[math.cos(math.pi), -math.sin(math.pi), 0, 0],
                           [math.sin(math.pi), math.cos(math.pi), 0, 0],
                           [0, 0, 1, 0],
                           [0., 0., 0., 1.]])
        rotation = numpy.linalg.multi_dot([t_1, t_2, t_3])
        translation = numpy.array([[1, 0, 0, 0],
                                   [0, 1, 0, .039],
                                   [0, 0, 1, -.17],
                                   [0., 0., 0., 1.]])
        rel = numpy.dot(rotation, translation)
        self.relative[('bowl')].append(rel)

        # TSR range
        bw = numpy.array([[0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [-math.pi, math.pi]])
        self.bw_range[('bowl')] = bw

        # utensils
        rotation = numpy.array([[math.cos(math.pi), 0, math.sin(math.pi), 0],
                           [0, 1, 0, 0],
                           [-math.sin(math.pi), 0, math.cos(math.pi), 0],
                           [0., 0., 0., 1.]])
        translation = numpy.array([[1, 0, 0, 0],
                                   [0, 1, 0, 0],
                                   [0, 0, 1, -.125],
                                   [0., 0., 0., 1.]])
        rel = numpy.dot(rotation, translation)
        self.relative[self.utensils] = [rel]

        z = numpy.array([[math.cos(math.pi), -math.sin(math.pi), 0, 0],
                      [math.sin(math.pi), math.cos(math.pi), 0, 0],
                      [0, 0, 1, 0],
                      [0., 0., 0., 1.]])
        rotation = numpy.dot(z, rotation)
        translation = numpy.array([[1, 0, 0, 0],
                                   [0, 1, 0, 0],
                                   [0, 0, 1, -.125],
                                   [0., 0., 0., 1.]])
        rel = numpy.dot(rotation, translation)

        self.relative[self.utensils].append(rel)

        bw = numpy.array([[-0.03, 0.03], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0]])
        self.bw_range[self.utensils] = bw

        # Door
        translation = numpy.array([[1, 0, 0, 0],
                                   [0, 1, 0, -0.13],
                                   [0, 0, 1, -0.29],
                                   [0., 0., 0., 1.]])
        t1 = numpy.array([[1, 0, 0, 0],
                           [0, math.cos(math.pi/2), -math.sin(math.pi/2), 0],
                           [0, math.sin(math.pi/2), math.cos(math.pi/2), 0],
                           [0., 0., 0., 1.]])
        t2 = numpy.array([[math.cos(math.pi/2), 0, math.sin(math.pi/2), 0],
                           [0, 1, 0, 0],
                           [-math.sin(math.pi/2), 0, math.cos(math.pi/2), 0],
                           [0., 0., 0., 1.]])
        rotation = numpy.dot(t1, t2)
        rel = numpy.dot(rotation, translation)
        self.relative['door'] = [rel]

        t1 = numpy.array([[1, 0, 0, 0],
                           [0, math.cos(3*math.pi/2), -math.sin(3*math.pi/2), 0],
                           [0, math.sin(3*math.pi/2), math.cos(3*math.pi/2), 0],
                           [0., 0., 0., 1.]])
        #y, z, x
        translation = numpy.array([[1, 0, 0, 0],
                                   [0, 1, 0, -0.13],
                                   [0, 0, 1, -0.29],
                                   [0., 0., 0., 1.]])
        rotation = numpy.dot(t1, t2)
        rel = numpy.dot(rotation, translation)
        self.relative['door'].append(rel)
        bw = numpy.array([[0, 0], [-0.03, 0.03], [0, 0], [0, 0], [0, 0], [0, 0]])
        self.bw_range['door'] = bw

    def set_tsr(self, obj, pose):
        if obj in self.utensils:
            # Obj Pose, Relative Pose, Range
            self.grasp_tsr[obj] = [TSR(pose, self.relative[self.utensils][0], self.bw_range[self.utensils])]
            self.grasp_tsr[obj].append(TSR(pose, self.relative[self.utensils][1], self.bw_range[self.utensils]))
        else:
            self.grasp_tsr[obj] = [TSR(pose, self.relative[obj][0], self.bw_range[obj])]
            self.grasp_tsr[obj].append(TSR(pose, self.relative[obj][1], self.bw_range[obj]))

    def grasp(self, obj, pose):
        # Sample grasp of obj
        # r,g,b = x,y,z
        self.set_tsr(obj, pose)
        computed_q = None
        for _ in range(50):
            if computed_q is not None:
                return grasp_world, computed_q
            grasp_idx = random.randint(0, 1)
            # Grasp in world frame
            grasp_world = self.grasp_tsr[obj][grasp_idx].sample()
            computed_q = self.robot.arm.ComputeIK(grasp_world)
        return None, None
        
    
        
    
