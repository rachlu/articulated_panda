#!/usr/bin/env python

import numpy
import random
import math
from tsr.tsr import TSR
import util

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
        t_1 = util.get_rotation_arr('X', math.pi)
        t_2 = util.get_rotation_arr('X', -math.pi/7)
        rotation = numpy.dot(t_1, t_2)

        # Relative translation of the robot from the bowl
        translation = numpy.array([[1, 0, 0, 0],
                                   [0, 1, 0, -.039],
                                   [0, 0, 1, -.17],
                                   [0., 0., 0., 1.]])
        rel = numpy.dot(rotation, translation)
        self.relative[('bowl')] = [rel]

        # Rotation 180 degrees of the robot end-effector
        t_3 = util.get_rotation_arr('Z', math.pi)
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
        rotation = util.get_rotation_arr('Y', math.pi)
        translation = numpy.array([[1, 0, 0, 0],
                                   [0, 1, 0, 0],
                                   [0, 0, 1, -.125],
                                   [0., 0., 0., 1.]])
        rel = numpy.dot(rotation, translation)
        self.relative[self.utensils] = [rel]

        z = util.get_rotation_arr('Z', math.pi)
        rotation = numpy.dot(z, rotation)
        translation = numpy.array([[1, 0, 0, 0],
                                   [0, 1, 0, 0],
                                   [0, 0, 1, -.125],
                                   [0., 0., 0., 1.]])
        rel = numpy.dot(rotation, translation)

        self.relative[self.utensils].append(rel)

        bw = numpy.array([[-0.03, 0.03], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0]])
        self.bw_range[self.utensils] = bw

        # Cabinet
        translation = numpy.array([[1, 0, 0, 0],
                                   [0, 1, 0, -0.125],
                                   [0, 0, 1, -0.275],
                                   [0., 0., 0., 1.]])
        t1 = util.get_rotation_arr('X', math.pi / 2)
        t2 = util.get_rotation_arr('Y', math.pi / 2)
        rotation = numpy.dot(t1, t2)
        rel = numpy.dot(rotation, translation)
        self.relative['cabinet_bottom'] = [rel]

        translation = numpy.array([[1, 0, 0, 0],
                                   [0, 1, 0, 0.125],
                                   [0, 0, 1, -0.275],
                                   [0., 0., 0., 1.]])
        rel = numpy.dot(rotation, translation)
        self.relative['cabinet_top'] = [rel]

        t1 = util.get_rotation_arr('X', 3*math.pi/2)
        #y, z, x
        translation = numpy.array([[1, 0, 0, 0],
                                   [0, 1, 0, 0.125],
                                   [0, 0, 1, -0.275],
                                   [0., 0., 0., 1.]])
        rotation = numpy.dot(t1, t2)
        rel = numpy.dot(rotation, translation)
        self.relative['cabinet_bottom'].append(rel)

        translation = numpy.array([[1, 0, 0, 0],
                                   [0, 1, 0, -0.125],
                                   [0, 0, 1, -0.275],
                                   [0., 0., 0., 1.]])
        rel = numpy.dot(rotation, translation)
        self.relative['cabinet_top'].append(rel)

        bw = numpy.array([[0, 0], [0, 0], [-0.018, 0.018], [0, 0], [0, 0], [0, 0]])
        self.bw_range['cabinet_bottom'] = bw
        self.bw_range['cabinet_top'] = bw

        # Door

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
        
    
        
    
