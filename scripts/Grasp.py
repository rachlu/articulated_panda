#!/usr/bin/env python

import numpy
import random
import math
from tsr.tsr import TSR


def get_relative(world, pose):
    return numpy.dot(numpy.linalg.inv(pose), world)


class Grasp:
    def __init__(self, robot, objects):
        self.objects = objects
        self.robot = robot
        self.relative = {}
        self.bw_range = {}
        self.utensils = ('fork', 'knife', 'spoon')
        #self.utensils = tuple(utensils.intersection(set(objects.keys())))

        self.grasp_tsr = {}
        self.set_info()
        self.set_tsr()

    def set_info(self):
        # bowl
        t_1 = numpy.array([[1, 0, 0, 0],
                           [0, math.cos(math.pi), -math.sin(math.pi), 0],
                           [0, math.sin(math.pi), math.cos(math.pi), 0],
                           [0., 0., 0., 1.]])
        t_2 = numpy.array([[1, 0, 0, 0],
                           [0, math.cos(-math.pi / 7), -math.sin(-math.pi / 7), 0],
                           [0, math.sin(-math.pi / 7), math.cos(-math.pi / 7), 0],
                           [0., 0., 0., 1.]])
        rotation = numpy.dot(t_1, t_2)
        translation = numpy.array([[1, 0, 0, 0],
                                   [0, 1, 0, -.039],
                                   [0, 0, 1, -.17],
                                   [0., 0., 0., 1.]])
        rel = numpy.dot(rotation, translation)
        self.relative[('bowl')] = [rel]

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

        bw = numpy.array([[0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [-math.pi, math.pi]])
        self.bw_range[('bowl')] = bw

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

        bw = numpy.array([[0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0]])
        self.bw_range[self.utensils] = bw

    def set_tsr(self):
        for obj in self.objects:
            self.grasp_tsr[obj] = [TSR(self.objects[obj].get_transform(), self.relative[self.utensils][0], self.bw_range[self.utensils])]
            self.grasp_tsr[obj].append(TSR(self.objects[obj].get_transform(), self.relative[self.utensils][1], self.bw_range[self.utensils]))

        self.grasp_tsr['bowl'] = [TSR(self.objects['bowl'].get_transform(), self.relative['bowl'][0], self.bw_range['bowl'])]
        self.grasp_tsr['bowl'].append(TSR(self.objects['bowl'].get_transform(), self.relative['bowl'][1], self.bw_range['bowl']))

    def grasp(self, obj):
        # r,g,b = x,y,z
        computed_q = None
        for _ in range(50):
            if computed_q is not None:
                return pose, computed_q
            grasp_idx = random.randint(0, 1)
            pose = self.grasp_tsr[obj][grasp_idx].sample()
            computed_q = self.robot.arm.ComputeIK(pose)
        return None, None
        
    
        
    
