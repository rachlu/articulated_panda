#!/usr/bin/env python

import numpy
import random
import math
from tsr.tsr import TSR
from RRT import RRT


def get_relative(world, pose):
    return numpy.dot(numpy.linalg.inv(pose), world)

class Grasp:
    def __init__(self, robot, objects):
        self.objects = objects
        self.robot = robot
        self.relative = {}
        self.bw_range = {}
        self.utensils = ('fork', 'knife', 'spoon')
        self.grasp_tsr = {}
        self.set_info()
        self.set_tsr()

    def set_info(self):
        # plate
        t_o = self.objects.get('plate').get_transform()
        t_ee = numpy.array([[math.cos(math.pi / 2), -math.sin(math.pi / 2), 0, .70],
                            [math.sin(math.pi / 2), math.cos(math.pi / 2), 0, 0],
                            [0, 0, 1, .18],
                            [0., 0., 0., 1.]])
        t_e = numpy.array([[1, 0, 0, 0],
                           [0, math.cos(math.pi), -math.sin(math.pi), 0],
                           [0, math.sin(math.pi), math.cos(math.pi), 0],
                           [0., 0., 0., 1.]])
        rel = get_relative(numpy.dot(t_ee, t_e), t_o)
        self.relative[('plate')] = [rel]

        t_ee = numpy.array([[math.cos(3 * math.pi / 2), -math.sin(3 * math.pi / 2), 0, .70],
                            [math.sin(3 * math.pi / 2), math.cos(3 * math.pi / 2), 0, 0],
                            [0, 0, 1, .18],
                            [0., 0., 0., 1.]])
        t_e = numpy.array([[1, 0, 0, 0],
                           [0, math.cos(math.pi), -math.sin(math.pi), 0],
                           [0, math.sin(math.pi), math.cos(math.pi), 0],
                           [0., 0., 0., 1.]])

        rel = get_relative(numpy.dot(t_ee, t_e), t_o)
        self.relative[('plate')].append(rel)

        bw = numpy.array([[0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [-math.pi, math.pi]])
        self.bw_range[('plate')] = bw

        t_ee = numpy.array([[math.cos(math.pi), -math.sin(math.pi), 0, -.4],
                            [math.sin(math.pi), math.cos(math.pi), 0, .4],
                            [0, 0, 1, .15],
                            [0., 0., 0., 1.]])
        t_e = numpy.array([[math.cos(math.pi), 0, math.sin(math.pi), 0],
                           [0, 1, 0, 0],
                           [-math.sin(math.pi), 0, math.cos(math.pi), 0],
                           [0., 0., 0., 1.]])
        rel = get_relative(numpy.dot(t_ee, t_e), t_o)
        self.relative[self.utensils] = [rel]

        t_o = self.objects.get('knife').get_transform()
        t_ee = numpy.array([[math.cos(math.pi), -math.sin(math.pi), 0, -.4],
                            [math.sin(math.pi), math.cos(math.pi), 0, .4],
                            [0, 0, 1, .15],
                            [0., 0., 0., 1.]])
        rel = get_relative(numpy.dot(t_ee, t_e), t_o)

        self.relative[self.utensils].append(rel)

        bw = numpy.array([[-0.12, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0]])
        self.bw_range[self.utensils] = bw

    def set_tsr(self):
        for obj in self.objects:
            self.grasp_tsr[obj] = [TSR(self.objects[obj].get_transform(), self.relative[self.utensils][0], self.bw_range[self.utensils])]
            self.grasp_tsr[obj].append(TSR(self.objects[obj].get_transform(), self.relative[self.utensils][1], self.bw_range[self.utensils]))

        self.grasp_tsr['plate'] = [TSR(self.objects['plate'].get_transform(), self.relative['plate'][0], self.bw_range['plate'])]
        self.grasp_tsr['plate'].append(TSR(self.objects['plate'].get_transform(), self.relative['plate'][1], self.bw_range['plate']))

    def grasp(self, obj):
        # r,g,b = x,y,z
        computed_q = None
        while computed_q is None:
            grasp_idx = random.randint(0, 1)
            pose = self.grasp_tsr[obj][grasp_idx].sample()
            computed_q = self.robot.arm.ComputeIK(pose)
        return pose, computed_q
        
    
        
    
