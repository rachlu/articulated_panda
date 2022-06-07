import numpy
import random
import math
from tsr.tsr import TSR
from RRT import RRT


def get_relative(world, pose):
    return numpy.dot(numpy.linalg.inv(pose), world)


class Place:
    def __init__(self, robot, objects, table):
        self.objects = objects
        self.table = table
        self.robot = robot
        self.relative = {}
        self.bw_range = {}
        self.utensils = ('fork', 'knife', 'spoon')
        self.place_tsr = {}
        self.set_info()
        self.set_tsr()

    def set_info(self):
        # Relative offset to valid position. Object in table frame
        t_ee = numpy.array([[1, 0, 0, -.4],
                            [0, 1, 0, .1],
                            [0, 0, 1, 0],
                            [0., 0., 0., 1.]])
        self.relative['plate'] = t_ee
        # Allowable range for placement
        bw = numpy.array([[0, 0], [-.04, .04], [0, 0], [0, 0], [0, 0], [-math.pi, math.pi]])

        self.bw_range['plate'] = bw

        t_ee = numpy.array([[1, 0, 0, -.4],
                            [0, math.cos(math.pi / 2), -math.sin(math.pi / 2), .3],
                            [0, math.sin(math.pi / 2), math.cos(math.pi / 2), .02],
                            [0., 0., 0., 1.]])
        self.relative['fork'] = t_ee

        t_ee = numpy.array([[1, 0, 0, -.4],
                            [0, math.cos(math.pi / 2), -math.sin(math.pi / 2), -.1],
                            [0, math.sin(math.pi / 2), math.cos(math.pi / 2), .02],
                            [0., 0., 0., 1.]])
        self.relative['knife'] = t_ee

        t_ee = numpy.array([[1, 0, 0, -.4],
                            [0, math.cos(math.pi / 2), -math.sin(math.pi / 2), -.3],
                            [0, math.sin(math.pi / 2), math.cos(math.pi / 2), .02],
                            [0., 0., 0., 1.]])
        self.relative['spoon'] = t_ee

        bw = numpy.array([[0, 0], [-.01, .01], [0, 0], [0, 0], [0, 0], [0, 0]])
        self.bw_range[self.utensils] = bw

    def set_tsr(self):
        # Object in world frame
        t_ow = self.table.get_transform()
        for obj in self.objects:
            self.place_tsr[obj] = TSR(t_ow, self.relative[obj], self.bw_range[self.utensils])

        self.place_tsr['plate'] = TSR(numpy.dot(t_ow, self.relative['plate']), numpy.identity(4), self.bw_range['plate'])

    def samplePlacePose(self, obj):
        return self.place_tsr[obj].sample()
