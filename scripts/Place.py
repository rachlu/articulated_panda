import math
import numpy
import random
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
        import Placements.Front, Placements.Left, Placements.Right

        choice = random.choice([Placements.Left, Placements.Right, Placements.Front])

        self.relative['bowl'] = choice.bowl_place
    
        self.bw_range['bowl'] = choice.bowl_bw
        
        self.relative['fork'] = choice.fork_place

        self.relative['knife'] = choice.knife_place

        self.relative['spoon'] = choice.spoon_place

        self.bw_range[self.utensils] = choice.utensils_bw

    def set_tsr(self):
        # Object in world frame
        t_ow = self.table.get_transform()
        for obj in self.objects:
            self.place_tsr[obj] = TSR(t_ow, self.relative[obj], self.bw_range[self.utensils])

        self.place_tsr['bowl'] = TSR(numpy.dot(t_ow, self.relative['bowl']), numpy.identity(4), self.bw_range['bowl'])

    def samplePlacePose(self, obj):
        return self.place_tsr[obj].sample()
