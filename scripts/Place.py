import numpy
import random
from tsr.tsr import TSR


class Place:
    def __init__(self, robot, objects, table):
        self.objects = objects
        self.table = table
        self.robot = robot
        self.relative = {}
        self.bw_range = {}
        self.utensils = ('fork', 'knife', 'spoon')
        self.place_tsr = {}

    def set_table_info(self):
        import Placements.Front, Placements.Left, Placements.Right
        self.t_ow = self.table.get_transform()

        choice = random.choice([Placements.Left, Placements.Right, Placements.Front])
        # choice = random.choice([Placements.Left, Placements.Right])
        self.relative['bowl'] = choice.bowl_place
    
        self.bw_range['bowl'] = choice.bowl_bw
        
        self.relative['fork'] = choice.fork_place

        self.relative['knife'] = choice.knife_place

        self.relative['spoon'] = choice.spoon_place

        self.bw_range[self.utensils] = choice.utensils_bw

    def set_cabinet_info(self, region, conf):
        import Placements.Cabinet
        knob = 'top_drawer_knob' if 'top' in region else 'bottom_drawer_knob'
        old_conf = self.objects['cabinet'].get_configuration()
        self.objects['cabinet'].set_configuration(conf)
        self.t_ow = self.objects['cabinet'].link_from_name(knob).get_link_tform(True)
        choice = Placements.Cabinet

        self.relative['bowl'] = choice.bowl_place
    
        self.bw_range['bowl'] = choice.bowl_bw
        
        self.relative['fork'] = choice.fork_place

        self.relative['knife'] = choice.knife_place

        self.relative['spoon'] = choice.spoon_place

        self.bw_range[self.utensils] = choice.utensils_bw
        self.objects['cabinet'].set_configuration(old_conf)

    def set_tsr(self):
        # Object in world frame
        for obj in ['fork', 'spoon', 'knife']:
            self.place_tsr[obj] = TSR(self.t_ow, self.relative[obj], self.bw_range[self.utensils])

        self.place_tsr['bowl'] = TSR(numpy.dot(self.t_ow, self.relative['bowl']), numpy.identity(4), self.bw_range['bowl'])

    def samplePlacePose(self, obj, region, conf=None):
        # Object pose in world frame
        if 'knob' in region:
            self.set_cabinet_info(region, conf)
        else:
            self.set_table_info()
        self.set_tsr()
        return self.place_tsr[obj].sample()
