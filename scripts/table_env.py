#!/usr/bin/env python

# from __future__ import print_function

import os
import IPython
import pb_robot
import numpy
import math
import util
import vobj

from Plan import Plan
from Grasp import Grasp
from RRT import RRT
from Place import Place

def collision_free(objects, obj):
    for obj2 in objects:
        if pb_robot.collisions.body_collision(obj2, obj):
            return False
        else:
            x = obj.get_transform()[0][-1]
            y = obj.get_transform()[1][-1]
            x2 = obj2.get_transform()[0][-1]
            y2 = obj2.get_transform()[1][-1]
            if math.sqrt((x-x2)**2+(y-y2)**2) < 0.15:
                return False
    return True


def execute():
    # Launch pybullet
    pb_robot.utils.connect(use_gui=True)
    pb_robot.utils.disable_real_time()
    pb_robot.utils.set_default_camera()
    pb_robot.viz.draw_pose(pb_robot.geometry.pose_from_tform(numpy.eye(4)), length=0.5, width=10)

    # Create robot object
    robot = pb_robot.panda.Panda()

    # Add floor object 
    objects_path = pb_robot.helper.getDirectory("YCB_Robot")
    floor_file = os.path.join(objects_path, 'short_floor.urdf')
    floor = pb_robot.body.createBody(floor_file)
    floor_pose = floor.get_transform()
    floor_pose[0][3] += 0.16764
    floor.set_transform(floor_pose)

    # Add fork object
    fork_file = os.path.join(objects_path, 'fork.urdf')
    fork = pb_robot.body.createBody(fork_file)
    # fork_pose = numpy.array([[1, 0, 0, 0.4],
    #                          [0, 0, -1, -0.45],
    #                          [0, 1, 0, 0.024],
    #                          [0., 0., 0., 1.]])
    while not collision_free([robot], fork):
        random_pos = util.sampleTable('fork')[0][0].pose
        print(random_pos)
        fork.set_transform(random_pos)

    # Add knife object
    knife_file = os.path.join(objects_path, 'knife.urdf')
    knife = pb_robot.body.createBody(knife_file)
    # knife_pose = numpy.array([[1., 0., 0., -0.2],
    #                           [0., 0., -1., 0.3],
    #                           [0., 1., 0., 0.024],
    #                           [0., 0., 0., 1.]])
    while not collision_free([fork, robot], knife):
        random_pos = util.sampleTable('knife')[0][0].pose
        knife.set_transform(random_pos)

    # Add spoon object
    spoon_file = os.path.join(objects_path, 'spoon.urdf')
    spoon = pb_robot.body.createBody(spoon_file)
    # spoon_pose = numpy.array([[1, 0, 0, 0.35],
    #                           [0, 0, -1, 0.4],
    #                           [0, 1, 0, 0.024],
    #                           [0., 0., 0., 1.]])
    while not collision_free([fork, knife, robot], spoon):
        random_pos = util.sampleTable('spoon')[0][0].pose
        spoon.set_transform(random_pos)

    # Add bowl object
    bowl_file = os.path.join(objects_path, 'bowl.urdf')
    bowl = pb_robot.body.createBody(bowl_file)
    # bowl_pose = numpy.array([[0., 0., 1., 0.2],
    #                           [0., 1., 0., -0.3],
    #                           [1., 0., 0., pb_robot.placements.stable_z(bowl, floor)],
    #                           [0., 0., 0., 1.]])
    # bowl_pose = numpy.array([[1., 0., 0, 0],
    #                           [0., 0, -1, 0],
    #                           [0, 1, 0, pb_robot.placements.stable_z(bowl, floor)],
    #                           [0., 0., 0., 1.]])
    # bowl_pose = vobj.Pose('_', bowl_pose)
    # bowl.set_transform(bowl_pose.pose)
    while not collision_free([knife, spoon, fork, robot], bowl):
        random_pos = util.sampleTable('bowl')[0][0].pose
        bowl.set_transform(random_pos)

    objects = {'fork': fork, 'spoon': spoon, 'knife': knife, 'bowl': bowl}

    return objects, floor, robot
