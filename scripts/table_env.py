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

    while not collision_free([robot], fork):
        random_pos = util.sampleTable('fork')[0][0].pose
        fork.set_transform(random_pos)
    #fork.set_transform([[ 0.35800727,  0.9337188,   0.,         -0.48509734],
    #                    [-0.9337188,   0.35800727, -0.,          0.30019573],
    #                    [-0.,          0.,          1.,          0.        ],
    #                    [ 0.,          0.,          0.,          1.        ]])


    
    # Add knife object
    knife_file = os.path.join(objects_path, 'knife.urdf')
    knife = pb_robot.body.createBody(knife_file)
    while not collision_free([fork, robot], knife):
        random_pos = util.sampleTable('knife')[0][0].pose
        knife.set_transform(random_pos)
    #knife.set_transform([[ 0.05352872,  0.99856631,  0.,          0.27479426],
    #                    [-0.99856631,  0.05352872, -0.,         -0.30335207],
    #                    [-0.,          0.,          1.,          0.        ],
    #                    [ 0.,          0.,          0.,          1.        ]])

    # Add spoon object
    spoon_file = os.path.join(objects_path, 'spoon.urdf')
    spoon = pb_robot.body.createBody(spoon_file)
    while not collision_free([fork, knife, robot], spoon):
        random_pos = util.sampleTable('spoon')[0][0].pose
        spoon.set_transform(random_pos)
    #spoon.set_transform([[ 0.62888441,  0.77749881,  0.,          0.64620689],
    #                    [-0.77749881,  0.62888441, -0.,          0.41487665],
    #                    [-0.,          0.,          1.,          0.        ],
    #                    [ 0.,          0.,          0.,          1.        ]])


    # Add bowl object
    bowl_file = os.path.join(objects_path, 'bowl.urdf')
    bowl = pb_robot.body.createBody(bowl_file)
    while not collision_free([knife, spoon, fork, robot], bowl):
        random_pos = util.sampleTable('bowl')[0][0].pose
        bowl.set_transform(random_pos)
    #bowl.set_transform([[ 0.81460261, -0.58001947,  0.,          0.42764541],
    #                    [ 0.58001947,  0.81460261,  0.,          0.42623734],
    #                    [ 0.,          0.,          1.,          0.        ],
    #                    [ 0.,          0.,          0.,          1.        ]])


    objects = {'fork': fork, 'spoon': spoon, 'knife': knife, 'bowl': bowl}

    return objects, floor, robot
