from __future__ import print_function

import vobj
import numpy
import IPython
import pb_robot
import os

from TAMP_Functions import *

from pddlstream.algorithms.focused import solve_focused
from pddlstream.language.constants import And, Equal, TOTAL_COST, print_solution, PDDLProblem
from pddlstream.language.generator import from_gen_fn, from_fn, from_test
from pddlstream.utils import user_input, read, INF
from pddlstream.algorithms.meta import solve, create_parser


def pddlstream_from_tamp(robot, movable, tamp, panda = None):
    domain_pddl = read('domain.pddl')
    stream_pddl = read('stream.pddl')

    constant_map = {}
    if panda is None:
        conf = vobj.BodyConf(robot, robot.arm.GetJointValues())
    else:
        robot.arm.SetJointValues(panda.convertToList(panda.joint_angles()))
        conf = vobj.BodyConf(robot, panda.convertToList(panda.joint_angles()))
    # ('Pose', obj, obj.GetTransform()),

    init = [
        ('Conf', conf),
        ('AtConf', conf),
        ('HandEmpty',),
        ('Region', 'spoon_region'),
        ('Region', 'fork_region'),
        ('Region', 'plate_region'),
        ('Region', 'knife_region')
    ]
    #goal_config = tamp.sample_config()[0]
    #init += [('Conf', goal_config)]

    # goal =  ('and', ('not', ('HandEmpty', )))
    #goal = ('and', ('AtConf', goal_config))
    #goal = (('Holding', 'knife'))
    #goal = (('On', 'fork', 'fork_region'))
    goal = ('and', ('On', 'knife', 'knife_region'), ('On', 'fork', 'fork_region'))

    # goal = ('and', ('On', 'knife', 'knife_region'), ('On', 'fork', 'fork_region'), ('On', 'spoon', 'spoon_region'))

    #goal = ('and', ('On', 'knife', 'knife_region'), ('On', 'fork', 'fork_region'), ('On', 'spoon', 'spoon_region'), ('On', 'plate', 'plate_region'), ('AtConf', conf))
    # objPoses = {}
    for obj in movable:
        position = vobj.Pose(robot, movable[obj].get_transform())
        # objPoses[obj] = position
        init.extend([('Graspable', obj),
                 ('AtPose', obj, position),
                  ('ObjPose', obj, position)
        ])
    # init += ('ObjPoses', objPoses)
    stream_map = {
        'get_trajectory': from_gen_fn(tamp.calculate_path),
        'sampleGraspPose': from_gen_fn(tamp.sampleGrabPose),
        'inverse-kinematics': from_gen_fn(tamp.computeIK),
        'samplePlacePose': from_gen_fn(tamp.samplePlacePose),
        'get_trajectory_holding': from_gen_fn(tamp.calculate_path_holding),
        'collisionCheck': from_test(tamp.collisionCheck),
        'sampleTable': from_gen_fn(tamp.sampleTable)
    }

    return domain_pddl, constant_map, stream_pddl, stream_map, init, goal
