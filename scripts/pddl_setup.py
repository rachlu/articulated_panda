from __future__ import print_function
from TAMP_Functions import *
from pddlstream.language.generator import from_gen_fn, from_fn, from_test
from pddlstream.utils import user_input, read, INF
from pddlstream.algorithms.focused import solve_focused
from pddlstream.language.constants import And, Equal, TOTAL_COST, print_solution, PDDLProblem

import vobj
import util
import math


def pddlstream_from_tamp(robot, movable, tamp, panda=None):
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
        ('CanMove',),
        ('Region', 'spoon_region'),
        ('Region', 'fork_region'),
        ('Region', 'bowl_region'),
        ('Region', 'knife_region'),
        ('UprightObj', 'bowl'),
        ('Openable', 'door'),
        ('Openable', 'cabinet'),
        ('Open_Amount', 'door', math.pi/5),
        ('Open_Amount', 'cabinet', 0.1),
        ('Open_Amount', 'cabinet', 0),
        ('Open_Amount', 'cabinet', 0.25),
        ('OpenAllAmount', 'door', math.pi/10),
        ('OpenAllAmount', 'cabinet', 0.25)
    ]
    goal = (('Open', 'cabinet', 0.1))
    # goal = (('Holding', 'cabinet'))
    # goal = (('Open', 'cabinet', 0.25))
    # goal = ('and', ('OpenAll', 'cabinet'), ('Open', 'door', math.pi/5))
    # goal = ('and', ('OpenAll', 'door'), ('Open', 'cabinet', 0.05))
    # goal = ('and', ('OpenAll', 'door'), ('Open', 'cabinet', 0.1))
    # goal = (('Holding', 'spring'))
    #goal = ('and', ('Open', 'door'), ('AtConf', conf), ('Holding', 'knife'))
    #goal = (('On', 'bowl', 'bowl_region'))
    # goal = ('and', ('On', 'spoon', 'spoon_region'), ('Open', 'door'))

    # goal = ('and', ('On', 'knife', 'knife_region'), ('On', 'fork', 'fork_region'))

    # goal = ('and', ('On', 'knife', 'knife_region'), ('On', 'fork', 'fork_region'), ('On', 'spoon', 'spoon_region'))

    # goal = ('and', ('On', 'knife', 'knife_region'), ('On', 'fork', 'fork_region'), ('On', 'spoon', 'spoon_region'), ('AtConf', conf))
    # goal = ('and', ('Open', 'door'), ('AtConf', conf))
    #goal = ('and', ('On', 'knife', 'knife_region'), ('On', 'fork', 'fork_region'), ('On', 'spoon', 'spoon_region'),
    #        ('On', 'bowl', 'bowl_region'), ('Open', 'door'), ('AtConf', conf))
    # objPoses = {}
    for obj in movable:
        if obj in ['door', 'cabinet']:
            position = vobj.Pose(robot, movable[obj].get_configuration())
        else:
            position = vobj.Pose(robot, movable[obj].get_transform())
        # objPoses[obj] = position
        init.extend([('AtPose', obj, position),
                     ('ObjPose', obj, position)
                     ])
        if obj not in ['door', 'cabinet']:
            init.extend([('Placeable', obj)])
        init.extend([('Graspable', obj)])
    # init += ('ObjPoses', objPoses)
    stream_map = {
        'get_trajectory': from_gen_fn(tamp.calculate_path),
        'sampleGraspPose': from_gen_fn(tamp.sampleGrabPose),
        'inverse-kinematics': from_gen_fn(tamp.computeIK),
        'samplePlacePose': from_gen_fn(tamp.samplePlacePose),
        'get_trajectory_holding': from_gen_fn(tamp.calculate_path_holding),
        'get_trajectory_holding_upright': from_gen_fn(tamp.calculate_path_holding_upright),
        'collisionCheck': from_test(tamp.collisionCheck),
        'sampleTable': from_gen_fn(util.sampleTable),
        'cfree': from_test(tamp.cfreeTraj_Check),
        'cfreeholding': from_test(tamp.cfreeTrajHolding_Check),
        'open_traj': from_gen_fn(tamp.get_open_traj)
    }

    return domain_pddl, constant_map, stream_pddl, stream_map, init, goal
