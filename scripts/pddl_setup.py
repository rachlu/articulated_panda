from __future__ import print_function
from TAMP_Functions import *
from pddlstream.language.generator import from_gen_fn, from_fn, from_test
from pddlstream.utils import user_input, read, INF
from pddlstream.algorithms.focused import solve_focused
from pddlstream.language.constants import And, Equal, TOTAL_COST, print_solution, PDDLProblem
from pddlstream.language.stream import StreamInfo

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
        ('Handle', 'door', 'knob'),
        ('Handle', 'cabinet', 'top_drawer_knob'),
        ('Handle', 'cabinet', 'bottom_drawer_knob'),
        ('ObjState', 'cabinet', vobj.BodyConf('cabinet', (0, 0))),
        ('ObjState', 'door', vobj.BodyConf('door', (0,))),
    ]
    # goal = ('and', ('Open', 'cabinet', 'bottom_drawer_knob'), ('Open', 'cabinet', 'top_drawer_knob'), ('AtConf', conf))
    # goal = (('Open', 'cabinet', 'bottom_drawer_knob'))
    goal = ('and', ('Close', 'cabinet', 'bottom_drawer_knob'), ('AtConf', conf))
    # goal = ('and', ('Open', 'cabinet', 'top_drawer_knob'), ('Open', 'cabinet', 'bottom_drawer_knob'))
    # goal = (('Holding_Openable', 'cabinet', 'bottom_drawer_knob'))
    # goal = ('and', ('Holding_Openable', 'cabinet', 'top_drawer_knob'), ('AtConf', conf))
    # goal = (('Holding_Openable', 'cabinet', 'bottom_drawer_knob'))
    # goal = (('Open', 'cabinet', 0.25))
    # goal = ('and', ('OpenAll', 'cabinet'), ('Open', 'door', math.pi/5))
    # goal = ('and', ('OpenAll', 'door'), ('Open', 'cabinet', 0.05))
    # goal = ('and', ('OpenAll', 'door'), ('Open', 'cabinet', 0.1))
    # goal = (('Holding', 'bowl'))
    #goal = ('and', ('Open', 'door'), ('AtConf', conf), ('Holding', 'knife'))
    # goal = (('On', 'bowl', 'bowl_region'))
    # goal = ('and', ('On', 'spoon', 'spoon_region'), ('AtConf', conf), ('Open', 'cabinet', 'top_drawer_knob'))

    # goal = ('and', ('On', 'knife', 'knife_region'), ('On', 'fork', 'fork_region'))

    # goal = ('and', ('On', 'knife', 'knife_region'), ('On', 'fork', 'fork_region'), ('On', 'spoon', 'spoon_region'))

    # goal = ('and', ('On', 'knife', 'knife_region'), ('On', 'fork', 'fork_region'), ('On', 'spoon', 'spoon_region'), ('AtConf', conf))
    # goal = ('and', ('Open', 'door'), ('AtConf', conf))
    #goal = ('and', ('On', 'knife', 'knife_region'), ('On', 'fork', 'fork_region'), ('On', 'spoon', 'spoon_region'),
    #        ('On', 'bowl', 'bowl_region'), ('Open', 'door'), ('AtConf', conf))
    # objPoses = {}
    for obj in movable:
        if obj in ['door', 'cabinet']:                
            position = vobj.BodyConf(obj, movable[obj].get_configuration())
            init.extend([
                         ('ObjState', obj, position),
                         ('AtObjState', obj, position)
                         ])
            if obj == 'door' and position.conf[0] == 0:
                init.extend([('Close', 'door', 'knob')])
            
            if obj == 'cabinet':
                if position.conf[0] == 0:
                    init.extend([('Close', 'cabinet', 'top_drawer_knob')])
                if position.conf[1] == 0:
                    init.extend([('Close', 'cabinet', 'bottom_drawer_knob')])
        else:
            init.extend([('Placeable', obj)])
            position = vobj.Pose(robot, movable[obj].get_transform())
            init.extend([('AtObjState', obj, position),
                         ('ObjState', obj, position)
                         ])
        init.extend([('Graspable', obj)])
        
    stream_map = {
        'get_trajectory': from_fn(tamp.calculate_path),
        'sampleGraspPose': from_fn(tamp.sampleGrabPose),
        'inverse-kinematics': from_fn(tamp.computeIK),
        'samplePlacePose': from_fn(tamp.samplePlacePose),
        'get_trajectory_holding': from_fn(tamp.calculate_path_holding),
        'get_trajectory_holding_upright': from_fn(tamp.calculate_path_holding_upright),
        'collisionCheck': from_test(tamp.collisionCheck),
        'sampleTable': from_fn(util.sampleTable),
        'cfree': from_test(tamp.cfreeTraj_Check),
        'cfreeholding': from_test(tamp.cfreeTrajHolding_Check),
        'open_traj': from_fn(tamp.get_openable_traj),
        'close_traj': from_fn(tamp.get_openable_traj),
        'inverse-nonplaceable-kinematics': from_fn(tamp.compute_nonplaceable_IK),
        'sampleGraspOpenable': from_fn(tamp.sample_grasp_openable('Open')),
        'sampleDeltaOpenableConf': from_fn(tamp.sample_delta_openableconf),
        'sampleOpenableConf': from_fn(tamp.sample_openableconf),
        'testOpenEnough': from_test(tamp.test_open_enough),
        'sampleGraspCloseable': from_fn(tamp.sample_grasp_openable('Close')),
        'sampleCloseTransition': from_fn(tamp.sample_close_conf)
    }

    return domain_pddl, constant_map, stream_pddl, stream_map, init, goal
