from __future__ import print_function
from TAMP_Functions import *
from pddlstream.language.generator import from_gen_fn, from_fn, from_test
from pddlstream.utils import user_input, read, INF
from pddlstream.algorithms.focused import solve_focused
from pddlstream.language.constants import And, Equal, TOTAL_COST, print_solution, PDDLProblem
from pddlstream.language.stream import StreamInfo

import numpy as np
import vobj
import util

def pddlstream_from_tamp(robot, movable, tamp, panda=None, minForce=None, placed=set()):
    domain_pddl = read('domain.pddl')
    stream_pddl = read('stream.pddl')

    # self.state = [robot, movable, tamp, panda]

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
        ('CabinetRegion', 'top_drawer_knob'),
        ('CabinetRegion', 'bottom_drawer_knob'),
        ('UprightObj', 'bowl'),
        ('Openable', 'door'),
        ('Openable', 'cabinet'),
        ('Handle', 'door', 'knob'),
        ('Handle', 'cabinet', 'top_drawer_knob'),
        ('Handle', 'cabinet', 'bottom_drawer_knob'),
        ('ObjState', 'cabinet', vobj.BodyConf('cabisnet', (0, 0))),
        ('ObjState', 'door', vobj.BodyConf('door', (0,))),
    ]
    # goal = ('and', ('In', 'fork', 'cabinet', 'bottom_drawer_knob'), ('Close', 'cabinet', 'bottom_drawer_knob'), ('AtConf', conf))
    # goal = ('and', ('In', 'fork', 'cabinet', 'bottom_drawer_knob'), ('Open', 'cabinet', 'bottom_drawer_knob'))
    # goal = ('and', ('Close', 'cabinet', 'bottom_drawer_knob'))
    # goal = (('On', 'fork', 'fork_region'))
    goal = ('and', ('Open', 'cabinet', 'bottom_drawer_knob'))

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
                else:
                    init.extend([('Open', 'cabinet', 'top_drawer_knob')])
                if position.conf[1] == 0:
                    init.extend([('Close', 'cabinet', 'bottom_drawer_knob')])
                else:
                    init.extend([('Open', 'cabinet', 'bottom_drawer_knob')])
        else:
            init.extend([('Placeable', obj)])
            if obj in placed:
                init.extend([('In', obj, 'cabinet', 'bottom_drawer_knob')])
                continue
            position = vobj.Pose(robot, movable[obj].get_transform())
            init.extend([('AtObjState', obj, position),
                        ('ObjState', obj, position)
                        ])
        init.extend([('Graspable', obj)])
    if minForce is not None:
        print("minForce", minForce)
        init.extend([('ForceNeeded', 'cabinet', 'bottom_drawer_knob', minForce)])
        init.extend([('ForceNeeded', 'cabinet', 'top_drawer_knob', minForce)])
    else:
        init.extend([('ForceNeeded', 'cabinet', 'bottom_drawer_knob', np.zeros(6))])
        init.extend([('ForceNeeded', 'cabinet', 'top_drawer_knob', np.zeros(6))])

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
        'open_traj': from_fn(tamp.get_open_traj_merge),
        'close_traj': from_fn(tamp.get_close_traj_merge),
        'sampleOpenableConf': from_fn(tamp.sample_openableconf),
        'sampleCloseTransition': from_fn(tamp.sample_close_conf),
        'samplePlaceCabinetPose': from_fn(tamp.samplePlaceCabinetPose),
        'randomRobotConf': from_fn(tamp.randomConf)
    }

    return domain_pddl, constant_map, stream_pddl, stream_map, init, goal
