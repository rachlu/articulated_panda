from __future__ import print_function

import vobj
import numpy
import IPython
import pb_robot
import os
import table_env

from pddlstream.algorithms.focused import solve_focused
from pddlstream.language.constants import And, Equal, TOTAL_COST, print_solution, PDDLProblem
from pddlstream.language.generator import from_gen_fn, from_fn, from_test
from pddlstream.utils import user_input, read, INF
from pddlstream.algorithms.meta import solve, create_parser


from TAMP_Functions import *

def pddlstream_from_tamp(robot, movable, tamp):
    domain_pddl = read('domain.pddl')
    stream_pddl = read('stream.pddl')

    constant_map = {}
    conf = vobj.BodyConf(robot, robot.arm.GetJointValues())
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
    # goal_config = tamp.sample_config()[0]
    # init += [('Conf', goal_config)]

    # goal =  ('and', ('not', ('HandEmpty', )))
    # goal = ('and', ('AtConf', goal_config))
    # goal = (('Holding', 'plate'))
    # goal = ('On', 'knife', 'knife_region')
    goal = ('and', ('On', 'knife', 'knife_region'), ('On', 'fork', 'fork_region'))

    # goal = ('and', ('On', 'knife', 'knife_region'), ('On', 'fork', 'fork_region'), ('On', 'spoon', 'spoon_region'))

    # goal = ('and', ('On', 'knife', 'knife_region'), ('On', 'fork', 'fork_region'), ('On', 'spoon', 'spoon_region'),
    #         ('On', 'plate', 'plate_region'), ('AtConf', conf))
    # objPoses = {}
    for obj in movable:
        position = vobj.Pose(robot, objects[obj].get_transform())
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
        'collisionCheck': from_gen_fn(tamp.collisionCheck),
        'sampleTable': from_gen_fn(tamp.sampleTable)
    }

    return domain_pddl, constant_map, stream_pddl, stream_map, init, goal


if __name__ == '__main__':
    objects, floor, robot = table_env.execute()
    robot.arm.hand.Open()

    tamp = TAMP_Functions(robot, objects, floor)
    pddlstream_problem = pddlstream_from_tamp(robot, objects, tamp)
    _, _, _, stream_map, init, goal = pddlstream_problem
    print('stream', stream_map)
    print('init', init)
    print('goal', goal)
    solution = solve_focused(pddlstream_problem)
    print_solution(solution)
    plan, cost, evaluations = solution
    print('plan', plan)

    if plan is None:
        print('No plan found')
    else:
        input('Execute?')
        tamp.execute_path(plan)

    IPython.embed()
    pb_robot.utils.wait_for_user()
    pb_robot.utils.disconnect()
