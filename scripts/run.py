from __future__ import print_function

import vobj
import numpy
import IPython
import pb_robot
import os

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
    # goal = (('Holding', 'knife'))

    goal = ('and', ('On', 'knife', 'knife_region'), ('On', 'fork', 'fork_region'), ('On', 'spoon', 'spoon_region'),
            ('On', 'plate', 'plate_region'), ('AtConf', conf))

    for obj in movable:
        position = vobj.Pose(robot, objects[obj].get_transform())
        init.extend([('Graspable', obj),
                 ('AtPose', obj, position),
                  ('ObjPose', obj, position)
        ])

    stream_map = {
        'get_trajectory': from_gen_fn(tamp.calculate_path),
        'sampleGraspPose': from_gen_fn(tamp.sampleGrabPose),
        'inverse-kinematics': from_gen_fn(tamp.computeIK),
        'samplePlacePose': from_gen_fn(tamp.samplePlacePose)
    }

    return domain_pddl, constant_map, stream_pddl, stream_map, init, goal


if __name__ == '__main__':
    pb_robot.utils.connect(use_gui=True)
    pb_robot.utils.disable_real_time()
    pb_robot.utils.set_default_camera()

    robot = pb_robot.panda.Panda()
    robot.arm.hand.Open()
    # Add floor object
    objects_path = pb_robot.helper.getDirectory("YCB_Robot")
    floor_file = os.path.join(objects_path, 'short_floor.urdf')
    floor = pb_robot.body.createBody(floor_file)

    # # Add fork object
    fork_file = os.path.join(objects_path, 'fork.urdf')
    fork = pb_robot.body.createBody(fork_file)
    fork_pose = numpy.array([[1, 0, 0, 0.3],
                             [0, 0, -1, 0.2],
                             [0, 1, 0, pb_robot.placements.stable_z(fork, floor) + 0.01],
                             [0., 0., 0., 1.]])
    fork.set_transform(fork_pose)

    # Add knife object
    knife_file = os.path.join(objects_path, 'knife.urdf')
    knife = pb_robot.body.createBody(knife_file)
    knife_pose = numpy.array([[1., 0., 0., -0.4],
                              [0., 0., -1., 0.4],
                              [0., 1., 0., pb_robot.placements.stable_z(knife, floor) + 0.01],
                              [0., 0., 0., 1.]])
    knife.set_transform(knife_pose)

    # Add spoon object
    spoon_file = os.path.join(objects_path, 'spoon.urdf')
    spoon = pb_robot.body.createBody(spoon_file)
    spoon_pose = numpy.array([[1, 0, 0, -0.3],
                              [0, 0, -1, -0.2],
                              [0, 1, 0, pb_robot.placements.stable_z(spoon, floor) + 0.01],
                              [0., 0., 0., 1.]])
    spoon.set_transform(spoon_pose)

    # Add plate object
    plate_file = os.path.join(objects_path, 'plate.urdf')
    plate = pb_robot.body.createBody(plate_file)
    plate_pose = numpy.array([[0., 0., 1., 0.8],
                              [0., 1., 0., 0],
                              [1., 0., 0., pb_robot.placements.stable_z(plate, floor)],
                              [0., 0., 0., 1.]])
    plate.set_transform(plate_pose)
    objects = {'fork': fork, 'spoon': spoon, 'knife': knife, 'plate': plate}
    # objects = {'knife': knife, 'fork': fork}
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
