from __future__ import print_function

import vobj
import numpy
import IPython
import pb_robot
import os
import table_env

from TAMP_Functions import *
from pddl_setup import *

if __name__ == '__main__':
    objects, floor, robot = table_env.execute()
    robot.arm.hand.Open()

    tamp = TAMP_Functions(robot, objects, floor)
    pddlstream_problem = pddlstream_from_tamp(robot, objects, tamp)
    _, _, _, stream_map, init, goal = pddlstream_problem
    print('stream', stream_map)
    print('init', init)
    print('goal', goal)
    solution = solve_focused(pddlstream_problem, planner='ff-astar')
    print_solution(solution)
    plan, cost, evaluations = solution
    print('plan', plan)

    if plan is None:
        print('No plan found')
    else:
        for obj in objects:
            pose = objects[obj].get_transform()
            print(obj, (pose[0][-1], pose[1][-1]))
        input('Execute?')
        tamp.execute_path(plan)

    IPython.embed()
    pb_robot.utils.wait_for_user()
    pb_robot.utils.disconnect()
