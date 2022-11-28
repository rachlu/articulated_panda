from __future__ import print_function
from pddl_setup import *
from TAMP_Functions import *

import IPython
import pb_robot
import table_env


if __name__ == '__main__':
    objects, openable, floor, robot = table_env.execute()
    robot.arm.hand.Open()

    tamp = TAMP_Functions(robot, objects, floor, openable)
    pddlstream_problem = pddlstream_from_tamp(robot, objects, tamp)
    _, _, _, stream_map, init, goal = pddlstream_problem
    print('stream', stream_map)
    print('init', init)
    print('goal', goal)
    stream_info = {'cfree': StreamInfo(negate=True), 'cfreeholding': StreamInfo(negate=True),
                   'collisionCheck': StreamInfo(negate=True)}
    solution = solve_focused(pddlstream_problem, stream_info=stream_info, planner='ff-astar')
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
        util.execute_path(plan, objects, None)

    IPython.embed()
    pb_robot.utils.wait_for_user()
    pb_robot.utils.disconnect()
