from __future__ import print_function
from pddl_setup import *
from TAMP_Functions import *

import IPython
import pb_robot
import table_env

def plan_solution(robot, objects, openable, floor):
    max_iteration = 2
    cur_iteration = 1
    tamp = TAMP_Functions(robot, objects, floor, openable)
    minForce = [0, 0, 0, 0, 0, 0]
    while cur_iteration <= max_iteration:
        print("Starting Iteration", cur_iteration, minForce)
        robot.arm.hand.Open()
        result, newForce = iteration(robot, objects, tamp, minForce)
        if result:
            # Completed
            return
        else:
            if newForce:
                minForce = newForce

def iteration(robot, objects, tamp, minForce):
    pddlstream_problem = pddlstream_from_tamp(robot, objects, tamp, None, minForce)
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
    return util.execute_path(plan, tamp, None)

if __name__ == '__main__':
    objects, openable, floor, robot = table_env.execute()
    plan = plan_solution(robot, objects, openable, floor)

    IPython.embed()
    pb_robot.utils.wait_for_user()
    pb_robot.utils.disconnect()
