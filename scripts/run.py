from __future__ import print_function
from pddl_setup import *
from TAMP_Functions import *

import IPython
import pb_robot
import table_env
from util import Status

def plan_solution(robot, objects, openable, floor):
    max_iteration = 2
    cur_iteration = 1
    tamp = TAMP_Functions(robot, objects, floor, openable)
    minForce = [0, 0, 0, 0, 0, 0]
    placed = set()
    while cur_iteration <= max_iteration:
        print("Starting Iteration", cur_iteration, minForce)
        robot.arm.hand.Open()
        result = iteration(robot, objects, tamp, minForce, placed)
        if result[0] == Status.SUCCESS:
            # Completed
            return
        else:
            if result[1] is not None: # newForce
                minForce = result[1]
            if result[0] == Status.COLLISION:
                placed = result[2]

def iteration(robot, objects, tamp, minForce, placed):
    pddlstream_problem = pddlstream_from_tamp(robot, objects, tamp, None, minForce, placed)
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
    result = util.execute_path(plan, tamp, None, True)
    if result[0] == Status.COLLISION:
        input('Execute Up to?')
        util.execute_path(plan[:result[1]], tamp, None, False)
        return Status.COLLISION, None, result[-1]
    input("Execute all?")
    return util.execute_path(plan, tamp, None, False)

if __name__ == '__main__':
    objects, openable, floor, robot = table_env.execute()
    plan = plan_solution(robot, objects, openable, floor)

    IPython.embed()
    pb_robot.utils.wait_for_user()
    pb_robot.utils.disconnect()
