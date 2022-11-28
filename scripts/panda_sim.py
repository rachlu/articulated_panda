from franka_interface import ArmInterface
from pddl_setup import *

import rospy
import table_env
import pb_robot
import IPython
import vobj
import util


def reset():
    """
    Reset all objects to their initial conditions.
    """
    for obj in objects:
        objects[obj].set_transform(init_conditions[obj])
    for obj in list(robot.arm.grabbedObjects):
        robot.arm.Release(robot.arm.grabbedObjects[obj])
    robot.arm.SetJointValues(init_conditions['initial_q'])
    path = [arm.joint_angles(), init_conditions['initial_q']]
    path = util.convert(arm, path)
    arm.execute_position_path(path)


if __name__ == '__main__':
    rospy.init_node('testing_node')
    arm = ArmInterface()
    objects, openable, floor, robot = table_env.execute()
    arm.hand.open()
    robot.arm.hand.Open()
    
    global init_conditions
    init_conditions = {'initial_q': robot.arm.GetJointValues()}
    
    for obj in objects:
        init_conditions[obj] = objects[obj].get_transform()

    tamp = TAMP_Functions(robot, objects, floor, openable)

    pddlstream_problem = pddlstream_from_tamp(robot, objects, tamp, arm)
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
            print(obj, pose)

    IPython.embed()
    pb_robot.utils.wait_for_user()
    pb_robot.utils.disconnect()

