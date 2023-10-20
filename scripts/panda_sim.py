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

def plan_solution(arm, robot, objects, openable, floor):
    iteration += 1
    if iteration >= max_iteration:
        print("Reached Max iteration. Failed")
        return

    print("Starting Iteration", iteration)

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
    stream_info = {'cfree': StreamInfo(negate=True), 'cfreeholding': StreamInfo(negate=True),
                   'collisionCheck': StreamInfo(negate=True)}
    solution = solve_focused(pddlstream_problem, stream_info=stream_info, planner='ff-astar')
    print_solution(solution)
    plan, cost, evaluations = solution
    print('plan', plan)
    return plan

if __name__ == '__main__':
    rospy.init_node('testing_node')
    arm = ArmInterface()
    global iteration 
    iteration = 1
    global max_iteration
    max_iteration = 2
    objects, openable, floor, robot = table_env.execute()
    plan = plan_solution(arm, robot, objects, openable, floor)
    if plan is None:
        print('No plan found')
    else:
        for obj in objects:
            pose = objects[obj].get_transform()
            print(obj, pose)

    # util.execute_path(plan, objects, arm) To execute plan
    IPython.embed()
    pb_robot.utils.wait_for_user()
    pb_robot.utils.disconnect()

