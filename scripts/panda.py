import rospy
from franka_interface import ArmInterface
import table_env
from Grasp import Grasp
import numpy
import pb_robot
from RRT import RRT
import IPython

from pddl_setup import *
import vobj


def convert(path):
    """
    :param path: list of robot configurations
    :return: list of dictionaries of robot configurations
    """
    final_path = []
    for num in range(len(path)):
        q = path[num]
        print(q)
        print(num+1, '/', len(path), '...')
        final_path.append(arm.convertToDict(q))
    return final_path


def execute_plan(plan, g=False):
    """
    Execute a plan. If g is True, sets the start poses of the objects.
    :param plan: Sequence of actions
    :param g: Dictate whether to pick and place or just set the start poses.
    """
    for action in plan:
        for cmd in action.args[-1]:
            cmd.execute()
            ans = input('Redo? (Y/N)')
            while ans.upper() == 'Y':
                cmd.execute()
                ans = input('Redo? (Y/N)')
            if isinstance(cmd, vobj.TrajPath):
                path = convert(cmd.path)
                arm.execute_position_path(path)
            else:
                if action.name == 'grab':
                    arm.hand.grasp(0.02, 40, epsilon_inner=0.1, epsilon_outer=0.1)
                elif action.name == 'place':
                    arm.hand.open()
            time.sleep(1)

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
    path = convert(path)
    arm.execute_position_path(path)


if __name__ == '__main__':
    rospy.init_node('testing_node')
    arm = ArmInterface()
    objects, floor, robot = table_env.execute()
    arm.hand.open()
    robot.arm.hand.Open()
    
    global init_conditions
    init_conditions = {'initial_q': robot.arm.GetJointValues()}
    
    for obj in objects:
        init_conditions[obj] = objects[obj].get_transform()

    tamp = TAMP_Functions(robot, objects, floor)

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

