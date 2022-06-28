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
    final_path = []
    for num in range(len(path)):
        q = path[num]
        print(q)
        print(num+1, '/', len(path), '...')
        final_path.append(arm.convertToDict(q))
    return final_path


def execute_plan(plan, g=False):
    for action in plan:
        time.sleep(1)
        if action.name == 'grab':
            obj, obj_pose, grasp, conf, traj = action.args
            start = vobj.TrajPath(robot, traj.path[:2])
            end = vobj.TrajPath(robot, traj.path[1:])
            start.execute()

            ans = input('Execute Robot? (Y/N)')

            if ans.upper() == 'N':
                break

            start_path = convert(start.path)
            arm.execute_position_path(start_path)

            robot.arm.Grab(objects[obj], grasp.pose)
            robot.arm.hand.Close()

            ans = input('Execute Robot? (Y/N)')
            if ans.upper() == 'N':
                break
            arm.hand.grasp(0.02, 40, epsilon_inner=0.1, epsilon_outer=0.1)

            if g:
                arm.hand.open()
                robot.arm.hand.Open()
            
            end.execute()

            ans = input('Execute Robot? (Y/N)')

            if ans.upper() == 'N':
                break

            end_path = convert(end.path)
            arm.execute_position_path(end_path)
            continue
        if action.name == 'place':
            if g:
                continue
            obj, obj_pose, grasp, conf, traj = action.args
            start = vobj.TrajPath(robot, traj.path[:2])
            end = vobj.TrajPath(robot, traj.path[1:])
            start.execute()

            ans = input('Execute Robot? (Y/N)')

            if ans.upper() == 'N':
                break
            obj_pose.pose[2][-1] -= 0.015
            grasp_in_world = numpy.dot(obj_pose.pose, grasp.pose)
            q = robot.arm.ComputeIK(grasp_in_world, seed_q = traj.path[1])
            q = arm.convertToDict(q)
            arm.move_to_touch(q)
            #start_path = convert(start.path)
            #arm.move_to_touch(start_path[1])
            robot.arm.Release(objects[obj])
            robot.arm.hand.Open()

            ans = input('Execute Robot? (Y/N)')
            if ans.upper() == 'N':
                break
            arm.hand.open()

            end.execute()

            ans = input('Execute Robot? (Y/N)')
            if ans.upper() == 'N':
                break
            end_path = convert(end.path)
            arm.move_from_touch(end_path[1])
            continue

        action.args[-1].execute()
        path = action.args[-1].path

        ans = input('Execute Robot? (Y/N/R)')
        while ans.upper() == 'R':
            action.args[-1].execute()
            ans = input('Execute Robot? (Y/N/R)')
        if ans.upper() == 'N':
            break
        path = convert(path)
        arm.execute_position_path(path)


def reset():
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
            print(obj, (pose[0][-1], pose[1][-1]))

    IPython.embed()
    pb_robot.utils.wait_for_user()
    pb_robot.utils.disconnect()

