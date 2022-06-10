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

def execute_path_panda(path):
    final_path = []
    for num in range(len(path)):
        q = path[num]
        print(q)
        print(num+1, '/', len(path), '...')
        final_path.append(arm.convertToDict(q))
    arm.execute_position_path(final_path)

if __name__ == '__main__':
    rospy.init_node('testing_node')
    arm = ArmInterface()
    objects, floor, robot = table_env.execute()
    arm.hand.open()
    robot.arm.hand.Open()

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
        input('Execute?')
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
                execute_path_panda(start.path)

                robot.arm.Grab(objects[obj], grasp.pose)
                robot.arm.hand.Close()

                ans = input('Execute Robot? (Y/N)')
                if ans.upper() == 'N':
                    break
                arm.hand.grasp(0.02, 40, epsilon_inner=0.1, epsilon_outer=0.1)

                end.execute()

                ans = input('Execute Robot? (Y/N)')
                if ans.upper() == 'N':
                    break
                execute_path_panda(end.path)
                continue
            if action.name == 'place':
                obj, obj_pose, grasp, conf, traj = action.args
                start = vobj.TrajPath(robot, traj.path[:2])
                end = vobj.TrajPath(robot, traj.path[1:])
                start.execute()

                ans = input('Execute Robot? (Y/N)')
                if ans.upper() == 'N':
                    break
                execute_path_panda(start.path)

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
                execute_path_panda(end.path)
                continue

            action.args[-1].execute()
            path = action.args[-1].path

            ans = input('Execute Robot? (Y/N)')
            if ans.upper() == 'N':
                break
            execute_path_panda(path)

    IPython.embed()
    pb_robot.utils.wait_for_user()
    pb_robot.utils.disconnect()

