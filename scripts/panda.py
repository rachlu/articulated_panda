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
    for q in path:
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
    solution = solve_focused(pddlstream_problem)
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
                robot.arm.Grab(objects[action.args[0]], action.args[-1].pose)
                robot.arm.hand.Close()
                input('Execute Robot?')
                arm.hand.close()
                move_up = numpy.array(action.args[1].pose)
                move_up[2][-1] += 0.1
                move_up = vobj.Pose(action.args[0], move_up)
                new_q = tamp.computeIK(action.args[0], move_up, action.args[-1])[0][0]
                path = tamp.calculate_path(action.args[2], new_q)[0][0]
                path.execute()
                input('Execute Robot?')
                execute_path_panda(path.path)
                continue
            if action.name == 'place':
                robot.arm.Release(objects[action.args[0]])
                robot.arm.hand.Open()
                input('Execute Robot?')
                arm.hand.open()
                continue

            action.args[-1].execute()
            path = action.args[-1].path
            input('Execute Robot?')
            execute_path_panda(path)

    # grasp = Grasp(robot, objects)
    #
    # arm.hand.open()
    # motion = RRT(robot)
    # new_path = None
    # q_start = arm.convertToList(arm.joint_angles())
    # while new_path is None:
    #     grasp_pose, q_grasp = grasp.grasp('plate')
    #     relative_grasp = numpy.dot(numpy.linalg.inv(objects['plate'].get_transform()), grasp_pose)
    #     grasp_in_world = numpy.dot(objects['plate'].get_transform(), relative_grasp)
    #     conf = robot.arm.ComputeIK(grasp_in_world)
    #     new_path = motion.motion(q_start, conf)
    #
    # final_path = []
    # for q in new_path:
    #     final_path.append(arm.convertToDict(q))
    # print(final_path)
    # arm.execute_position_path(final_path)

    IPython.embed()
    pb_robot.utils.wait_for_user()
    pb_robot.utils.disconnect()

