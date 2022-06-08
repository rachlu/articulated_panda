from RRT import RRT
import numpy
from Plan import Plan
import pb_robot
import table_env
import IPython
from Grasp import Grasp
import math
import vobj
from TAMP_Functions import TAMP_Functions


if __name__ == '__main__':
    # pb_robot.utils.connect(use_gui=True)
    # pb_robot.utils.disable_real_time()
    # pb_robot.utils.set_default_camera()

    objects, floor, robot = table_env.execute()
    robot.arm.hand.Open()
    grasp = Grasp(robot, objects)
    rrt = RRT(robot)
    while True:
        q_start = robot.arm.GetJointValues()
        print('start', q_start)
        q_end = rrt.sample_config()
        path = rrt.motion(q_start, q_end)
        print(path)
        if path is None:
            continue
        input('execute?')
        p = vobj.TrajPath(robot, path)
        p.execute()
        input('next')
    # while True:
    #     q_start = robot.arm.GetJointValues()
    #     q_goal = rrt.sample_config()
    #     path = rrt.motion(q_start, q_goal)
    #     print(path)
    #     if path is None:
    #         continue
    #     p = vobj.TrajPath(robot, path)
    #     p.execute()
    #     input('next')
    # while True:
    #    q = grasp.grasp('plate')[1]
    #    robot.arm.SetJointValues(q)
    #    print(robot.arm.IsCollisionFree(q, obstacles= [objects['plate']], self_collisions = False))
    #    input('next')

    IPython.embed()
    pb_robot.utils.wait_for_user()
    pb_robot.utils.disconnect()
