from RRT import *
import numpy
from Plan import Plan
import pb_robot
import table_env
import IPython
from Grasp import Grasp
import math
import vobj
from TAMP_Functions import *
from Place import Place
from Open import Open
from scipy.spatial.transform import Rotation as R


if __name__ == '__main__':
    # pb_robot.utils.connect(use_gui=True)
    # pb_robot.utils.disable_real_time()
    # pb_robot.utils.set_default_camera()

    objects, floor, robot = table_env.execute()
    robot.arm.hand.Open()
    grasp = Grasp(robot, objects)
    # translation = numpy.array([[1, 0, 0, -0.2],
    #                            [0,  1,  0, 0.19],
    #                            [0,  0, 1, 0.58],
    #                            [ 0,  0,  0, 1]])
    # t1 = util.get_rotation_arr('X', math.pi)
    # t2 = util.get_rotation_arr('Z', -math.pi/2)
    # rotation = numpy.dot(t1, t2)
    # rel = numpy.dot(rotation, translation)
    # world_grasp = numpy.dot(objects['door'].get_transform(), rel)
    # # # print(world_grasp)
    # q = robot.arm.ComputeIK(world_grasp)
    # robot.arm.SetJointValues(q)

    # while True:
    #     g, q = grasp.grasp('cabinet_top', objects['cabinet'].get_transform())
    #     robot.arm.SetJointValues(q)
    #     print(robot.arm.IsCollisionFree(q))
    #     input('next')

    open_class = Open(robot, objects, floor)
    g, q = grasp.grasp('cabinet_bottom', objects['cabinet'].get_transform())
    robot.arm.hand.Close()
    robot.arm.SetJointValues(q)
    increment = 0.05
    traj = open_class.get_trajectory(q, g, -increment)
    x = 0
    input('execute')
    for q in traj:
        robot.arm.SetJointValues(q)
        # (Top, Bottom)
        objects['cabinet'].set_configuration((0, x))
        x += increment
        time.sleep(0.5)
    IPython.embed()
    pb_robot.utils.wait_for_user()
    pb_robot.utils.disconnect()
