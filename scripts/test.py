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
    # translation = numpy.array([[1, 0, 0, 0.8],
    #                            [0,  1,  0, 0.0931],
    #                            [0,  0, 1, 0.588],
    #                            [ 0,  0,  0, 1]])
    # t1 = util.get_rotation_arr('X', math.pi)
    # t2 = util.get_rotation_arr('Z', math.pi/2)
    # rotation = numpy.dot(t1, t2)
    # rel = numpy.dot(translation, rotation)
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
    increment = math.pi / 20
    # g, q = grasp.grasp('door', objects['door'].get_transform())
    # open_class.get_door_traj(q, numpy.dot(numpy.linalg.inv(objects['door'].get_transform()), g))
    for _ in range(4):
        g, q = grasp.grasp('door', objects['door'].get_transform())
        robot.arm.hand.Close()
        robot.arm.SetJointValues(q)
        traj = open_class.get_door_traj(q, numpy.dot(numpy.linalg.inv(objects['door'].get_transform()), g), 3*math.pi/4,
                                        increment)[0][0]
        if traj is not None:
            break
    #
    # # print('traj', traj)
    # # # x = 0
    input('execute')
    # # for t in traj[0]:
    # #     if isinstance(t, vobj.TrajPath):
    # #         t.execute()
    # traj = vobj.TrajPath(robot, traj.path)
    traj.execute(objects['door'], None, increment)
    # r = R.from_matrix([[0, -1, 0], [-1, 0, 0], [0, 0, -1]])
    # print(r.as_euler('XYZ', degrees=True))
    IPython.embed()
    pb_robot.utils.wait_for_user()
    pb_robot.utils.disconnect()
