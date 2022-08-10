from scipy.spatial.transform import Rotation as R
from TAMP_Functions import *
from Open import Open

import pb_robot
import table_env
import IPython
import math
import numpy

if __name__ == '__main__':
    # pb_robot.utils.connect(use_gui=True)
    # pb_robot.utils.disable_real_time()
    # pb_robot.utils.set_default_camera()

    objects, openable, floor, robot = table_env.execute()
    robot.arm.hand.Open()
    grasp = Grasp(robot, objects)
    tamp = TAMP_Functions(robot, objects, floor, openable)
    open_class = Open(robot, objects, floor)

    # translation = numpy.array([[1, 0, 0, 0],
    #                            [0, 1, 0, 0],
    #                            [0, 0, 1, -0.13],
    #                            [0, 0, 0, 1]])
    # t1 = util.get_rotation_arr('X', 3 * math.pi / 2)
    # t2 = util.get_rotation_arr('Y', math.pi / 2)
    # rotation = numpy.dot(t1, t2)
    # rel = numpy.dot(rotation, translation)
    # world_grasp = numpy.dot(objects['cabinet'].link_from_name('bottom_drawer_knob').get_link_tform(True), rel)
    # q = robot.arm.ComputeIK(world_grasp)
    # robot.arm.SetJointValues(q)
    # for angle in numpy.linspace(0, 2*math.pi, 20):
    #     objects['door'].set_configuration((angle, ))
    #     pose = objects['door'].link_from_name('knob').get_link_tform(True)
    #     pb_robot.viz.draw_tform(pose)
    #

    # for _ in range(7):
    #     relative_grasp, q = tamp.sampleGrabPose('door', vobj.Pose('door', objects['door'].get_configuration()))[0]
    #     # if q is None or not robot.arm.IsCollisionFree(q):
    #     #     print('q None')
    #     #     continue
    #     # g, q = grasp.grasp('cabinet_top', objects['cabinet'].get_transform())
    #     robot.arm.SetJointValues(q)
    #     # robot.arm.hand.Close()
    #     # Door closed is pi/2
    #     # traj = open_class.get_door_traj(q, numpy.dot(numpy.linalg.inv(objects['door'].link_from_name('knob').get_link_tform(True)), g),
    #     #                                 increment, 3)
    #     traj = open_class.get_door_traj(q, relative_grasp.pose, increment, 3)
    #     # traj = open_class.get_cabinet_traj(q, g, 'top', increment)
    #     if traj is not None:
    #         break
    #
    # input('execute')
    # traj[0][0].execute(objects['door'], None, increment)
    obj = 'door'
    for _ in range(7):
        relative_grasp, q = tamp.sampleGrabPose(obj, vobj.Pose(obj, objects[obj].get_configuration()), 'knob')[0]
        robot.arm.SetJointValues(q)
        traj = open_class.open_obj(obj, q, relative_grasp.pose, objects[obj].get_configuration(), (math.pi/20,), 3, 'knob')
        if traj is not None:
            break
    input('execute')
    traj[0][0].execute(objects[obj], None, math.pi/20)

    IPython.embed()
    pb_robot.utils.wait_for_user()
    pb_robot.utils.disconnect()

