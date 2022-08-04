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

    # for angle in numpy.linspace(0, 2*math.pi, 20):
    #     objects['door'].set_configuration((angle, ))
    #     pose = objects['door'].link_from_name('knob').get_link_tform(True)
    #     pb_robot.viz.draw_tform(pose)
    #
    relative_grasp = tamp.sampleGrabPose('door', vobj.Pose('door', objects['door'].get_configuration()))[0][0].pose
    objects['door'].set_configuration((0,))

    knob_pose = objects['door'].link_from_name('knob').get_link_tform(True)
    r = R.from_matrix(knob_pose[:3, :3])
    offset = r.as_euler('xyz')[-1]
    print(r.as_euler('xyz', degrees=True))

    start_grasp = numpy.dot(knob_pose, relative_grasp)
    q = robot.arm.ComputeIK(start_grasp)
    robot.arm.SetJointValues(q)

    objects['door'].set_configuration((math.pi / 2,))
    knob_pose = objects['door'].link_from_name('knob').get_link_tform(True)

    right_angle = numpy.dot(knob_pose, relative_grasp)

    objects['door'].set_configuration((math.pi,))
    knob_pose = objects['door'].link_from_name('knob').get_link_tform(True)

    one_eighty = numpy.dot(knob_pose, relative_grasp)

    objects['door'].set_configuration((3*math.pi/2,))
    knob_pose = objects['door'].link_from_name('knob').get_link_tform(True)

    two_seventy = numpy.dot(knob_pose, relative_grasp)
    objects['door'].set_configuration((0, ))

    if offset % math.pi == 0:
        print('modulo')
        b = (two_seventy[1][-1] - right_angle[1][-1]) / 2
        y_0 = two_seventy[1][-1] - b

        a = (one_eighty[0][-1] - start_grasp[0][-1]) / 2
        x_0 = one_eighty[0][-1] - a
        t = (offset + math.pi) % (2 * math.pi)
    else:
        a = (two_seventy[0][-1] - right_angle[0][-1]) / 2
        x_0 = two_seventy[0][-1] - a

        b = (start_grasp[1][-1] - one_eighty[1][-1]) / 2
        y_0 = start_grasp[1][-1] - b
        t = math.pi/2

    angle = math.atan((start_grasp[1][-1] - y_0) / (start_grasp[0][-1] - x_0)) + offset
    sample = 10

    increment = math.pi/18
    initial = t
    # for _ in range(sample):
    #     print('t', t)
    #     t += increment
    #     new_grasp = numpy.array(start_grasp)
    #     x = a * math.cos(t + angle) + x_0
    #     y = b * math.sin(t + angle) + y_0
    #     print('x, y', x, y)
    #     new_grasp[0][-1] = x
    #     new_grasp[1][-1] = y
    #     new_grasp = numpy.dot(new_grasp, util.get_rotation_arr('Z', -(t - initial)))
    #     pb_robot.viz.draw_tform(new_grasp)
    #     input('next')

    for _ in range(7):
        relative_grasp, q = tamp.sampleGrabPose('door', vobj.Pose('door', objects['door'].get_configuration()))[0]
        # if q is None or not robot.arm.IsCollisionFree(q):
        #     print('q None')
        #     continue
        # g, q = grasp.grasp('cabinet_top', objects['cabinet'].get_transform())
        robot.arm.SetJointValues(q)
        # robot.arm.hand.Close()
        # Door closed is pi/2
        # traj = open_class.get_door_traj(q, numpy.dot(numpy.linalg.inv(objects['door'].link_from_name('knob').get_link_tform(True)), g),
        #                                 increment, 3)
        traj = open_class.get_door_traj(q, relative_grasp.pose, increment, 3)
        # traj = open_class.get_cabinet_traj(q, g, 'top', increment)
        if traj is not None:
            break

    input('execute')
    traj[0][0].execute(objects['door'], None, increment)

    IPython.embed()
    pb_robot.utils.wait_for_user()
    pb_robot.utils.disconnect()

