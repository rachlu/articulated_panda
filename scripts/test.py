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

    objects, openable, floor, robot = table_env.execute()
    robot.arm.hand.Open()
    grasp = Grasp(robot, objects)
    tamp = TAMP_Functions(robot, objects, floor, openable)
    open_class = Open(robot, objects, floor)

    for angle in numpy.linspace(0, 2*math.pi, 40):
        objects['door'].set_configuration((angle, ))
        pose = objects['door'].link_from_name('knob').get_link_tform(True)
        pb_robot.viz.draw_tform(pose)

    relative_grasp = tamp.sampleGrabPose('door', vobj.Pose('door', objects['door'].get_configuration()))[0][0].pose

    objects['door'].set_configuration((0,))
    start_grasp = objects['door'].link_from_name('knob').get_link_tform(True)
    # start_grasp = numpy.dot(knob_pose, relative_grasp)

    objects['door'].set_configuration((math.pi / 2,))
    right_angle = objects['door'].link_from_name('knob').get_link_tform(True)

    # right_angle = numpy.dot(knob_pose, relative_grasp)

    x_0 = start_grasp[0][-1]
    y_0 = right_angle[1][-1]
    a = x_0 - right_angle[0][-1]
    b = start_grasp[1][-1] - y_0

    for t in numpy.linspace(0, 2 * math.pi, 40):
        new_grasp = numpy.array(start_grasp)
        x = a * math.cos(t) + x_0
        y = b * math.sin(t) + y_0
        new_grasp[0][-1] = x
        new_grasp[1][-1] = y
        new_grasp = numpy.dot(new_grasp, util.get_rotation_arr('Z', -(t - math.pi / 2)))
        pb_robot.viz.draw_tform(new_grasp)

    # rotate = util.get_rotation_arr('Z', math.pi/8)
    # g = robot.arm.GetEETransform()
    # q = robot.arm.ComputeIK(numpy.dot(g, rotate))
    # objects['door'].set_configuration((math.pi/2, ))
    # increment = -.04
    # g, q = grasp.grasp('cabinet_top', objects['cabinet'].get_transform())
    # robot.arm.SetJointValues(q)
    # robot.arm.hand.Close()
    # for link in objects['cabinet'].links:
    #     # if pb_robot.collisions.body_collision(link.body, robot):
    #     #     print(link.get_link_name())
    #     if pb_robot.collisions.pairwise_link_collision(objects['cabinet'], link, robot):
    #         print(link.get_link_name())
    # open_class.get_door_traj(q, numpy.dot(numpy.linalg.inv(objects['door'].get_transform()), g))
    # for _ in range(7):
    #     g, q = grasp.grasp('door',  objects['door'].link_from_name('knob').get_link_tform(True))
    #     if q is None:
    #         continue
    #     # g, q = grasp.grasp('cabinet_top', objects['cabinet'].get_transform())
    #     robot.arm.SetJointValues(q)
    #     # robot.arm.hand.Close()
    #     # Door closed is pi/2
    #     traj = open_class.get_door_traj(q, numpy.dot(numpy.linalg.inv(objects['door'].link_from_name('knob').get_link_tform(True)), g),
    #                                     increment, 3)
    #     # traj = open_class.get_cabinet_traj(q, g, 'top', increment)
    #     if traj is not None:
    #         break
    # # sample = math.ceil((math.pi - math.pi/2) / increment)
    # for t in numpy.linspace(math.pi / 2, math.pi, 11):
    #     print(t)
    #     objects['door'].set_configuration((t - math.pi / 2,))
    #     pose = objects['door'].link_from_name('knob').get_link_tform(True)
    #     print(pose)
    # g, q = grasp.grasp('cabinet_bottom',  objects['cabinet'].get_transform())
    # robot.arm.SetJointValues(q)
    # robot.arm.hand.Close()
    # print(util.collision_link_body(objects['cabinet'], objects['cabinet'].link_from_name('top_drawer_knob'), robot))
    # for t in numpy.linspace(math.pi / 2, 3*math.pi/4, 10):
    #     new_grasp = numpy.array(g)
    #     x = a * math.cos(t) + x_0
    #     y = b * math.sin(t) + y_0
    #
    #     new_grasp[0][-1] = x
    #     new_grasp[1][-1] = y
    #     new_grasp = numpy.dot(new_grasp, util.get_rotation_arr('Z', -(t-math.pi/2)))
    #     pb_robot.viz.draw_tform(new_grasp)

    # # print('traj', traj)
    # # # x = 0
    # input('execute')
    # # for t in traj[0]:
    # #     if isinstance(t, vobj.TrajPath):
    # #         t.execute()
    # traj = vobj.TrajPath(robot, traj.path)
    # traj[0][0].execute(objects['door'], None, increment)
    # r = R.from_matrix([[0, -1, 0], [-1, 0, 0], [0, 0, -1]])
    # print(r.as_euler('XYZ', degrees=True))
    IPython.embed()
    pb_robot.utils.wait_for_user()
    pb_robot.utils.disconnect()

