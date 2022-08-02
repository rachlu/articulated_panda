from TAMP_Functions import *
from Open import Open

import pb_robot
import table_env
import IPython


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
    #
    #
    # relative_grasp = tamp.sampleGrabPose('door', vobj.Pose('door', objects['door'].get_configuration()))[0][0].pose
    # objects['door'].set_configuration((0,))
    # knob_pose = objects['door'].link_from_name('knob').get_link_tform(True)
    # start_grasp = numpy.dot(knob_pose, relative_grasp)
    # # print('start_grasp')
    # # pb_robot.viz.draw_tform(start_grasp)
    # # input('next')
    #
    # objects['door'].set_configuration((math.pi / 2,))
    # knob_pose = objects['door'].link_from_name('knob').get_link_tform(True)
    #
    # right_angle = numpy.dot(knob_pose, relative_grasp)
    # # print('right_angle')
    # # pb_robot.viz.draw_tform(right_angle)
    # # input('next')
    #
    # objects['door'].set_configuration((math.pi,))
    # knob_pose = objects['door'].link_from_name('knob').get_link_tform(True)
    #
    # one_eighty = numpy.dot(knob_pose, relative_grasp)
    # # print('one_eighty')
    # # pb_robot.viz.draw_tform(one_eighty)
    # # input('next')
    #
    # objects['door'].set_configuration((3*math.pi/2,))
    # knob_pose = objects['door'].link_from_name('knob').get_link_tform(True)
    #
    # two_seventy = numpy.dot(knob_pose, relative_grasp)
    # # print('two_seventy')
    # # pb_robot.viz.draw_tform(two_seventy)
    # # input('next')
    #
    # a = (two_seventy[0][-1] - right_angle[0][-1])/2
    # b = (start_grasp[1][-1] - one_eighty[1][-1])/2
    # x_0 = two_seventy[0][-1] - a
    # y_0 = start_grasp[1][-1] - b
    # print('x_0', x_0)
    # print('y_0', y_0)
    # print('a', a)
    # print('b', b)
    # angle = math.atan((start_grasp[1][-1] - y_0)/(start_grasp[0][-1] - x_0))
    # print(angle)
    #
    # for t in numpy.linspace(0, 2 * math.pi, 20):
    #     new_grasp = numpy.array(start_grasp)
    #     x = a * math.cos(t) + x_0
    #     y = b * math.sin(t) + y_0
    #     new_grasp[0][-1] = x
    #     new_grasp[1][-1] = y
    #     new_grasp = numpy.dot(new_grasp, util.get_rotation_arr('Z', -(t - math.pi / 2)))
    #     pb_robot.viz.draw_tform(new_grasp)
    #     input('next')

    increment = math.pi/18
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

    # input('execute')
    # traj[0][0].execute(objects['door'], None, increment)

    IPython.embed()
    pb_robot.utils.wait_for_user()
    pb_robot.utils.disconnect()

