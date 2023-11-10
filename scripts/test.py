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
    print('objects', objects)
    tamp = TAMP_Functions(robot, objects, floor, openable)
    open_class = Open(robot, objects, floor)

    # objects['cabinet'].set_configuration((0, 0))
#     while True:
#         # g, q = grasp.grasp('cabinetOpen', objects['cabinet'].link_from_name('bottom_drawer_knob').get_link_tform(True))
#         # robot.arm.SetJointValues(q)
#         # robot.arm.hand.Close()
#         # print(robot.arm.IsCollisionFree(q))
#         # input('next')
#         # p = tamp.samplePlacePose('fork', 'bottom_cabinet_region')[0]
#         objects['fork'].set_transform([[ 1.00000000e+00,  2.44929371e-16,  0.00000000e+00,  6.99999976e-01],
#  [-2.44929371e-16,  1.00000000e+00,  0.00000000e+00, -3.14320943e-01],
#  [ 0.00000000e+00,  0.00000000e+00,  1.00000000e+00,  3.50500000e-01],
#  [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])
        # g, q = grasp.grasp('fork', p.pose)
        # print(q)
        # if q:
        #     robot.arm.SetJointValues(q)
        # input('next')
    # obj = 'cabinet'
    # knob = 'bottom_drawer_knob'
    # pose = objects['cabinet'].link_from_name(knob).get_link_tform(True)

    # obj_conf = vobj.BodyConf(obj, objects['cabinet'].get_configuration())

    # end_conf = vobj.BodyConf(obj, (0, 0))
    # g, start_q = grasp.grasp('cabinetClose', pose)
    # print('start_q', start_q)
    # robot.arm.SetJointValues(start_q)
    # relative_grasp = vobj.Pose('cabintet', numpy.dot(numpy.linalg.inv(pose), g))
    # robot.arm.Grab(objects[obj], relative_grasp, 'M')
    # traj = tamp.get_openable_traj(obj, obj_conf, end_conf,  vobj.BodyConf(robot, start_q), relative_grasp, knob)
    # print(traj)
    # if traj:
    #     for cmd in traj[0]:
    #         cmd.execute(None)
    #         input('next')

    IPython.embed()
    pb_robot.utils.wait_for_user()
    pb_robot.utils.disconnect()

