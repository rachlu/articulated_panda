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

    IPython.embed()
    pb_robot.utils.wait_for_user()
    pb_robot.utils.disconnect()

