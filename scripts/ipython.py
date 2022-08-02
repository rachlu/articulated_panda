from TAMP_Functions import *
from Place import Place
from RRT import *

import table_env
import IPython
import pb_robot

if __name__ == '__main__':
    objects, floor, robot = table_env.execute()
    robot.arm.hand.Open()
    grasp = Grasp(robot, objects)
    rrt = RRT(robot, constraint=rotation_constraint)
    place = Place(robot, objects, floor)

    IPython.embed()
    pb_robot.utils.wait_for_user()
    pb_robot.utils.disconnect()

