from franka_interface import ArmInterface

import rospy
import IPython
import table_env


if __name__ == '__main__':
    objects, openable, floor, robot = table_env.execute()
    rospy.init_node('reset')
    arm = ArmInterface()
    arm.hand.open()
    robot.arm.hand.Open()
    ans = input('Execute?')
    if ans.upper() != 'N':
        arm.move_to_neutral()

    IPython.embed()
