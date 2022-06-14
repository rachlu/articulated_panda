import rospy
from franka_interface import ArmInterface
import IPython

if __name__ == '__main__':
    rospy.init_node('reset')
    arm = ArmInterface()
    arm.hand.open()
    ans = input('Execute?')
    if ans.upper() != 'N':
        arm.move_to_neutral()
    IPython.embed()
