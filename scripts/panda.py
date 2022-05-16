import rospy
from franka_interface import ArmInterface

if __name__ == '__main__':
    rospy.init_node('testing_node')
    arm = ArmInterface()
    q_start = arm.convertToList(arm.joint_angles())
    q_end = list(q_start)
    q_end[1] += 0.2
    q_end[0] += 0.4
    q_end[2] -= 0.2
    q_end = arm.convertToDict(q_end)
    q_start = arm.convertToDict(q_start)
    path = [q_start, q_end]
    arm.execute_position_path(path)
