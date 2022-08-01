import rospy
from franka_interface import ArmInterface
import IPython
from TAMP_Functions import TAMP_Functions
import table_env
import vobj
from Spring import Spring

def convert(path):
    """
    :param path: list of robot configurations
    :return: list of dictionaries of robot configurations
    """
    final_path = []
    for num in range(len(path)):
        q = path[num]
        print(q)
        print(num+1, '/', len(path), '...')
        final_path.append(arm.convertToDict(q))
    return final_path


if __name__ == '__main__':
    objects, openable, floor, robot = table_env.execute()
    rospy.init_node('reset')
    arm = ArmInterface()
    arm.hand.open()
    robot.arm.hand.Open()
    ans = input('Execute?')
    if ans.upper() != 'N':
        arm.move_to_neutral()
    
    start_q = arm.convertToList(arm.joint_angles())
    robot.arm.SetJointValues(start_q)
    
    spring = Spring(robot, arm)
    tamp = TAMP_Functions(robot, objects, floor, openable)
    start_conf = vobj.BodyConf(robot, robot.arm.GetJointValues())
    pose = vobj.Pose('spring', objects['spring'].get_transform())
    relative_grasp = tamp.sampleGrabPose('spring', pose)[0][0]
    q, hand_traj = tamp.computeIK('spring', pose, relative_grasp)[0]
    hand_traj = hand_traj[:2]
    traj = tamp.calculate_path(start_conf, q)[0][0][0]
    traj.execute()
    traj = convert(traj.path)
    input('execute')
    arm.execute_position_path(traj)
    hand_traj[1].execute()
    arm.hand.close()
    hand_traj[0].execute()
    input('move_to_touch')
    arm.move_to_touch(arm.convertToDict(hand_traj[0].path[1]))

    IPython.embed()
