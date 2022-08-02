from franka_interface import ArmInterface
from TAMP_Functions import TAMP_Functions

import rospy
import IPython
import table_env
import util
import numpy
import vobj

class Spring:
    def __init__(self, robot, arm, k_constant=None, stiffness=None):
        self.K = k_constant
        self.arm = arm
        self.robot = robot
        self.stiffness = stiffness

    def set_stiffness_conf(self, stiffness):
        self.stiffness = stiffness

    def set_stiffness_conf_quad(self, transform, rotational):
        self.stiffness = [*transform, *rotational]

    def get_force(self, distance):
        return -1 * self.K * distance

    def get_distance_from_force(self, force):
        """
        :param force: Force in Newtons
        :return: Linear distance to apply specified force in meters
        """
        return -1 * force / self.K

    def apply_force(self, force):
        """
        :param force: Force in Newtons
        """
        distance = self.get_distance_from_force(force)
        end_effector = self.robot.arm.GetEETransform()
        current_q = self.robot.arm.GetJointValues()
        end_effector[2][-1] += distance
        new_q = self.robot.arm.ComputeIKQ(end_effector, current_q)
        print(new_q)
        if new_q:
            self.robot.arm.SetJointValues(new_q)
        input('Exert Force')
        if self.robot.arm.InsideTorqueLimits(new_q, [0, 0, force, 0, 0, 0]):
            self.arm.set_joint_impedance_config(new_q)
        else:
            print('Torque Limit!')
    
    def move_to_position_force(self, q, error=0.001):
        q = numpy.array(q)
        diff = 999999
        force = 1
        while diff > error:
            self.apply_force(force)
            current = numpy.array(self.robot.arm.GetEETransform())
            diff = util.getDistance(q, current)
            force += 0.3

    def set_k(self, k):
        self.K = k

    def get_k(self):
        return self.K


if __name__ == '__main__':
    rospy.init_node('Spring')
    arm = ArmInterface()
    objects, openable, floor, robot = table_env.execute()
    spring = Spring(robot, arm)

    start_q = arm.convertToList(arm.joint_angles())
    robot.arm.SetJointValues(start_q)
    robot.arm.hand.Open()
    arm.hand.open()

    spring = Spring(robot, arm)
    tamp = TAMP_Functions(robot, objects, floor, openable)
    start_conf = vobj.BodyConf(robot, robot.arm.GetJointValues())
    pose = vobj.Pose('spring', objects['spring'].get_transform())
    relative_grasp = tamp.sampleGrabPose('spring', pose)[0][0]
    q, hand_traj = tamp.computeIK('spring', pose, relative_grasp)[0]
    hand_traj = hand_traj[:2]
    traj = tamp.calculate_path(start_conf, q)[0][0][0]
    traj.execute()
    traj = util.convert(arm, traj.path)

    input('execute')
    arm.execute_position_path(traj)
    hand_traj[1].execute()
    arm.hand.close()
    hand_traj[0].execute()

    input('move_to_touch')
    arm.move_to_touch(arm.convertToDict(hand_traj[0].path[1]))

    IPython.embed()


