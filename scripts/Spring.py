import rospy
from franka_interface import ArmInterface
import IPython
import pb_robot
import table_env
import util


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
        return -1 * force / self.K

    def apply_force(self, force):
        distance = self.get_distance_from_force(force)
        end_effector = self.robot.arm.GetEETransform()
        current_q = self.robot.arm.GetJointValues()
        end_effector[2][-1] += distance
        new_q = self.robot.arm.ComputeIKQ(end_effector, current_q)

        # new_q = self.arm.convertToDict(new_q)
        self.arm.set_joint_impedance_config(new_q)

    def set_k(self, k):
        self.K = k

    def get_k(self):
        return self.K


if __name__ == '__main__':
    rospy.init_node('Spring')
    arm = ArmInterface()
    objects, floor, robot = table_env.execute()
    spring = Spring(robot, arm)
