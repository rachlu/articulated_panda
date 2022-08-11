# from franka_interface import ArmInterface
from TAMP_Functions import TAMP_Functions

# import rospy
import IPython
import table_env
import util
import numpy
import vobj


def stiff_matrix(stiffness):
    return numpy.array([400, 400, stiffness, 40, 40, 40])


def stiffness_and_force(force):
    """
    Given a force (Newtons) to be applied return the difference in Z and
    corresponding stiffness 6x1 matrix.

    Maximizes stiffness with the condition that Z offset is less than 0.

    Stiffness Equations (Source: Rachel Holladay)
    Stiffness 400:  f = 426d - 2.39
    Stiffness 200:  f = 231d - 1.87
    Stiffness 100:  f = 124d - 1.23
    Stiffness 50:   f = 79.3d - 0.99
    Stiffness 0:    f = 50.7d - 0

    :param force: Force in Newtons
    :return: (Z offset, stiffness)
    """

    stiffness_funcs = {400: lambda x: (x + 2.39) / 426, 200: lambda x: (x + 1.87) / 231, 100: lambda x: (x + 1.23) / 124,
                 50: lambda x: (x + 0.99) / 79.3, 0: lambda x: x / 50.7}

    for stiffness in stiffness_funcs:
        z_offset = stiffness_funcs[stiffness](force)
        if z_offset <= 0:
            return z_offset, stiff_matrix(stiffness)

    return (None, None)


class Spring:
    def __init__(self, robot, arm):
        self.arm = arm
        self.robot = robot
        self.stiffness = [1, 1, 1, 1, 1, 1, 1]

    # Tested!
    def get_force(self, distance):
        return -1 * distance * 10
        # return -1 * numpy.linalg.norm(numpy.dot(self.stiffness, distance))

    # Tested!
    def get_distance_from_force(self, force):
        """
        :param force: Force in Newtons
        :return: Linear distance to apply specified force in meters
        """
        return -1 * force / 10
        # return -1 * numpy.linalg.norm(numpy.dot(force, numpy.linalg.inv(self.stiffness)))

    # Tested Using Simulation Only!
    def apply_force(self, force):
        """
        :param force: Force in Newtons
        """
        distance = self.get_distance_from_force(force)
        new_q = self.q_from_distance(distance)
        print(new_q)
        if new_q:
            self.robot.arm.SetJointValues(new_q)
        else:
            print('Q None')
            return None
        # input('Exert Force')
        # if self.robot.arm.InsideTorqueLimits(new_q, [0, 0, force, 0, 0, 0]):
        #     self.arm.set_joint_impedance_config(new_q)
        # else:
        #     print('Torque Limit!')
        return new_q

    # Tested!
    def q_from_distance(self, distance):
        end_effector = self.robot.arm.GetEETransform()
        current_q = self.robot.arm.GetJointValues()
        end_effector[2][-1] += distance
        new_q = self.robot.arm.ComputeIKQ(end_effector, current_q)
        return new_q

    def move_to_distance_force(self, distance, error=0.01):
        q = numpy.array(self.q_from_distance(distance))
        self.robot.arm.SetJointValues(q)
        goal = numpy.array(self.robot.arm.GetEETransform())
        # goal = goal[:, -1]
        diff = 999999
        force = self.get_force(distance)
        while diff > error:
            print('diff', diff)
            current_q = self.apply_force(force)
            # current_q = self.arm.convertToList(self.arm.joint_angles())
            self.robot.arm.SetJointValues(current_q)
            current = numpy.array(self.robot.arm.GetEETransform())
            # current = current[:, -1]
            # diff = util.getDistance(goal, current)
            diff = current[2][-1] - goal[2][-1]
            force += 0.001
        print('Position Reached!')


if __name__ == '__main__':
    # rospy.init_node('Spring')
    # arm = ArmInterface()
    objects, openable, floor, robot = table_env.execute()
    # spring = Spring(robot, arm)
    spring = Spring(robot, None)

    # start_q = arm.convertToList(arm.joint_angles())
    # robot.arm.SetJointValues(start_q)
    robot.arm.hand.Open()
    # arm.hand.open()

    tamp = TAMP_Functions(robot, objects, floor, openable)
    start_conf = vobj.BodyConf(robot, robot.arm.GetJointValues())
    pose = vobj.Pose('spring', objects['spring'].get_transform())
    relative_grasp = tamp.sampleGrabPose('spring', pose)[0][0]
    q, hand_traj = tamp.computeIK('spring', pose, relative_grasp)[0]
    hand_traj = hand_traj[:2]
    traj = tamp.calculate_path(start_conf, q)[0][0][0]
    traj.execute()
    # traj = util.convert(arm, traj.path)

    # input('execute')
    # arm.execute_position_path(traj)
    hand_traj[1].execute()
    # arm.hand.close()
    hand_traj[0].execute()

    # input('move_to_touch')
    # arm.move_to_touch(arm.convertToDict(hand_traj[0].path[1]))

    IPython.embed()
