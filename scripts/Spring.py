from franka_interface import ArmInterface
from TAMP_Functions import TAMP_Functions

from franka_tools import CollisionBehaviourInterface

import rospy
import IPython
import table_env
import util
import numpy
import vobj
import quaternion


def stiff_matrix(stiffness):
    return numpy.array([stiffness, stiffness, stiffness, stiffness/10, stiffness/10, stiffness/10])


def stiffness_and_offset(force):
    """
    Given a force (Newtons) to be applied return the difference in Z and
    corresponding 6x1 stiffness matrix.

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

    if force < -20 or force > 0:
        print('Invalid Force')
        return None, None

    stiffness_funcs = {400: lambda x: (x + 2.39) / 426, 200: lambda x: (x + 1.87) / 231,
                       100: lambda x: (x + 1.23) / 124, 50: lambda x: (x + 0.99) / 79.3, 0: lambda x: x / 50.7}

    for stiffness in stiffness_funcs:
        z_offset = stiffness_funcs[stiffness](force)
        if z_offset <= 0:
            return z_offset, stiff_matrix(stiffness)

    return None, None


def stiffness_and_force(offset):
    """
    Given a Z offset (meters) return force (Newtons) and
    corresponding 6x1 stiffness matrix.

    Maximizes stiffness with the condition that Z offset is less than 0.

    Stiffness Equations (Source: Rachel Holladay)
    Stiffness 400:  f = 426d - 2.39
    Stiffness 200:  f = 231d - 1.87
    Stiffness 100:  f = 124d - 1.23
    Stiffness 50:   f = 79.3d - 0.99
    Stiffness 0:    f = 50.7d - 0

    :param offset: offset in meters
    :return: (force , stiffness)
    """

    stiffness_funcs = {400: lambda x: (x * 426) - 2.39, 200: lambda x: (x * 231) - 1.87,
                       100: lambda x: (x * 124) - 1.23, 50: lambda x: (x * 79.3) - 0.99, 0: lambda x: x * 50.7}

    for stiffness in stiffness_funcs:
        force = stiffness_funcs[stiffness](offset)
        if -20 <= force <= 0:
            return force, stiff_matrix(stiffness)

    return None, None


def get_cartesian(pose):
    rotation = quaternion.from_rotation_matrix(pose[:3, :3])
    transform = pose[:3, 3]
    return {'position': transform, 'orientation': rotation}


def get_matrix(cart):
    matrix = numpy.zeros((4, 4))
    orientation = quaternion.as_rotation_matrix(cart['orientation'])
    transform = cart['position']
    matrix[:3, :3] = orientation
    matrix[:3, 3] = transform
    matrix[3, :] = [0, 0, 0, 1]
    return matrix


class Spring:
    def __init__(self, robot, arm):
        self.arm = arm
        self.robot = robot

    def valid_force(self, q, force):
        if force < -20 or force > 0:
            return False

        return self.robot.arm.InsideTorqueLimits(q, [0, 0, force, 0, 0, 0])

    # Tested Using Simulation Only!
    def apply_force(self, force):
        """
        :param force: Force in Newtons
        """
        distance, matrix = stiffness_and_offset(force)
        print(force, distance, matrix)
        if distance is None:
            print('No distance and stiffness matrix')
            return
        cart, new_q = self.qp_from_distance(distance)
        print(cart, new_q)
        if new_q:
            self.robot.arm.SetJointValues(new_q)
        else:
            print('Q None')
            return None

        ans = input('Exert Force')
        if ans.upper() == 'N':
            return

        if self.valid_force(new_q, force):
            self.arm.set_cart_impedance_pose(cart, matrix)
        else:
            print('Torque Limit!')
        return new_q

    # Tested!
    def qp_from_distance(self, distance):
        # end_effector = self.robot.arm.GetEETransform()
        # current_q = self.robot.arm.GetJointValues()
        # end_effector[2][-1] += distance
        # new_q = self.robot.arm.ComputeIKQ(end_effector, current_q)
        # return get_cartesian(end_effector), new_q
        if distance > 0:
            print('Nonnegative Distance')
            return None, None
        end_effector = self.arm.endpoint_pose()
        current_q = self.arm.convertToList(self.arm.joint_angles())
        end_effector = get_matrix(end_effector)
        end_effector[2][-1] += distance
        new_q = self.robot.arm.ComputeIKQ(end_effector, current_q)
        return get_cartesian(end_effector), new_q

    def move_to_distance_offset(self, distance, stiffness, error=0.01):
        cart, q = numpy.array(self.qp_from_distance(distance))
        goal = cart['position'][-1]
        print('goal', goal)
        offset = -0.01
        diff = 9999
        while diff > error:
            # force, matrix = stiffness_and_force(offset)
            # if force is None:
            #     print('No force and stiffness matrix')
            #     return
            matrix = stiff_matrix(stiffness)
            print('diff', diff)
            # print(force, matrix)
            ans = input('Apply force?')
            if ans.upper() == 'N':
                break
            current_pose = self.arm.endpoint_pose()
            current = current_pose['position'][-1]
            execute_pose = {'position': numpy.array(current_pose['position']),
                            'orientation': current_pose['orientation']}
            print('before')
            print(execute_pose)
            execute_pose['position'][-1] += offset
            print('parameters')
            print(execute_pose, matrix)
            self.arm.set_cart_impedance_pose(execute_pose, matrix)
            print('current', current)
            diff = current - goal
            print('diff', diff)
        print('Position Reached!')

    def move_to_distance_force(self, distance, error=0.01):
        cart, q = numpy.array(self.qp_from_distance(distance))
        goal = cart['position'][-1]
        print('goal', goal)
        offset = distance
        diff = 9999
        force, matrix = stiffness_and_force(offset)
        if force is None:
            print('No force and stiffness matrix')
            return
        while diff > error:
            print('diff', diff)
            ans = input('Apply force?')
            if ans.upper() == 'N':
                break
            self.apply_force(force)
            current_pose = self.arm.endpoint_pose()
            current = current_pose['position'][-1]
            print('current', current)
            diff = current - goal
            print('diff', diff)
            force -= 1
        print('Position Reached!')


if __name__ == '__main__':
    rospy.init_node('Spring')
    arm = ArmInterface()
    objects, openable, floor, robot = table_env.execute()
    spring = Spring(robot, arm)
    # spring = Spring(robot, None)

    start_q = arm.convertToList(arm.joint_angles())
    robot.arm.SetJointValues(start_q)
    robot.arm.hand.Open()
    arm.hand.open()

    tamp = TAMP_Functions(robot, objects, floor, openable)
    start_conf = vobj.BodyConf(robot, robot.arm.GetJointValues())
    pose = vobj.Pose('spring', objects['spring'].get_transform())
    relative_grasp = tamp.sampleGrabPose('spring', pose)[0][0]
    q, hand_traj = tamp.computeIK('spring', pose, relative_grasp)[0]
    hand_traj = hand_traj[:2]
    traj = tamp.calculate_path(start_conf, q)[0][0][0]
    traj.execute()
    # input('execute')
    # for q in traj.path:
    #     robot.arm.SetJointValues(q)
    #     pose = robot.arm.GetEETransform()
    #     pose = get_cartesian(pose)
    #     arm.set_cart_impedance_pose(pose, [200, 200, 200, 20, 20, 20])
    #     input('next')
    traj = util.convert(arm, traj.path)
    #
    input('execute')
    arm.execute_position_path(traj)
    hand_traj[1].execute()
    arm.hand.close()
    
    collision = CollisionBehaviourInterface()
    collision.set_collision_threshold(cartesian_forces=[5, 5, 5, 25, 25, 25])

    ans = input('move_to_touch')
    if ans.upper() != 'N':
        hand_traj[0].execute()
        arm.move_to_touch(arm.convertToDict(hand_traj[0].path[1]))

    IPython.embed()
