import time
import util
import numpy as np
from pddl_setup import *

def get_euclDist(p1, p2):
    return np.sqrt((p1[1] - p2[1])**2)

class BodyConf:
    def __init__(self, body, q):
        self.body = body
        self.conf = q

    def __repr__(self):
        return 'q{}'.format(id(self) % 1000)


class TrajPath:
    def __init__(self, robot, path, impedance=False):
        self.robot = robot
        self.path = path
        self.impedance = impedance

    def __repr__(self):
        return 't{}'.format(id(self) % 1000)

    def execute(self, arm, imped=False):
        ans = 'Y'
        while ans.upper() == 'Y':
            for q in self.path:
                self.robot.arm.SetJointValues(q)
                time.sleep(0.2)
            ans = input('Redo?')
        if arm:
            # path = util.convert(arm, self.path)
            if not self.impedance: 
                arm.execute_position_path(util.convert(arm, self.path)) 
            else: 
                i = 0
                stiffness = [200, 200, 200, 20, 20, 20]
                while i < len(self.path):
                    q = self.path[i]
                    # if (not self.robot.arm.InsideTorqueLimits(q, stiffness)):
                    #     raise AssertionError("Over Torque Limit, need to replan")
                    cart_pose = self.robot.arm.ComputeFK(q)
                    new_pose = arm.endpoint_pose()
                    new_pose['position'][0] = cart_pose[0][-1]
                    new_pose['position'][1] = cart_pose[1][-1]
                    new_pose['position'][2] = cart_pose[2][-1]
                    arm.set_cart_impedance_pose(new_pose, stiffness)
                    curr_pose = arm.endpoint_pose()

                    dist = get_euclDist(new_pose['position'], curr_pose['position'])
                    if (dist > 0.01):
                        print('Did not reach desired position\n')
                        print('Current Pose: {0}\nExpected Pose: {1}\n'.format(curr_pose['position'], new_pose['position']))
                        stiffness[0] += 10
                        print('Raising Stiffness to {0}\n', stiffness)
                        # raise AssertionError("Did not reach desired position")
                        if (stiffness[0] > 300):
                            break
                        continue
                    i += 1

                # success = arm.execute_joint_impedance_traj_recover(self.path)
                
                # if not success:
                #     current_q = arm.joint_angles()
                #     self.robot.arm.SetJointValues(current_q)
                #     current_pose = self.robot.arm.GetEETransform()
                #     back = numpy.array([[1, 0, 0, 0],
                #             [0, 1, 0, 0],
                #             [0, 0, 1, -.07],
                #             [0., 0., 0., 1.]])
                #     back_grasp = numpy.dot(current_pose, back)
                #     recover_q = self.robot.arm.ComputeIKQ(back_grasp, current_q)
                #     arm.execute_position_path(util.convert(arm, [current_q, recover_q]))
            
                    # TODO: Add replan
                    # Pddl.replan()


class HandCmd:
    def __init__(self, robot, obj, grasp=None, status=None):
        self.robot = robot
        self.obj = obj
        self.grasp = grasp
        self.status = status

    def execute(self, arm):
        if len(self.robot.arm.grabbedObjects) != 0:
            self.robot.arm.hand.Open()
            self.robot.arm.Release(self.obj)
            if arm:
                input('Open')
                arm.hand.open()
        else:
            print('grabbed')
            self.robot.arm.hand.Close()
            self.robot.arm.Grab(self.obj, self.grasp, self.status)
            if arm:
                input('Close')
                arm.hand.grasp(0.02, 40, epsilon_inner=0.1, epsilon_outer=0.1)

    def set_status(self, status):
        self.status = status

    def __repr__(self):
        return 't{}'.format(id(self) % 1000)


class Pose:
    def __init__(self, obj, pose):
        self.obj = obj
        self.pose = pose

    def __repr__(self):
        return 'p{}'.format(id(self) % 1000)
