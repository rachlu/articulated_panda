import time
import util
import numpy as np
from pddl_setup import *

def get_euclDist(p1, p2):
    # return np.linalg.norm(p1-p2)
    return (p1[1] - p2[1])

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
        print(self.path)
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
                # [150, 130, 130, 110, 60, 50, 90] 
                stiffness = np.array([120, 100, 100, 80, 30, 20, 50]) # 50 (Last value) is end effector joint
                while i < len(self.path):
                    q = self.path[i]
                    # if (not self.robot.arm.InsideTorqueLimits(q, stiffness)):
                    #     raise AssertionError("Over Torque Limit, need to replan")
                    cart_pose = self.robot.arm.ComputeFK(q)
                    cart_pose = np.array(cart_pose)[:-1, -1]
                    # new_pose = arm.endpoint_pose()
                    # new_pose['position'][0] = cart_pose[0][-1]
                    # new_pose['position'][1] = cart_pose[1][-1]
                    # new_pose['position'][2] = cart_pose[2][-1]
                    # arm.set_cart_impedance_pose(new_pose, stiffness)
                    arm.set_joint_impedance_config(q, stiffness)
                    status = arm.get_robot_status()
                    if status['robot_mode'] == 4: # Cartesian reflex error
                        print("Collision!")
                        arm.resetErrors()
                        arm.hand.close()
                        new_stiffness = util.increment_stiffness(stiffness, 30, [300, 280, 280, 260, 210, 120, 120])
                        if (stiffness == new_stiffness).all():
                            print("Reached max Stiffness, need to replan")
                            # Recover
                            current_q = arm.joint_angles()
                            current_q = arm.convertToList(current_q)
                            self.robot.arm.SetJointValues(current_q)
                            arm.hand.open()
                            self.robot.arm.hand.Open()
                            self.robot.arm.ReleaseAll()
                            ans = input("Go to Recovery Position")
                            while 'N' in ans.upper():
                                arm.hand.open()
                                ans = input("Go to Recovery Position")
                            arm.resetErrors()

                            # Calculate Recovery Configuration
                            current_pos = self.robot.arm.GetEETransform()
                            current_pos[0][-1] -= 0.05
                            recover_q = self.robot.arm.ComputeIK(current_pos, seed_q=current_q)

                            # Move to recovery Position
                            arm.execute_position_path(util.convert(arm, [current_q, recover_q]))
                            self.robot.arm.SetJointValues(recover_q)

                            minForce = np.dot(np.linalg.pinv(np.array(self.robot.arm.GetJacobian(current_q)).T), arm.coriolis_comp())
                            return False, minForce # Need to Replan

                        stiffness = new_stiffness
                        input("Increasing Stiffnes to {0}. Continue?".format(stiffness))
                        continue
                    if status['robot_mode'] != 2:
                        print("Other error")
                        return
                    curr_pose = arm.endpoint_pose()
                    print('Current Pose: {0}\nExpected Pose: {1}\n'.format(curr_pose['position'], cart_pose))
                    dist = get_euclDist(cart_pose, curr_pose['position'])
                    i += 1
            return True, None


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
                ans = 'Y'
                while 'Y' in ans.upper():
                    arm.hand.open()
                    ans = input('Repeat?')
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
