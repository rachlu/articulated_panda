import time

from Plan import Plan
from RRT import RRT
from Grasp import Grasp
import vobj
import numpy
from Place import Place
import pb_robot
import random

class TAMP_Functions:
    def __init__(self, robot, objects, floor):
        self.robot = robot
        self.objects = objects
        self.floor = floor
        self.place = Place(robot, objects, floor)
        self.grasp = Grasp(robot, objects)


    def calculate_path(self, q1, q2):
        print(q1, q2)
        rrt = RRT(self.robot)
        path = rrt.motion(q1.conf, q2.conf)
        if path is None:
            print(q2.conf)
            return (None, )
        cmd = [vobj.TrajPath(self.robot, path)]
        return (cmd, )

    def calculate_path_holding(self, q1, q2, obj, grasp):
        original_position = self.objects[obj].get_transform()
        self.robot.arm.Grab(self.objects[obj], grasp.pose)
        self.robot.arm.hand.Close()
        path = self.calculate_path(q1, q2)
        self.robot.arm.Release(self.objects[obj])
        self.robot.arm.hand.Open()
        self.objects[obj].set_transform(original_position)
        return path

    def sampleGrabPose(self, obj, obj_pose):
        # grasp_pose is grasp in world frame
        grasp_pose, q = self.grasp.grasp(obj)
        for _ in range(20):
            up = numpy.array([[1, 0, 0, 0],
                              [0, 1, 0, 0],
                              [0, 0, 1, -.03],
                              [0., 0., 0., 1.]])
            pre_grasp = numpy.dot(grasp_pose, up)
            pre_q = self.robot.arm.ComputeIK(pre_grasp)
            if self.robot.arm.IsCollisionFree(q) and self.robot.arm.IsCollisionFree(pre_q):
                # Grasp in object frame
                relative_grasp = numpy.dot(numpy.linalg.inv(obj_pose.pose), grasp_pose)

                relative_pregrasp = numpy.dot(numpy.linalg.inv(obj_pose.pose), pre_grasp)
                cmd1 = [vobj.Pose(obj, relative_pregrasp), vobj.Pose(obj, relative_grasp)]
                return (cmd1, )
        return (None, )
        
    def execute_path(self, path):
        print(path)
        for action in path:
            time.sleep(1)
            if action.name == 'grab':
                down = self.computeIK(action.args[0], action.args[1], action.args[-1], seed_q=action.args[2].conf)[0][0]
                path = vobj.TrajPath(self.robot, [action.args[2].conf, down.conf])
                path.execute()
                input('next?')
                self.robot.arm.Grab(self.objects[action.args[0]], action.args[-1].pose)
                self.robot.arm.hand.Close()
                path = vobj.TrajPath(self.robot, [down.conf, action.args[2].conf])
                path.execute()
                continue
            if action.name == 'place':
                down_pose = numpy.array(action.args[1].pose)
                down_pose[2][-1] -= 0.04
                down_pose = vobj.Pose(action.args[0], down_pose)
                new_q = self.computeIK(action.args[0], down_pose, action.args[-1], seed_q=action.args[-2].conf)[0][0]
                path = vobj.TrajPath(self.robot, [action.args[-2].conf, new_q.conf])
                path.execute()
                self.robot.arm.Release(self.objects[action.args[0]])
                self.robot.arm.hand.Open()
                input('next?')
                path = vobj.TrajPath(self.robot, [new_q.conf, action.args[-2].conf])
                path.execute()
                continue

            action.args[-1].execute()

    def computeIK(self, obj, obj_pose, grasp, seed_q=None):
        # grasp is grasp in object frame
        grasp_in_world = numpy.dot(obj_pose.pose, grasp.pose)
        conf = self.robot.arm.ComputeIK(grasp_in_world, seed_q)
        if conf == None:
            return (None, )
        cmd = [vobj.BodyConf(obj, conf)]
        return (cmd, )


    def samplePlacePose(self, obj, region):
        # Obj pose in world frame
        place_pose = self.place.place_tsr[obj].sample()
        cmd = [vobj.Pose(obj, place_pose)]
        return (cmd, )


    def sampleTable(self, obj, objPose):
        x = random.uniform(-0.5, 0.5)
        y = random.uniform(-0.6, 0.6)
        pose = numpy.array(objPose.pose)
        pose[0][3] = x
        pose[1][3] = y
        cmd = [vobj.Pose(obj, pose)]
        return (cmd, )

    def collisionCheck(self, obj, pos, other, other_pos):
        obj_oldpos = self.objects[obj].get_transform()
        other_oldpos = self.objects[other].get_transform()
        if other != obj:
            self.objects[obj].set_transform(pos)
            self.objects[other].set_transform(other_pos)
            if pb_robot.collisions.body_collision(self.objects[obj], self.objects[other]):
                self.objects[obj].set_transform(obj_oldpos)
                self.objects[other].set_transform(other_oldpos)
                print('True', obj, other)
                return True
        self.objects[obj].set_transform(obj_oldpos)
        self.objects[other].set_transform(other_oldpos)
        print('False', obj, other)
        return False

    def cfreeTraj_Check(self, traj, obj, pose):
        obj_oldpos = self.objects[obj].get_transform()
        self.objects[obj].set_transform(pose.pose)
        for q in traj:
            if not self.robot.arm.IsCollisionFree(q):
                self.objects[obj].set_transform(obj_oldpos)
                return False
        self.objects[obj].set_transform(obj_oldpos)
        return True

    def cfreeTrajHolding_Check(self, traj, obj, grasp, obj2, pose):
        self.robot.arm.Grab(self.objects[obj], grasp.pose)
        result = self.cfreeTraj_Check(traj, obj2, pose)
        self.robot.arm.Release(self.objects[obj])
        return result


