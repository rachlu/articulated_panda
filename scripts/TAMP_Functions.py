import time
import vobj
import numpy
import pb_robot
import random
import util

from Plan import Plan
from RRT import *
from Grasp import Grasp
from Place import Place


class TAMP_Functions:
    def __init__(self, robot, objects, floor):
        self.robot = robot
        self.objects = objects
        self.floor = floor
        placable = {key: objects[key] for key in (set(objects.keys()) - {'door'})}
        self.place = Place(robot, placable, floor)
        self.grasp = Grasp(robot, objects)

    def calculate_path(self, q1, q2, constraint=None):
        print(q1, q2)
        if not self.robot.arm.IsCollisionFree(q2.conf, obstacles=[self.floor]):
            return (None,)
        rrt = RRT(self.robot, self.objects, nonmovable=[self.floor], constraint=constraint)
        path = rrt.motion(q1.conf, q2.conf)
        for _ in range(3):
            path = rrt.motion(q1.conf, q2.conf)
            if path is not None:
                break
        if path is None:
            print(q2.conf)
            return (None,)
        cmd = [vobj.TrajPath(self.robot, path)]
        return (cmd,)

    def calculate_path_holding(self, q1, q2, obj, grasp):
        original_position = self.objects[obj].get_transform()
        self.robot.arm.Grab(self.objects[obj], grasp.pose)
        self.robot.arm.hand.Close()

        path = self.calculate_path(q1, q2)
        self.robot.arm.Release(self.objects[obj])
        self.robot.arm.hand.Open()
        self.objects[obj].set_transform(original_position)
        return path

    def calculate_path_holding_upright(self, q1, q2, obj, grasp):
        original_position = self.objects[obj].get_transform()
        self.robot.arm.Grab(self.objects[obj], grasp.pose)
        self.robot.arm.hand.Close()

        path = self.calculate_path(q1, q2, (rotation_constraint, obj))
        self.robot.arm.Release(self.objects[obj])
        self.robot.arm.hand.Open()
        self.objects[obj].set_transform(original_position)
        return path

    def sampleGrabPose(self, obj, obj_pose):
        # grasp_pose is grasp in world frame
        grasp_pose, q = self.grasp.grasp(obj)
        if q is None:
            return (None,)
        for _ in range(40):
            if self.robot.arm.IsCollisionFree(q, obstacles=[self.floor]):
                # Grasp in object frame
                relative_grasp = numpy.dot(numpy.linalg.inv(obj_pose.pose), grasp_pose)
                cmd1 = [vobj.Pose(obj, relative_grasp)]
                return (cmd1,)
        return (None,)

    def execute_path(self, path):
        print(path)
        for action in path:
            time.sleep(1)
            if action.name == 'grab':
                obj, obj_pose, grasp, conf, traj = action.args
                start = vobj.TrajPath(self.robot, traj.path[:2])
                end = vobj.TrajPath(self.robot, traj.path[1:])
                start.execute()
                self.robot.arm.Grab(self.objects[obj], grasp.pose)
                self.robot.arm.hand.Close()
                end.execute()
                continue
            if action.name == 'place':
                obj, obj_pose, grasp, conf, traj = action.args
                start = vobj.TrajPath(self.robot, traj.path[:2])
                end = vobj.TrajPath(self.robot, traj.path[1:])
                start.execute()
                self.robot.arm.Release(self.objects[obj])
                self.robot.arm.hand.Open()
                end.execute()
                continue

            action.args[-1].execute()

    def computeIK(self, obj, obj_pose, grasp):
        """
        :param obj: string of object name
        :param obj_pose: Pose Object
        :param grasp: Relative grasp in object frame
        :return: start and end configuration and trajectory
        """
        grasp_in_world = numpy.dot(obj_pose.pose, grasp.pose)
        q_g = self.robot.arm.ComputeIK(grasp_in_world)
        if q_g is None or not self.robot.arm.IsCollisionFree(q_g, obstacles=[self.floor]):
            return (None,)
        up = numpy.array([[1, 0, 0, 0],
                          [0, 1, 0, 0],
                          [0, 0, 1, -.08],
                          [0., 0., 0., 1.]])
        new_g = numpy.dot(grasp_in_world, up)
        translated_q = self.robot.arm.ComputeIK(new_g, seed_q=q_g)
        if translated_q is None:
            return (None,)
        translated_q = numpy.array(translated_q)
        q_g = numpy.array(q_g)
        q = vobj.BodyConf(self.robot, translated_q)
        traj = vobj.TrajPath(self.robot, [translated_q, q_g, translated_q])
        cmd = [q, traj]
        return (cmd,)

    def samplePlacePose(self, obj, region):
        # Obj pose in world frame
        place_pose = self.place.place_tsr[obj].sample()
        cmd = [vobj.Pose(obj, place_pose)]
        return (cmd,)

    def collisionCheck(self, obj, pos, other, other_pos):
        """
        Return True if collision free.
        :param obj: object to be checked (string)
        :param pos: position of object
        :param other: other object (string)
        :param other_pos: other object position
        :return: True or False
        """
        obj_oldpos = self.objects[obj].get_transform()
        other_oldpos = self.objects[other].get_transform()
        if other != obj:
            self.objects[obj].set_transform(pos.pose)
            self.objects[other].set_transform(other_pos.pose)
            if pb_robot.collisions.body_collision(self.objects[obj], self.objects[other]):
                self.objects[obj].set_transform(obj_oldpos)
                self.objects[other].set_transform(other_oldpos)
                print('False', obj, other)
                return False
        self.objects[obj].set_transform(obj_oldpos)
        self.objects[other].set_transform(other_oldpos)
        print('True', obj, other)
        return True

    def cfreeTraj_Check(self, traj, obj, pose):
        """
        :param traj: List of configurations
        :param obj: Object to collision check against
        :param pose: Pose of the object
        :return: True if traj is collision free
        """
        obj_oldpos = self.objects[obj].get_transform()
        self.objects[obj].set_transform(pose.pose)
        for num in range(len(traj.path) - 1):
            if not util.collision_Test(self.robot, self.objects, [self.floor, self.objects[obj]], traj.path[num], traj.path[num + 1], 50):
                self.objects[obj].set_transform(obj_oldpos)
                return False
        self.objects[obj].set_transform(obj_oldpos)
        return True

    def cfreeTrajHolding_Check(self, traj, obj, grasp, obj2, pose):
        old_pos = self.objects[obj].get_transform()
        self.robot.arm.Grab(self.objects[obj], grasp.pose)
        result = self.cfreeTraj_Check(traj, obj2, pose)
        self.robot.arm.Release(self.objects[obj])
        self.objects[obj].set_transform(old_pos)
        return result
