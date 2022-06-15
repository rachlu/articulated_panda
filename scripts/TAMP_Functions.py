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
        if not self.robot.arm.IsCollisionFree(q2.conf, obstacles=[self.floor]):
            return (None, )
        rrt = RRT(self.robot, nonmovable = [self.floor])
        path = rrt.motion(q1.conf, q2.conf)
        for _ in range(3):
            path = rrt.motion(q1.conf, q2.conf)
            if path is not None:
                break
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
        for _ in range(40):
            # up = numpy.array([[1, 0, 0, 0],
            #                   [0, 1, 0, 0],
            #                   [0, 0, 1, -.03],
            #                   [0., 0., 0., 1.]])
            # pre_grasp = numpy.dot(grasp_pose, up)
            # pre_q = self.robot.arm.ComputeIK(pre_grasp)
            if self.robot.arm.IsCollisionFree(q):
                # Grasp in object frame
                relative_grasp = numpy.dot(numpy.linalg.inv(obj_pose.pose), grasp_pose)

                # relative_pregrasp = numpy.dot(numpy.linalg.inv(obj_pose.pose), pre_grasp)
                cmd1 = [vobj.Pose(obj, relative_grasp)]
                return (cmd1, )
        return (None, )
        
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
            return (None, )
        up = numpy.array([[1, 0, 0, 0],
                          [0, 1, 0, 0],
                          [0, 0, 1, -.07],
                          [0., 0., 0., 1.]])
        new_g = numpy.dot(grasp_in_world, up)
        translated_q = self.robot.arm.ComputeIK(new_g, seed_q = q_g)
        if translated_q is None:
            return (None, )
        q = vobj.BodyConf(self.robot, translated_q)
        traj = vobj.TrajPath(self.robot, [translated_q, q_g, translated_q])
        cmd = [q, traj]
        # return ([q], [traj], )
        return (cmd, )



    def samplePlacePose(self, obj, region):
        # Obj pose in world frame
        place_pose = self.place.place_tsr[obj].sample()
        # Maybe doesnt have to be a list
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
        obj_oldpos = self.objects[obj].get_transform()
        self.objects[obj].set_transform(pose.pose)
        for q in traj.path:
            if not self.robot.arm.IsCollisionFree(q):
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


