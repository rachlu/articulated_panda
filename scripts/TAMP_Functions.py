import time

from Plan import Plan
from RRT import RRT
from Grasp import Grasp
import vobj
import numpy
from Place import Place
import pb_robot


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
            return (None, )
        cmd = [vobj.TrajPath(self.robot, path)]
        return (cmd, )

    def sampleGrabPose(self, obj, obj_pose):
        # grasp_pose is grasp in world frame
        grasp_pose, q = self.grasp.grasp(obj)
        if not self.robot.arm.IsCollisionFree(q):
            return (False, )
        # Grasp in object frame
        relative_grasp = numpy.dot(numpy.linalg.inv(obj_pose.pose), grasp_pose)
        cmd = [vobj.Pose(obj, relative_grasp)]
        return (cmd, )

    def execute_path(self, path):
        # Check type for open or close hand
        print(path)
        for action in path:
            time.sleep(1)
            if action.name == 'grab':
                self.robot.arm.Grab(self.objects[action.args[0]], action.args[-1].pose)
                self.robot.arm.hand.Close()
                continue
            if action.name == 'place':
                self.robot.arm.Release(self.objects[action.args[0]])
                self.robot.arm.hand.Open()
                continue

            action.args[-1].execute()

    def computeIK(self, obj, obj_pose, grasp):
        # grasp is grasp in object frame
        grasp_in_world = numpy.dot(obj_pose.pose, grasp.pose)
        conf = self.robot.arm.ComputeIK(grasp_in_world)
        if conf == None:
            return (None, )
        cmd = [vobj.BodyConf(obj, conf)]
        return (cmd, )


    def samplePlacePose(self, obj, region):
        # Obj pose in world frame
        place_pose = self.place.place_tsr[obj].sample()
        # original_pose = self.objects[obj].get_transform()
        # for obj in objPoses:
        #     self.objects
        # self.objects[obj].set_transform(place_pose)
        # for other in self.objects:
        #     if other != obj and \
        #             pb_robot.collisions.body_collision(self.objects[obj], self.objects[other]):
        #         return (None, )
        # self.objects[obj].set_transform(original_pose)
        cmd = [vobj.Pose(obj, place_pose)]
        return (cmd, )


    def sampleTable(self, obj, floor):
        pass

