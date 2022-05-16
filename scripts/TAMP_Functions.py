import time

from Plan import Plan
from RRT import RRT
from Grasp import Grasp
import vobj
import numpy
from Place import Place


class TAMP_Functions:
    def __init__(self, robot, objects, floor):
        self.robot = robot
        self.objects = objects
        self.floor = floor

    def calculate_path(self, q1, q2):
        print(q1, q2)
        rrt = RRT(self.robot)
        path = rrt.motion(q1.conf, q2.conf)
        while path is None:
            path = rrt.motion(q1.conf, q2.conf)
        cmd = [vobj.TrajPath(self.robot, path)]
        return (cmd, )

    def sampleGrabPose(self, obj, obj_pose):
        grasp = Grasp(self.robot, self.objects)
        # grasp_pose is grasp in world frame
        grasp_pose, q = grasp.grasp(obj)

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
            print('NO IKKKKK')
        cmd = [vobj.BodyConf(obj, conf)]
        return (cmd, )


    def samplePlacePose(self, obj, region):
        place = Place(self.robot, self.objects, self.floor)
        # Obj pose in world frame
        place_pose = place.place_tsr[obj].sample()
        cmd = [vobj.Pose(obj, place_pose)]
        return (cmd,)
