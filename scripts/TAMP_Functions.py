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

    def sample_config(self):
        rrt = RRT(self.robot)
        config = vobj.BodyConf(self.robot, rrt.sample_config())
        return (config, )

    def calculate_path(self, q1, q2):
        rrt = RRT(self.robot)
        path = rrt.motion(q1.conf, q2.conf)
        while path is None:
            path = rrt.motion(q1.conf, q2.conf)
        cmd = [vobj.TrajPath(self.robot, path)]
        return (cmd, )

    def sampleGrabPose(self, obj):
        grasp = Grasp(self.robot, self.objects)
        grasp_pose, q = grasp.grasp(obj)
        cmd = [vobj.Pose(obj, grasp_pose)]
        return (cmd, )

    def execute_path(self, path):
        # Check type for open or close hand
        print(path)
        for action in path:
            time.sleep(1)
            if action.name == 'grab':
                relative_grasp = numpy.dot(numpy.linalg.inv(self.objects[action.args[0]].get_transform()), action.args[2].pose)
                self.robot.arm.Grab(self.objects[action.args[0]], relative_grasp)
                self.robot.arm.hand.Close()
                continue
            if action.name == 'place':
                self.robot.arm.Release(self.objects[action.args[0]])
                self.robot.arm.hand.Open()
                continue

            action.args[-1].execute()

    def computeIK(self, pose):
        cmd = [vobj.BodyConf(self.robot, self.robot.arm.ComputeIK(pose.pose))]
        return (cmd, )

    def samplePlacePose(self, obj, grasp_pose):
        place = Place(self.robot, self.objects, self.floor)
        place_pose = place.place_tsr[obj].sample()
        relative_grasp = numpy.dot(numpy.linalg.inv(self.objects[obj].get_transform()), grasp_pose.pose)
        relative_pose = numpy.dot(place_pose, relative_grasp)
        cmd = [vobj.Pose(obj, relative_pose)]
        return (cmd,)
