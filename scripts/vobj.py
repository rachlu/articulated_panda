import numpy
import time


class BodyConf:
    def __init__(self, body, q):
        self.body = body
        self.conf = q

    def __repr__(self):
        return 'q{}'.format(id(self) % 1000)


class TrajPath:
    def __init__(self, robot, path):
        self.robot = robot
        self.path = path

    def __repr__(self):
        return 't{}'.format(id(self) % 1000)

    def execute(self):
        self.robot.arm.ExecutePositionPath(self.path, timestep=0.2)
    
    #def __str__(self):
    #    return str(self.path)

class HandCmd:
    def __init__(self, robot, obj, grasp=None):
        self.robot = robot
        self.obj = obj
        self.grasp = grasp

    def execute(self):
        if len(self.robot.arm.grabbedObjects) != 0:
            self.robot.arm.hand.Open()
            self.robot.arm.Release(self.obj)
        else:
            self.robot.arm.hand.Close()
            self.robot.arm.Grab(self.obj, self.grasp)

    def __repr__(self):
        return 't{}'.format(id(self) % 1000)

    #def __str__(self):
    #    result = 'Hand Cmd'
    #    return result

class Pose:
    def __init__(self, obj, pose):
        self.obj = obj
        self.pose = pose

    def __repr__(self):
        return 'p{}'.format(id(self) % 1000)
