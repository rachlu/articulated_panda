import random
import numpy
import vobj
import math

def sampleTable(obj):
    r = random.choice([(-0.5, -0.3), (0.2, 0.7)])
    x = random.uniform(*r)
    r = random.choice([(-0.45, -0.25), (0.25, 0.45)])
    y = random.uniform(*r)

    # Random rotation
    angle = random.uniform(0, 2*math.pi)
    rotate = numpy.array([[math.cos(angle), -math.sin(angle), 0, 0],
                 [math.sin(angle), math.cos(angle), 0, 0],
                 [0, 0, 1, 0],
                 [0., 0., 0., 1.]])

    translation = numpy.array([[1, 0, 0, x],
                                [0, 1, 0, y],
                                [0, 0, 1, 0],
                                [0., 0., 0., 1.]])

    pose = numpy.dot(translation, rotate)
    cmd = [vobj.Pose(obj, pose)]
    return (cmd,)

def collision_Test(robot, objects, nonmovable, q1, q2, sample, constraint=None):
    """
    Returns True if q1 to q2 is collision free.
    """
    for num in range(sample + 1):
        if constraint is not None:
            if not constraint[0](robot, q1 + (q2 - q1) / sample * num, objects[constraint[1]]):
                return False
        if not robot.arm.IsCollisionFree(q1 + (q2 - q1) / sample * num, obstacles=nonmovable):
            return False
    return True

def getDistance(q1, q2):
    """
    Returns the total radian distance from configuration q1 to configuration q2.
    """
    x = q1 - q2
    return numpy.sqrt(x.dot(x))
