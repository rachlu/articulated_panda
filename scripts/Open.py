import numpy
import util
import math
import vobj

class Open:
    def __init__(self, robot, objects, floor, door_type='cabinet'):
        self.door_type = door_type
        self.robot = robot
        self.objects = objects
        self.nonmovable = [floor]

    def get_trajectory(self, start_q, start_grasp):
        q = numpy.array(start_q)
        path = [q]
        back = numpy.array([[1, 0, 0, 0],
                            [0, 1, 0, 0],
                            [0, 0, 1, -.05],
                            [0., 0., 0., 1.]])
        grasp = start_grasp
        for _ in range(5):
            grasp = numpy.dot(grasp, back)
            q = numpy.array(self.robot.arm.ComputeIKQ(grasp, q))
            if q is not None:
                path.append(q)
            else:
                return None
        return path

    def get_circular(self, start_q, start_grasp):
        q = numpy.array(start_q)
        path = [q]
        for t in numpy.linspace(0, -math.pi / 2, 5):
            new_grasp = start_grasp
            x = 0.2 * math.cos(t) + 0.17
            y = 0.2 * math.sin(t) + 0.383
            new_grasp[0][-1] = x
            new_grasp[1][-1] = y
            q = self.robot.arm.ComputeIKQ(new_grasp, q)
            if q is not None:
                path.append(numpy.array(q))
                print(util.getDistance(path[-2], path[-1]))
            else:
                return None
        back = numpy.array([[1, 0, 0, 0],
                            [0, 1, 0, 0],
                            [0, 0, 1, -.15],
                            [0., 0., 0., 1.]])
        back_grasp = numpy.dot(new_grasp, back)
        q = self.robot.arm.ComputeIKQ(back_grasp, path[-1])
        if q is None:
            return None
        cmd = [vobj.TrajPath(self.robot, path), vobj.HandCmd(self.robot, self.objects['door']), vobj.TrajPath(self.robot, [path[-1], q])]
        return cmd



