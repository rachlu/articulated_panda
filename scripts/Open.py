import numpy
import util

class Open:
    def __init__(self, robot, objects, floor, door_type='cabinet'):
        self.door_type = door_type
        self.robot = robot
        self.objects = objects
        self.nonmovable = [floor]

    def get_trajectory(self, start_q, start_grasp):
        path = [start_q]
        back = numpy.array([[1, 0, 0, 0],
                            [0, 1, 0, 0],
                            [0, 0, 1, -.05],
                            [0., 0., 0., 1.]])
        grasp = start_grasp
        q = start_q
        for _ in range(5):
            grasp = numpy.dot(grasp, back)
            q = self.robot.arm.ComputeIK(grasp, seed_q=q)
            # Add check that max_distance is met
            if q is not None:
                path.append(q)
            else:
                return None
        return path

    def check_traj(self, start_q, end_grasp):
        path = [start_q]
        num = 1
        q1 = numpy.array(start_q)
        q2 = self.robot.arm.ComputeIK(end_grasp, seed_q=q1, max_distance=0.5)
        sample = 5
        while num < sample:
            q_new = q1 + (q2 - q1) / sample * num
            q_before = q1 + (q2 - q1) / sample * (num-1)
            if util.collision_Test(self.robot, self.objects, self.nonmovable, q_before, q_new, 50):
                path.append(q_new)
            else:
                return None
            # q1 = q_new
            num += 1
        return path

