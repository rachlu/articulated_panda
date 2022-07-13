import numpy
import util
import math
import vobj
import pb_robot

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

    def get_circular(self, start_q, relative_grasp):
        old_q = self.robot.arm.GetJointValues()
        self.robot.arm.SetJointValues(start_q)
        start_grasp = self.robot.arm.GetEETransform()

        old_pos = self.objects['door'].get_transform()
        self.robot.arm.Grab(self.objects['door'], relative_grasp)
        radius = 0.25
        x_0 = start_grasp[0][-1] - radius
        y_0 = start_grasp[1][-1]
        q = numpy.array(start_q)
        path = [q]
        for t in numpy.linspace(0, -math.pi / 2, 5):
            new_grasp = start_grasp
            x = radius * math.cos(t) + x_0
            y = radius * math.sin(t) + y_0
            new_grasp[0][-1] = x
            new_grasp[1][-1] = y
            q = self.robot.arm.ComputeIKQ(new_grasp, q)
            pb_robot.viz.draw_tform(new_grasp)
            if q is not None and self.robot.arm.IsCollisionFree(q):
                path.append(numpy.array(q))
            else:
                self.robot.arm.SetJointValues(old_q)
                self.objects['door'].set_transform(old_pos)
                self.robot.arm.Release(self.objects['door'])
                return None
        back = numpy.array([[1, 0, 0, 0],
                            [0, 1, 0, 0],
                            [0, 0, 1, -.15],
                            [0., 0., 0., 1.]])
        back_grasp = numpy.dot(new_grasp, back)
        q = self.robot.arm.ComputeIKQ(back_grasp, path[-1])
        if q is None:
            self.robot.arm.SetJointValues(old_q)
            self.objects['door'].set_transform(old_pos)
            self.robot.arm.Release(self.objects['door'])
            return None
        cmd = [vobj.TrajPath(self.robot, path), vobj.HandCmd(self.robot, self.objects['door']), vobj.TrajPath(self.robot, [path[-1], q])]
        end_pose = self.objects['door'].get_transform()
        self.robot.arm.SetJointValues(old_q)
        self.objects['door'].set_transform(old_pos)
        self.robot.arm.Release(self.objects['door'])
        return [cmd, vobj.BodyConf(self.robot, q), vobj.Pose('door', end_pose)]



