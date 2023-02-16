from scipy.spatial.transform import Rotation as R

import numpy
import util
import math
import vobj
import pb_robot


class Open:
    def __init__(self, robot, objects, floor):
        self.robot = robot
        self.objects = objects
        self.floor = floor

    def open_obj(self, obj, start_q, relative_grasp, obj_conf, increment, sample, knob):
        print(increment, sample)
        old_pos = self.objects[obj].get_configuration()
        self.objects[obj].set_configuration(obj_conf)
        q = numpy.array(start_q)
        path = [q]
        t = numpy.array(obj_conf)
        for _ in range(sample):
            t += increment
            self.objects[obj].set_configuration(t)
            knob_pose = self.objects[obj].link_from_name(knob).get_link_tform(True)
            new_grasp = numpy.dot(knob_pose, relative_grasp)
            print('new_grasp', new_grasp)
            q = self.robot.arm.ComputeIKQ(new_grasp, q)
            if q is not None and self.robot.arm.IsCollisionFree(q, obstacles=[self.floor]):
                path.append(numpy.array(q))
            else:
                print('grasp', new_grasp)
                print('q', q)
                self.objects[obj].set_configuration(old_pos)
                return None

        back = numpy.array([[1, 0, 0, 0],
                            [0, 1, 0, 0],
                            [0, 0, 1, -.07],
                            [0., 0., 0., 1.]])
        back_grasp = numpy.dot(new_grasp, back)
        q = self.robot.arm.ComputeIKQ(back_grasp, path[-1])
        if q is None:
            print('back None')
            self.robot.arm.Release(self.objects[obj])
            self.objects[obj].set_configuration(old_pos)
            return None

        cmd = [vobj.TrajPath(self.robot, path, impedance=True), vobj.HandCmd(self.robot, self.objects[obj], status='Open'),
               vobj.TrajPath(self.robot, [path[-1], q])]
        end_pose = self.objects[obj].get_configuration()
        self.objects[obj].set_configuration(old_pos)
        return [cmd, vobj.BodyConf(self.robot, q), vobj.Pose(obj, end_pose)]




