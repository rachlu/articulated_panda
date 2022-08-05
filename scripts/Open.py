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

    def get_cabinet_traj(self, start_q, relative_grasp, obj_pose, position, increment, sample):
        old_pos = self.objects['cabinet'].get_configuration()
        self.objects['cabinet'].set_configuration(obj_pose)
        q = numpy.array(start_q)
        path = [q]
        for x in range(sample+1):
            if position.upper() == 'TOP':
                self.objects['cabinet'].set_configuration((obj_pose[0] + 1*(increment*(x+1)), obj_pose[1]))
            else:
                self.objects['cabinet'].set_configuration((obj_pose[0], obj_pose[1] + 1*(increment*(x+1))))
            knob_pose = self.objects['cabinet'].link_from_name(position + '_drawer_knob').get_link_tform(True)
            world_grasp = numpy.dot(knob_pose, relative_grasp)
            pb_robot.viz.draw_tform(world_grasp)
            q = numpy.array(self.robot.arm.ComputeIKQ(world_grasp, q))
            for obj in set(self.objects.keys()) - {'cabinet'}:
                if pb_robot.collisions.body_collision(self.objects['cabinet'], self.objects[obj]):
                    self.objects['cabinet'].set_configuration(old_pos)
                    print('collison')
                    return None
            if q is not None and self.robot.arm.IsCollisionFree(q, obstacles=[self.floor, self.objects['cabinet']]):
                path.append(q)
            else:
                self.objects['cabinet'].set_configuration(old_pos)
                return None
        cmd = [vobj.TrajPath(self.robot, path[:-1]), vobj.HandCmd(self.robot, self.objects['cabinet'], status='Open'), vobj.TrajPath(self.robot, path[-2:])]
        end_pose = self.objects['cabinet'].get_configuration()
        self.objects['cabinet'].set_configuration(old_pos)
        return [cmd, vobj.BodyConf(self.robot, q), vobj.Pose('cabinet', end_pose)]

    def get_door_traj(self, start_q, relative_grasp, obj_pose, increment, sample):
        old_pos = self.objects['door'].get_configuration()
        self.objects['door'].set_configuration(obj_pose)
        q = numpy.array(start_q)
        path = [q]
        t = obj_pose
        for _ in range(sample):
            t += increment
            self.objects['door'].set_configuration((t,))
            knob_pose = self.objects['door'].link_from_name('knob').get_link_tform(True)
            new_grasp = numpy.dot(knob_pose, relative_grasp)
            pb_robot.viz.draw_tform(new_grasp)
            q = self.robot.arm.ComputeIKQ(new_grasp, q)
            for obj in set(self.objects.keys()) - {'door'}:
                if pb_robot.collisions.body_collision(self.objects['door'], self.objects[obj]):
                    self.objects['door'].set_configuration(old_pos)
                    return None

            if q is not None and self.robot.arm.IsCollisionFree(q, obstacles=[self.floor, self.objects['door']]):
                path.append(numpy.array(q))
            else:
                self.objects['door'].set_configuration(old_pos)
                return None

        back = numpy.array([[1, 0, 0, 0],
                            [0, 1, 0, 0],
                            [0, 0, 1, -.15],
                            [0., 0., 0., 1.]])
        back_grasp = numpy.dot(new_grasp, back)
        q = self.robot.arm.ComputeIKQ(back_grasp, path[-1])
        if q is None:
            self.robot.arm.Release(self.objects['door'])
            self.objects['door'].set_configuration(old_pos)
            return None
        cmd = [vobj.TrajPath(self.robot, path), vobj.HandCmd(self.robot, self.objects['door'], status='Open'),
               vobj.TrajPath(self.robot, [path[-1], q])]
        end_pose = self.objects['door'].get_configuration()
        self.objects['door'].set_configuration(old_pos)
        return [cmd, vobj.BodyConf(self.robot, q), vobj.Pose('door', end_pose)]



