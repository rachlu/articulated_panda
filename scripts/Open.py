import numpy
import util
import math
import vobj
import pb_robot

class Open:
    def __init__(self, robot, objects, floor):
        self.robot = robot
        self.objects = objects
        self.nonmovable = [floor]

    def get_cabinet_traj(self, start_q, start_grasp, position, increment, sample):
        old_pos = self.objects['cabinet'].get_configuration()
        q = numpy.array(start_q)
        path = [q]
        back = numpy.array([[1, 0, 0, 0],
                            [0, 1, 0, 0],
                            [0, 0, 1, increment],
                            [0., 0., 0., 1.]])
        grasp = start_grasp
        for x in range(sample):
            grasp = numpy.dot(grasp, back)
            q = numpy.array(self.robot.arm.ComputeIKQ(grasp, q))
            if position.upper() == 'TOP':
                self.objects['cabinet'].set_configuration((-1*(increment*(x+1)), 0))
            else:
                self.objects['cabinet'].set_configuration((0, -1*(increment*(x+1))))
            for obj in set(self.objects.keys()) - {'cabinet'}:
                if pb_robot.collisions.body_collision(self.objects['cabinet'], self.objects[obj]):
                    self.objects['cabinet'].set_configuration(old_pos)
                    # print('cabinet collision')
                    return None
            if q is not None and self.robot.arm.IsCollisionFree(q):
                path.append(q)
            else:
                self.objects['cabinet'].set_configuration(old_pos)
                return None
        cmd = [vobj.TrajPath(self.robot, path[:-1]), vobj.HandCmd(self.robot, self.objects['cabinet'], status='Open'), vobj.TrajPath(self.robot, path[-2:])]
        end_pose = self.objects['cabinet'].get_configuration()
        self.objects['cabinet'].set_configuration(old_pos)
        return [cmd, vobj.BodyConf(self.robot, q), vobj.Pose('cabinet', end_pose)]

    def get_door_traj(self, start_q, relative_grasp, increment, sample):
        old_pos = self.objects['door'].get_configuration()

        self.objects['door'].set_configuration((0, ))
        knob_pose = self.objects['door'].link_from_name('knob').get_link_tform(True)
        start_grasp = numpy.dot(knob_pose, relative_grasp)

        self.objects['door'].set_configuration((math.pi/2, ))
        knob_pose = self.objects['door'].link_from_name('knob').get_link_tform(True)

        right_angle = numpy.dot(knob_pose, relative_grasp)

        x_0 = start_grasp[0][-1]
        print('x_0', x_0)
        y_0 = right_angle[1][-1]
        print('y_0', y_0)
        a = x_0 - right_angle[0][-1]
        print('a', a)
        b = start_grasp[1][-1] - y_0
        print('b', b)

        q = numpy.array(start_q)
        path = [q]
        t = math.pi/2
        for _ in range(sample):
            t += increment
            new_grasp = numpy.array(start_grasp)
            x = a * math.cos(t) + x_0
            y = b * math.sin(t) + y_0
            new_grasp[0][-1] = x
            new_grasp[1][-1] = y
            new_grasp = numpy.dot(new_grasp, util.get_rotation_arr('Z', -(t-math.pi/2)))
            q = self.robot.arm.ComputeIKQ(new_grasp, q)
            pb_robot.viz.draw_tform(new_grasp)
            self.objects['door'].set_configuration((t-math.pi/2, ))
            for obj in set(self.objects.keys()) - {'door'}:
                if pb_robot.collisions.body_collision(self.objects['door'], self.objects[obj]):
                    self.objects['door'].set_configuration(old_pos)
                    # print('collision')
                    return None
            # TODO: Add Collision Checking
            if q is not None:
                path.append(numpy.array(q))
            else:
                # print(q, 'None')
                # print(self.robot.arm.IsCollisionFree(q))
                self.objects['door'].set_configuration(old_pos)
                return None
        back = numpy.array([[1, 0, 0, 0],
                            [0, 1, 0, 0],
                            [0, 0, 1, -.15],
                            [0., 0., 0., 1.]])
        back_grasp = numpy.dot(new_grasp, back)
        q = self.robot.arm.ComputeIKQ(back_grasp, path[-1])
        if q is None:
            # print('back None')
            self.robot.arm.Release(self.objects['door'])
            self.objects['door'].set_configuration(old_pos)
            return None
        cmd = [vobj.TrajPath(self.robot, path), vobj.HandCmd(self.robot, self.objects['door'], status='Open'), vobj.TrajPath(self.robot, [path[-1], q])]
        end_pose = self.objects['door'].get_configuration()
        self.objects['door'].set_configuration(old_pos)
        return [cmd, vobj.BodyConf(self.robot, q), vobj.Pose('door', end_pose)]



