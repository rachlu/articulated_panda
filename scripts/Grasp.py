from tsr.tsr import TSR

import numpy
import random
import math
import util


class Grasp:
    def __init__(self, robot, objects):
        self.objects = objects
        self.robot = robot
        self.relative = {}
        self.bw_range = {}
        self.utensils = ('fork', 'knife', 'spoon')
        self.grasp_tsr = {}
        self.set_info()

    def set_info(self):
        # bowl

        # Relative rotation of the robot from the bowl
        t_1 = util.get_rotation_arr('X', math.pi)
        t_2 = util.get_rotation_arr('X', -math.pi/7)
        rotation = numpy.dot(t_1, t_2)

        # Relative translation of the robot from the bowl
        translation = numpy.array([[1, 0, 0, 0],
                                   [0, 1, 0, -.039],
                                   [0, 0, 1, -.17],
                                   [0., 0., 0., 1.]])
        rel = numpy.dot(rotation, translation)
        self.relative[('bowl')] = [rel]

        # Rotation 180 degrees of the robot end-effector
        t_3 = util.get_rotation_arr('Z', math.pi)
        rotation = numpy.linalg.multi_dot([t_1, t_2, t_3])
        translation = numpy.array([[1, 0, 0, 0],
                                   [0, 1, 0, .039],
                                   [0, 0, 1, -.17],
                                   [0., 0., 0., 1.]])
        rel = numpy.dot(rotation, translation)
        self.relative[('bowl')].append(rel)

        # TSR range
        bw = numpy.array([[0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [-math.pi, math.pi]])
        self.bw_range[('bowl')] = bw

        # utensils
        rotation = util.get_rotation_arr('Y', math.pi)
        translation = numpy.array([[1, 0, 0, 0],
                                   [0, 1, 0, 0],
                                   [0, 0, 1, -.125],
                                   [0., 0., 0., 1.]])
        rel = numpy.dot(rotation, translation)
        self.relative[self.utensils] = [rel]

        z = util.get_rotation_arr('Z', math.pi)
        rotation = numpy.dot(z, rotation)
        rel = numpy.dot(rotation, translation)

        self.relative[self.utensils].append(rel)

        bw = numpy.array([[-0.03, 0.03], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0]])
        self.bw_range[self.utensils] = bw

        # Out and down
        # Close Cabinet
        translation = numpy.array([[1, 0, 0, -0.062],
                                   [0, 1, 0, 0],
                                   [0, 0, 1, -0.03],
                                   [0, 0, 0, 1]])
        t1 = util.get_rotation_arr('X', math.pi)
        t2 = util.get_rotation_arr('Y', math.pi/8)
        rotation = numpy.dot(t1, t2)
        rel = numpy.dot(translation, rotation)
        self.relative['cabinetClose'] = [rel]

        # t3 = util.get_rotation_arr('Z', math.pi)
        # rotation = numpy.dot(rotation, t3)
        # rel = numpy.dot(translation, rotation)
        self.relative['cabinetClose'].append(rel)

        bw = numpy.array([[0, 0], [-0.02, 0.02], [0, 0], [0, 0], [0, 0], [0, 0]])
        self.bw_range['cabinetClose'] = bw

        # Cabinet
        translation = numpy.array([[1, 0, 0, 0],
                                   [0, 1, 0, 0],
                                   [0, 0, 1, -0.13],
                                   [0, 0, 0, 1]])
        t1 = util.get_rotation_arr('X', 3 * math.pi / 2)
        t2 = util.get_rotation_arr('Y', math.pi / 2)
        rotation = numpy.dot(t1, t2)
        rel = numpy.dot(rotation, translation)
        self.relative['cabinetOpen'] = [rel]

        t1 = util.get_rotation_arr('X', math.pi / 2)
        rotation = numpy.dot(t1, t2)
        rel = numpy.dot(rotation, translation)
        self.relative['cabinetOpen'].append(rel)

        bw = numpy.array([[0, 0], [-0.02, 0.02], [0, 0], [0, 0], [0, 0], [0, 0]])
        self.bw_range['cabinetOpen'] = bw

        # Door
        translation = numpy.array([[1, 0, 0, 0],
                                   [0, 1, 0, 0],
                                   [0, 0, 1, 0.11],
                                   [0, 0, 0, 1]])
        t1 = util.get_rotation_arr('X', math.pi)
        t2 = util.get_rotation_arr('Z', math.pi / 2)
        rotation = numpy.dot(t1, t2)
        rel = numpy.dot(translation, rotation)
        self.relative['doorOpen'] = [rel]

        t2 = util.get_rotation_arr('Z', -math.pi / 2)
        rotation = numpy.dot(t1, t2)
        rel = numpy.dot(translation, rotation)
        self.relative['doorOpen'].append(rel)
        bw = numpy.array([[0, 0], [-0.11, 0.11], [-0.018, 0.018], [0, 0], [0, 0], [0, 0]])
        self.bw_range['doorOpen'] = bw

        # Spring
        rotation = util.get_rotation_arr('Y', math.pi)
        translation = numpy.array([[1, 0, 0, 0],
                                   [0, 1, 0, 0],
                                   [0, 0, 1, -.13],
                                   [0., 0., 0., 1.]])
        z = util.get_rotation_arr('Z', math.pi)
        rotation = numpy.dot(z, rotation)
        rel = numpy.dot(rotation, translation)
        self.relative['spring'] = [rel]

        z = util.get_rotation_arr('Z', math.pi)
        rotation = numpy.dot(z, rotation)
        rel = numpy.dot(rotation, translation)
        self.relative['spring'].append(rel)

        bw = numpy.array([[0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0]])
        self.bw_range['spring'] = bw

    def set_tsr(self, obj, pose):
        if obj in self.utensils:
            # Obj Pose, Relative Pose, Range
            self.grasp_tsr[obj] = [TSR(pose, self.relative[self.utensils][0], self.bw_range[self.utensils])]
            self.grasp_tsr[obj].append(TSR(pose, self.relative[self.utensils][1], self.bw_range[self.utensils]))
        else:
            self.grasp_tsr[obj] = [TSR(pose, self.relative[obj][0], self.bw_range[obj])]
            self.grasp_tsr[obj].append(TSR(pose, self.relative[obj][1], self.bw_range[obj]))

    def grasp(self, obj, pose):
        # Sample grasp of obj
        # r,g,b = x,y,z
        self.set_tsr(obj, pose)
        grasp_idx = random.randint(0, 1)
        # Grasp in world frame
        grasp_world = self.grasp_tsr[obj][grasp_idx].sample()
        computed_q = self.robot.arm.ComputeIK(grasp_world)
        return grasp_world, computed_q

    
        
    
