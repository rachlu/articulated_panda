from RRT import *
from Grasp import Grasp
from Place import Place
from Open import Open

import vobj
import numpy
import pb_robot


class TAMP_Functions:
    def __init__(self, robot, objects, floor, openable):
        self.robot = robot
        self.objects = objects
        self.floor = floor
        self.openable = openable
        placable = {key: objects[key] for key in (set(objects.keys()) - set(self.openable)-{'spring'})}
        self.place = Place(robot, placable, floor)
        self.grasp = Grasp(robot, objects)
        self.open_class = Open(robot, objects, floor)

    def calculate_path(self, q1, q2, constraint=None):
        print(q1, q2)
        if not self.robot.arm.IsCollisionFree(q2.conf, obstacles=[self.floor]):
            return (None,)
        rrt = RRT(self.robot, self.objects, nonmovable=[self.floor], constraint=constraint)
        for _ in range(3):
            path = rrt.motion(q1.conf, q2.conf)
            if path is not None:
                cmd = [[vobj.TrajPath(self.robot, path)]]
                return (cmd,)
        return (None, )

    def calculate_path_holding(self, q1, q2, obj, grasp):
        original_position = self.objects[obj].get_transform()
        self.robot.arm.Grab(self.objects[obj], grasp.pose)
        self.robot.arm.hand.Close()

        path = self.calculate_path(q1, q2)
        self.robot.arm.Release(self.objects[obj])
        self.robot.arm.hand.Open()
        self.objects[obj].set_transform(original_position)
        return path

    def calculate_path_holding_upright(self, q1, q2, obj, grasp):
        original_position = self.objects[obj].get_transform()
        self.robot.arm.Grab(self.objects[obj], grasp.pose)
        self.robot.arm.hand.Close()

        path = self.calculate_path(q1, q2, (rotation_constraint, obj))
        self.robot.arm.Release(self.objects[obj])
        self.robot.arm.hand.Open()
        self.objects[obj].set_transform(original_position)
        return path

    def increment(self, obj, knob):
        random_conf = random.uniform(0, 1)
        if knob == 'knob':
            random_conf *= 180
            pose = vobj.Pose((random_conf,))
        elif 'top' in knob:
            pose = vobj.Pose(obj, (random_conf, 0))
        else:
            pose = vobj.Pose(obj, (0, random_conf))
        return ([random_conf, pose], )

    def get_open_traj(self, obj, obj_pose, start_q, relative_grasp, total, knob):
        increment, sample = util.get_increment(obj, total)
        for _ in range(5):
            t2 = self.open_class.open_obj(obj, start_q.conf, relative_grasp.pose, obj_pose.pose, increment, sample, knob)
            if t2 is not None:
                # t2 = cmds, end_conf, end_pose
                cmds = [t2[0], t2[1], t2[2], increment, sample]
                return (cmds,)
        return (None,)


    '''
    def get_open_traj(self, obj, start_q, obj_pose, total):
        increment, sample = util.get_increment(obj, total)
        for _ in range(5):
            if obj == 'cabinet':
                knob = 'bottom_drawer_knob'
            else:
                knob = 'knob'
            relative_grasp = self.sampleGrabPose(obj, obj_pose, knob)[0]
            if relative_grasp is None:
                continue
            relative_grasp = relative_grasp[0]
            result = self.computeIK(obj, obj_pose, relative_grasp)[0]

            if result is None:
                print('result None')
                continue
            # end_q is up conf
            end_q, hand_traj = result
            t1 = self.calculate_path(start_q, end_q)[0]
            # Change to grasp conf
            end_q = vobj.BodyConf(self.robot, hand_traj[0].path[1])
            hand_traj[1].set_status('Close')
            t2 = self.open_class.open_obj(obj, end_q.conf, relative_grasp.pose, obj_pose.pose, increment, sample, knob)
            if t1 is not None and t2 is not None:
                t1 = t1[0]
                t1.extend(hand_traj[:2])
                cmds = [t1, t2[0], t2[1], t2[2], relative_grasp, increment, sample]
                return (cmds, )
        return (None, )
    '''
    def sample_grasp_openable(self, obj, obj_pose, knob):
        old_pos = self.objects[obj].get_configuration()
        self.objects[obj].set_configuration(obj_pose.pose)
        new_obj_pose = vobj.Pose(obj, self.objects[obj].link_from_name(knob).get_link_tform(True))
        self.objects[obj].set_configuration(old_pos)
        for _ in range(20):
            grasp_pose, q = self.grasp.grasp(obj, new_obj_pose.pose)
            if q is not None and self.robot.arm.IsCollisionFree(q, obstacles=[self.floor, self.objects[obj]]):
                # Grasp in object frame
                relative_grasp = numpy.dot(numpy.linalg.inv(new_obj_pose.pose), grasp_pose)
                cmd1 = [vobj.Pose(obj, relative_grasp)]
                return (cmd1,)

        return (None,)

    def sampleGrabPose(self, obj, obj_pose):
        # grasp_pose is grasp in world frame
        new_obj_pose = vobj.Pose(obj, obj_pose.pose)
        for _ in range(20):
            grasp_pose, q = self.grasp.grasp(obj, new_obj_pose.pose)
            if q is not None and self.robot.arm.IsCollisionFree(q, obstacles=[self.floor, self.objects[obj]]):
                # Grasp in object frame
                relative_grasp = numpy.dot(numpy.linalg.inv(new_obj_pose.pose), grasp_pose)
                cmd1 = [vobj.Pose(obj, relative_grasp)]
                return (cmd1, )
        return (None,)

    def compute_nonplaceable_IK(self, obj, obj_pose, grasp, knob):
        old_pos = self.objects[obj].get_configuration()
        self.objects[obj].set_configuration(obj_pose.pose)
        obj_pose = vobj.Pose(obj, self.objects[obj].link_from_name(knob).get_link_tform(True))
        self.objects[obj].set_configuration(old_pos)
        grasp_in_world = numpy.dot(obj_pose.pose, grasp.pose)
        q_g = self.robot.arm.ComputeIK(grasp_in_world)
        if q_g is None or not self.robot.arm.IsCollisionFree(q_g, obstacles=[self.floor]):
            print('q_g None')
            return (None,)
        up = numpy.array([[1, 0, 0, 0],
                          [0, 1, 0, 0],
                          [0, 0, 1, -.08],
                          [0., 0., 0., 1.]])
        new_g = numpy.dot(grasp_in_world, up)
        translated_q = self.robot.arm.ComputeIK(new_g, seed_q=q_g)
        if translated_q is None:
            print('translated none')
            return (None,)
        translated_q = numpy.array(translated_q)
        q_g = numpy.array(q_g)
        q1 = vobj.BodyConf(self.robot, translated_q)
        q2 = vobj.BodyConf(self.robot, q_g)
        traj = vobj.TrajPath(self.robot, [translated_q, q_g])
        hand_cmd = vobj.HandCmd(self.robot, self.objects[obj], grasp.pose)
        cmd = [q1, q2, [traj, hand_cmd]]
        return (cmd,)

    def computeIK(self, obj, obj_pose, grasp):
        """
        :param obj: string of object name
        :param obj_pose: Pose Object
        :param grasp: Relative grasp in object frame
        :return: start and end configuration and trajectory
        """
        grasp_in_world = numpy.dot(obj_pose.pose, grasp.pose)
        q_g = self.robot.arm.ComputeIK(grasp_in_world)
        if q_g is None or not self.robot.arm.IsCollisionFree(q_g, obstacles=[self.floor]):
            print('q_g None')
            return (None,)
        up = numpy.array([[1, 0, 0, 0],
                          [0, 1, 0, 0],
                          [0, 0, 1, -.08],
                          [0., 0., 0., 1.]])
        new_g = numpy.dot(grasp_in_world, up)
        translated_q = self.robot.arm.ComputeIK(new_g, seed_q=q_g)
        if translated_q is None:
            print('translated none')
            return (None,)
        translated_q = numpy.array(translated_q)
        q_g = numpy.array(q_g)
        q = vobj.BodyConf(self.robot, translated_q)
        traj = vobj.TrajPath(self.robot, [translated_q, q_g])
        hand_cmd = vobj.HandCmd(self.robot, self.objects[obj], grasp.pose)
        traj2 = vobj.TrajPath(self.robot, [q_g, translated_q])
        cmd = [q, [traj, hand_cmd, traj2]]
        return (cmd,)

    def samplePlacePose(self, obj, region):
        # Obj pose in world frame
        place_pose = self.place.place_tsr[obj].sample()
        cmd = [vobj.Pose(obj, place_pose)]
        return (cmd,)

    def collisionCheck(self, obj, pos, other, other_pos):
        """
        Return True if collision free.
        :param obj: object to be checked (string)
        :param pos: position of object
        :param other: other object (string)
        :param other_pos: other object position
        :return: True or False
        """
        print(obj, pos.pose)
        print(other, other_pos.pose)
        if obj in self.openable:
            obj_oldpos = self.objects[obj].get_configuration()
        else:
            obj_oldpos = self.objects[obj].get_transform()

        if other in self.openable:
            other_oldpos = self.objects[other].get_configuration()
        else:
            other_oldpos = self.objects[other].get_transform()
        
        if other != obj:
            if obj in self.openable:
                self.objects[obj].set_configuration(pos.pose)
            else:
                self.objects[obj].set_transform(pos.pose)
            
            if other in self.openable:
                self.objects[other].set_configuration(other_pos.pose)
            else:
                self.objects[other].set_transform(other_pos.pose)
            
            if pb_robot.collisions.body_collision(self.objects[obj], self.objects[other]):
                if obj in self.openable:
                    self.objects[obj].set_configuration(obj_oldpos)
                else:
                    self.objects[obj].set_transform(obj_oldpos)
                
                if other in self.openable:
                    self.objects[other].set_configuration(other_oldpos)
                else:
                    self.objects[other].set_transform(other_oldpos)

                print('False', obj, other)
                return False

        if obj in self.openable:
            self.objects[obj].set_configuration(obj_oldpos)
        else:
            self.objects[obj].set_transform(obj_oldpos)
        if other in self.openable:
            self.objects[other].set_configuration(other_oldpos)
        else:
            self.objects[other].set_transform(other_oldpos)
        print('True', obj, other)
        return True

    def cfreeTraj_Check(self, cmds, obj, pose):
        """
        :param cmds: List of configurations and commands
        :param obj: Object to collision check against
        :param pose: Pose of the object
        :return: True if traj is collision free
        """
        conf = False
        if obj in ['door', 'cabinet']:
            obj_oldpos = self.objects[obj].get_configuration()
            self.objects[obj].set_configuration(pose.pose)
            conf = True
        else:
            obj_oldpos = self.objects[obj].get_transform()
            self.objects[obj].set_transform(pose.pose)
        for traj in cmds:
            if isinstance(traj, vobj.TrajPath):
                for num in range(len(traj.path) - 1):
                    if not util.collision_Test(self.robot, self.objects, [self.floor, self.objects[obj]], traj.path[num], traj.path[num + 1], 50):
                        if conf:
                            self.objects[obj].set_configuration(obj_oldpos)
                        else:
                            self.objects[obj].set_transform(obj_oldpos)
                        return False
        if conf:
            self.objects[obj].set_configuration(obj_oldpos)
        else:
            self.objects[obj].set_transform(obj_oldpos)
        return True

    def cfreeTrajHolding_Check(self, traj, obj, grasp, obj2, pose):
        old_pos = self.objects[obj].get_transform()
        self.robot.arm.Grab(self.objects[obj], grasp.pose)
        result = self.cfreeTraj_Check(traj, obj2, pose)
        self.robot.arm.Release(self.objects[obj])
        self.objects[obj].set_transform(old_pos)
        return result

