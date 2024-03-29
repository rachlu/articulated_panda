from RRT import *
from Grasp import Grasp
from Place import Place
from Open import Open

import vobj
import numpy as np
import pb_robot


class TAMP_Functions:
    def __init__(self, robot, objects, floor, openable, forbiddenQs=[]):
        self.robot = robot
        self.objects = objects
        self.floor = floor
        self.openable = openable
        self.place = Place(robot, objects, floor)
        self.grasp = Grasp(robot, objects)
        self.open_class = Open(robot, objects, floor)
        self.forbiddenQs = forbiddenQs

    def calculate_path(self, q1, q2, constraint=None, nonmovable=[]):
        print(q1, q2)
        if not self.robot.arm.IsCollisionFree(q2.conf, obstacles=[self.floor]):
            return None
        nonmovable.extend([self.floor])
        rrt = RRT(self.robot, self.objects, nonmovable=nonmovable, constraint=constraint)
        for _ in range(7):
            print("Iteration:", _)
            path = rrt.motion(q1.conf, q2.conf)
            if path is not None:
                cmd = [[vobj.TrajPath(self.robot, path)]]
                return cmd
        return None

    def calculate_path_holding(self, q1, q2, obj, grasp):
        original_position = self.objects[obj].get_transform()
        self.robot.arm.Grab(self.objects[obj], grasp.pose, 'RB')
        self.robot.arm.hand.Close()

        path = self.calculate_path(q1, q2)
        self.robot.arm.Release(self.objects[obj])
        self.robot.arm.hand.Open()
        self.objects[obj].set_transform(original_position)
        return path

    def calculate_path_holding_upright(self, q1, q2, obj, grasp):
        original_position = self.objects[obj].get_transform()
        self.robot.arm.Grab(self.objects[obj], grasp.pose, 'RB')
        self.robot.arm.hand.Close()

        path = self.calculate_path(q1, q2, (rotation_constraint, obj))
        self.robot.arm.Release(self.objects[obj])
        self.robot.arm.hand.Open()
        self.objects[obj].set_transform(original_position)
        return path
    
    def randomConf(self):
        q = self.robot.arm.randomConfiguration()
        while not self.robot.arm.IsCollisionFree(q, obstacles=[self.floor]):
            q = self.robot.arm.randomConfiguration()
        return [vobj.BodyConf(self.robot, q)]

    def sample_delta_openableconf(self, obj, knob):
        # Assuming that we only want the door or cabinet to open all the way
        if obj == 'door':
            random_conf = random.uniform(0, 50)
            delta_pose = np.array((random_conf, ))
            # delta_pose = np.array((50, ))
        else:
            # Cabinet open all the way is 0.3
            random_conf = random.uniform(0, 0.17)
            # random_conf = 0.17
            if 'top' in knob:
                delta_pose = np.array((random_conf, 0))
            else:
                delta_pose = np.array((0, random_conf))
        delta_pose = [vobj.BodyConf(obj, delta_pose)]
        return delta_pose

    def sample_openableconf(self, obj, conf, knob):
        # for _ in range(25):
        #     delta = self.sample_delta_openableconf(obj, knob)[0]
        #     new_conf = vobj.BodyConf(obj, conf.conf + delta.conf)
        #     if delta.conf[0]:
        #         if 0.15 <= new_conf.conf[0] <= 0.17:
        #             return [new_conf]
        #     else:
        #         if 0.15 <= new_conf.conf[1] <= 0.17:
        #             return [new_conf]        
        if obj == 'door':
            random_conf = random.uniform(40, 50)
            new_conf = vobj.BodyConf(obj, (random_conf,))
        else:
            random_conf = random.uniform(0.15, 0.17)
            if 'top' in knob:
                new_conf = vobj.BodyConf(obj, (random_conf, conf.conf[1]))
            else:
                new_conf = vobj.BodyConf(obj, (conf.conf[0], random_conf))
        return [new_conf]

    def sample_close_conf(self, obj, conf, knob):
        if obj == 'cabinet':
            if 'top' in knob:
                new_conf = [vobj.BodyConf(obj, np.array((0, conf.conf[1])))]
            else:
                new_conf = [vobj.BodyConf(obj, np.array((conf.conf[0], 0)))]
        else:
            new_conf = [vobj.BodyConf(obj, np.array((0,)))]
        return new_conf


    def test_open_enough(self, obj, obj_conf, knob):
        print('test_open_enough', obj_conf.conf)
        if obj == 'door':
            if 40 <= obj_conf.conf[0] <= 50:
                return True

        current = obj_conf.conf[0] if 'top' in knob else obj_conf.conf[1]
        if 0.15 <= current:
            return True

        return False
        
    def get_open_traj_merge(self, obj, obj_conf, end_conf, start_q, knob, minForce):
        print('Open Start Conf', start_q.conf)
        init_q = self.robot.arm.GetJointValues()
        for _ in range(5):
            for _ in range(5):
                relative_grasp = self.sample_grasp_openable('Open')(obj, obj_conf, knob)[0]
                q, q_grasp, grab_t = self.compute_nonplaceable_IK(obj, obj_conf, relative_grasp, knob, 'Open')
                print("Open grasp q", q.conf)
                t = self.calculate_path(start_q, q)
                if t is not None:
                    t = t[0]
                    # cur_q = t[0].path[-1]
                    # self.robot.arm.SetJointValues(cur_q)
                    # current_pos = self.robot.arm.GetEETransform
                    # current_pos[0][-1] -= 0.05
                    # recover_q = self.robot.ComputeIK(current_pos, seed_q=current_q)

                    print("Open Traj", t)
                    break
            result = self.get_openable_traj(obj, obj_conf, end_conf, q_grasp, relative_grasp, knob, minForce, 'Open')
            if result is not None:
                t2, end_q = result
                t.extend(grab_t)
                print("Open end t", t)
                print("Open Traj", t2)
                return [end_q, relative_grasp, t, t2]
        return None
    
    def get_close_traj_merge(self, obj, obj_conf, end_conf, start_q, knob, minForce):
        for _ in range(10):
            for _ in range(10):
                relative_grasp = self.sample_grasp_openable('Close')(obj, obj_conf, knob)[0]
                q, q_grasp, grab_t = self.compute_nonplaceable_IK(obj, obj_conf, relative_grasp, knob, 'Close')
                t = self.calculate_path(start_q, q)
                if t is not None:
                    t = t[0]
                    break
            if t is None:
                print("t None for close")
                continue
            result = self.get_openable_traj(obj, obj_conf, end_conf, q_grasp, relative_grasp, knob, minForce, 'Close')
            if result is not None:
                t2, end_q = result
                t.extend(grab_t)
                return [end_q, relative_grasp, t, t2]
        return None

    def get_openable_traj(self, obj, obj_conf, end_conf, start_q, relative_grasp, knob, minForce, action):
        print("Openable Traj", end_conf.conf, obj_conf.conf)
        diff = np.array(end_conf.conf) - np.array(obj_conf.conf)
        if knob == 'knob' or 'top' in knob:
            total = diff[0]
        else:
            total = diff[1]
        if total == 0:
            return None
        for _ in range(5):
            increment, sample = util.get_increment(obj, obj_conf.conf, total, knob)
            print('INCREMENT', increment, 'SAMPLE', sample)
            t2 = self.open_class.open_obj(obj, start_q.conf, relative_grasp.pose, obj_conf.conf, increment, sample, knob, minForce, action)
            if t2 is not None:
                # t2 = cmds, end_conf
                cmds = [t2[0], t2[1]]
                return cmds
        return None
    
    def sample_grasp_openable(self, status):
        def func(obj, obj_conf, knob):
            old_pos = self.objects[obj].get_configuration()
            self.objects[obj].set_configuration(obj_conf.conf)
            new_obj_pose = self.objects[obj].link_from_name(knob).get_link_tform(True)
            for _ in range(20):
                grasp_pose, q = self.grasp.grasp(obj+status, new_obj_pose)
                if q is not None and self.robot.arm.IsCollisionFree(q, obstacles=[self.floor, self.objects[obj]]):
                    # Grasp in object frame
                    relative_grasp = np.dot(np.linalg.inv(new_obj_pose), grasp_pose)
                    cmd1 = [vobj.Pose(obj, relative_grasp)]
                    self.objects[obj].set_configuration(old_pos)
                    return cmd1
            self.objects[obj].set_configuration(old_pos)
            return None

        return func

    def sampleGrabPose(self, obj, obj_pose):
        # grasp_pose is grasp in world frame
        new_obj_pose = vobj.Pose(obj, obj_pose.pose)
        for _ in range(20):
            grasp_pose, q = self.grasp.grasp(obj, new_obj_pose.pose)
            if q is not None and self.robot.arm.IsCollisionFree(q, obstacles=[self.floor, self.objects[obj]]):
                # Grasp in object frame
                relative_grasp = np.dot(np.linalg.inv(new_obj_pose.pose), grasp_pose)
                cmd1 = [vobj.Pose(obj, relative_grasp)]
                return cmd1
        return None

    def compute_nonplaceable_IK(self, obj, obj_conf, grasp, knob, action):
        old_pos = self.objects[obj].get_configuration()
        self.objects[obj].set_configuration(obj_conf.conf)
        obj_pose = self.objects[obj].link_from_name(knob).get_link_tform(True)
        self.objects[obj].set_configuration(old_pos)
        grasp_in_world = np.dot(obj_pose, grasp.pose)
        q_g = self.robot.arm.ComputeIK(grasp_in_world)
        if q_g is None or not self.robot.arm.IsCollisionFree(q_g, obstacles=[self.floor, self.objects[obj]]):
            print("None q_q", q_g)
            return None
        up = np.array([[1, 0, 0, 0],
                        [0, 1, 0, 0],
                        [0, 0, 1, -.08],
                        [0., 0., 0., 1.]])          
        new_g = np.dot(grasp_in_world, up)
        translated_q = self.robot.arm.ComputeIK(new_g, seed_q=q_g)
        if translated_q is None:
            print("None translated q", translated_q)
            return None
        translated_q = np.array(translated_q)
        q_g = np.array(q_g)
        q1 = vobj.BodyConf(self.robot, translated_q)
        q2 = vobj.BodyConf(self.robot, q_g)
        traj = vobj.TrajPath(self.robot, [translated_q, q_g])
        hand_cmd = vobj.HandCmd(self.robot, self.objects[obj], grasp.pose, status='M')
        cmd = [q1, q2, [traj, hand_cmd]]
        return cmd

    def computeIK(self, obj, obj_pose, grasp):
        """
        :param obj: string of object name
        :param obj_pose: Pose Object
        :param grasp: Relative grasp in object frame
        :return: start and end configuration and trajectory
        """
        grasp_in_world = np.dot(obj_pose.pose, grasp.pose)
        q_g = self.robot.arm.ComputeIK(grasp_in_world)
        if q_g is None or not self.robot.arm.IsCollisionFree(q_g, obstacles=[self.floor]):
            print('q_g None', obj_pose.pose)
            return None
        up = np.array([[1, 0, 0, 0],
                          [0, 1, 0, 0],
                          [0, 0, 1, -.05],
                          [0., 0., 0., 1.]])
        new_g = np.dot(grasp_in_world, up)
        translated_q = self.robot.arm.ComputeIK(new_g, seed_q=q_g)
        if translated_q is None:
            print('translated none', obj_pose.pose, grasp.pose)
            return None
        translated_q = np.array(translated_q)
        q_g = np.array(q_g)
        q = vobj.BodyConf(self.robot, translated_q)
        traj = vobj.TrajPath(self.robot, [translated_q, q_g])
        hand_cmd = vobj.HandCmd(self.robot, self.objects[obj], grasp.pose, 'RB')
        traj2 = vobj.TrajPath(self.robot, [q_g, translated_q])
        cmd = [q, [traj, hand_cmd, traj2]]
        return cmd

    def samplePlacePose(self, obj, region):
        # Obj pose in world frame
        place_pose = self.place.samplePlacePose(obj, region)
        cmd = [vobj.Pose(obj, place_pose)]
        return cmd
    
    def samplePlaceCabinetPose(self, obj, cabinet, region, conf):
        print("Sample Place", conf.conf)
        place_pose = self.place.samplePlacePose(obj, region, conf.conf)
        cmd = [vobj.Pose(obj, place_pose)]
        return cmd

    def collisionCheck(self, obj, pos, other, other_pos):
        """
        Return True if collision free.
        :param obj: object to be checked (string)
        :param pos: position of object
        :param other: other object (string)
        :param other_pos: other object position
        :return: True or False
        """
        print(obj, pos)
        print(other, other_pos)
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
                self.objects[obj].set_configuration(pos.conf)
            else:
                self.objects[obj].set_transform(pos.pose)
            
            if other in self.openable:
                self.objects[other].set_configuration(other_pos.conf)
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
        if obj in self.openable:
            obj_oldpos = self.objects[obj].get_configuration()
            self.objects[obj].set_configuration(pose.conf)
            conf = True
            print('cfree', pose.conf)
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
    
    def testCollisionAndReplan(self, cmds, objects):
        objs = []
        for obj in objects:
            objs.append(self.objects[obj])
        for i in range(len(cmds)):
            traj = cmds[i]
            if isinstance(traj, vobj.TrajPath):
                collision = False
                traj = traj.path
                for num in range(len(traj) - 1):
                    if not util.collision_Test(self.robot, self.objects, objs, traj[num], traj[num + 1], 75):
                        collision = True
                        break
                if collision:
                    print("Collision detected!")
                    start = vobj.BodyConf(self.robot, traj[0])
                    end = vobj.BodyConf(self.robot, traj[-1])
                    new_traj = self.calculate_path(start, end, nonmovable=objs)
                    if new_traj is None:
                        return False, None
                    cmds[i] = new_traj[0][0]
        return True, cmds

    def cfreeTrajHolding_Check(self, traj, obj, grasp, obj2, pose):
        old_pos = self.objects[obj].get_transform()
        self.robot.arm.Grab(self.objects[obj], grasp.pose, 'RB')
        result = self.cfreeTraj_Check(traj, obj2, pose)
        self.robot.arm.Release(self.objects[obj])
        self.objects[obj].set_transform(old_pos)
        print('cfreeHolding', obj, obj2, result)
        return result

