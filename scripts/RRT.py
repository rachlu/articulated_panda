#!/usr/bin/env python

import numpy as np
import random
import networkx as nx
import time
import util
import math
from scipy.spatial.transform import Rotation as R

def getDistance(q1, q2):
    """
    Returns the total radian distance from configuration q1 to configuration q2.
    """
    x = q1 - q2
    return np.sqrt(x.dot(x))


def filter(arraylist):
    """
    :param arraylist: List of configurations
    :return: Returns a list of configurations with duplicates removed.
    """
    temp = {array.tostring(): array for array in arraylist}
    return list(temp.values())


def rotation_constraint(robot, q, obj):
    """
    :param robot: robot
    :param q: configuration of the robot
    :param obj: object the robot is holding.
    :return: true if the obj is upright given q configuration
    """
    old_q = robot.arm.GetJointValues()
    robot.arm.SetJointValues(q)
    obj_pose = obj.get_transform()
    # 3 x 3 matrix that depicts rotation of the object
    rotation_matrix = [obj_pose[0][:3], obj_pose[1][:3], obj_pose[2][:3]]
    r = R.from_matrix(rotation_matrix)
    angles = r.as_euler('xyz', degrees=True)

    if -15 <= angles[0] <= 15 and -15 <= angles[1] <= 15:
        robot.arm.SetJointValues(old_q)
        return True
    robot.arm.SetJointValues(old_q)
    return False
            

class RRT:
    # Test step size
    def __init__(self, robot, objects, nonmovable=None, max_step=0.5, max_time=5, max_shortcut=3, constraint=None):
        self.robot = robot
        self.max_time = max_time
        self.max_step = max_step
        self.G = nx.DiGraph()
        self.nonmovable = nonmovable
        self.max_shortcut = max_shortcut
        self.constraint = constraint
        self.objects = objects

    # Test Completed
    def closest_node(self, q):
        """
        Find the closest node on the Tree from configuration q.
        """
        node = None
        shortest_distance = 999999999999
        for n in self.G.nodes:
            new_distance = getDistance(q, self.G.nodes[n]['config'])
            if shortest_distance > new_distance:
                node = n
                shortest_distance = new_distance
        return node

    def collisionFree(self, q1, q2):
        """
        Returns list of collision-free configurations along q1 to q2 with given step-size.
        """
        sample = int(np.sqrt((q1 - q2).dot(q1 - q2)) / self.max_step)
        q_list = []
        num = 1
        while num < sample:
            q_new = q1 + (q2 - q1) / sample * num
            if util.collision_Test(self.robot, self.objects, self.nonmovable, q1, q_new, 50, self.constraint):
                q_list.append(q_new)
            else:
                return q_list
            q1 = q_new
            num += 1
            #print('collision_free', sample)
        if self.robot.arm.IsCollisionFree(q2, obstacles=self.nonmovable):
            q_list.append(q2)
        return q_list

    def sample_config(self):
        q_rand = self.robot.arm.randomConfiguration()
        
        while True:
            if self.robot.arm.IsCollisionFree(q_rand, obstacles=self.nonmovable):
                if self.constraint is not None:
                    constraint_function, obj = self.constraint
                    if constraint_function(self.robot, q_rand, self.objects[obj]):
                        break
                else:
                    break
            q_rand = self.robot.arm.randomConfiguration()
        
        '''
        while not self.robot.arm.IsCollisionFree(q_rand, obstacles=self.nonmovable):
            q_rand = self.robot.arm.randomConfiguration()
        '''
        #print(q_rand)
        return q_rand

    # Working?
    def motion(self, q_start, q_goal):
        self.G.clear()
        q_start = np.array(q_start)
        q_goal = np.array(q_goal)
        self.G.add_node('q_start', config=q_start)
        start = time.time()
        while time.time() - start < self.max_time:
            # if int((time.time() - start)) % 5 == 0:
            # print('Time', time.time()-start)
            if random.random() < 0.1:
                q_rand = q_goal
                node_closest = self.closest_node(q_rand)
            else:
                q_rand = self.sample_config()

                node_closest = self.closest_node(q_rand)

            q_list = self.collisionFree(self.G.nodes[node_closest]['config'], q_rand)
            for q in q_list:
                new_node = self.G.number_of_nodes()
                self.G.add_node(new_node, config=q)
                self.G.add_edge(node_closest, new_node)
                node_closest = self.closest_node(q)

            if not q_list:
                continue
            q_rand = q_list[-1]

            if (q_rand == q_goal).all():
                # print('found')
                self.G = nx.relabel_nodes(self.G, {new_node: 'q_goal'})
                path = [self.G.nodes['q_goal']['config']]
                predecessors = list(self.G.predecessors('q_goal'))
                print(self.G)

                while len(predecessors) != 0:
                    config = self.G.nodes[predecessors[0]]['config']
                    path.insert(0, config)
                    predecessors = list(self.G.predecessors(predecessors[0]))

                path = filter(path)
                start = time.time()
                while time.time() - start < self.max_shortcut:
                    #print('shortcutting', len(path))
                    if len(path) < 3:
                        break
                    n1 = random.randint(0, len(path) - 2)
                    n2 = random.randint(n1 + 1, len(path) - 1)

                    prior_distance = 0
                    for n in range(len(path[n1 + 1: n2 + 1])):
                        prior_distance += getDistance(path[n - 1], path[n])
                    if getDistance(path[n1], path[n2]) < prior_distance:
                        result = self.collisionFree(path[n1], path[n2])
                        if result and (result[-1] == path[n2]).all():
                            new_path = path[:n1 + 1]
                            new_path.extend(result)
                            new_path.extend(path[n2:])
                            path = new_path
                path = filter(path)
                print('path length:', len(path))
                return path
