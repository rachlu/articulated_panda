#!/usr/bin/env python

import numpy as np
import random
import networkx as nx
import time


def getDistance(q1, q2):
    """
    Returns the total radian distance from configuration q1 to configuration q2.
    """
    x = q1-q2
    return np.sqrt(x.dot(x))


class RRT:
    # Test step size
    def __init__(self, robot, nonmovable = None, max_step=0.5, max_time=10, max_shortcut=2):
        self.robot = robot
        self.max_time = max_time
        self.max_step = max_step
        self.G = nx.DiGraph()
        self.nonmovable = nonmovable
        self.max_shortcut = max_shortcut

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
        sample = int(np.sqrt((q1-q2).dot(q1-q2))/self.max_step)
        #print('sample', sample)
        q_list = []
        num = 1
        while num < sample:   
        #for num in range(1, sample+1):
            q_new = q1 + (q2 - q1) / sample * num
            #print(q_new)
            if self.collision_Test(q1, q_new, 5):
                #q_new = q1 + (q2 - q1) / sample * num
                q_list.append(q_new)
            else:
                return q_list
            q1 = q_new
            num += 1
        if self.robot.arm.IsCollisionFree(q2, obstacles = self.nonmovable):
            q_list.append(q2)

        return q_list

    def collision_Test(self, q1, q2, sample):
        for num in range(1, sample+1):
            if not self.robot.arm.IsCollisionFree(q1 + (q2 - q1) / sample * num, obstacles = self.nonmovable):
                return False
        return True

    def sample_config(self):
        q_rand = self.robot.arm.randomConfiguration()
        while not self.robot.arm.IsCollisionFree(q_rand, obstacles = self.nonmovable):
            q_rand = self.robot.arm.randomConfiguration()

        return q_rand

    # Working?
    def motion(self, q_start, q_goal):
        self.G.clear()
        q_start = np.array(q_start)
        q_goal = np.array(q_goal)
        self.G.add_node('q_start', config=q_start)
        start = time.time()
        while time.time() - start < self.max_time:
            #if int((time.time() - start)) % 5 == 0:
                #print('Time', time.time()-start)
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
                #print('found')
                self.G = nx.relabel_nodes(self.G, {new_node: 'q_goal'})
                path = [self.G.nodes['q_goal']['config']]
                predecessors = list(self.G.predecessors('q_goal'))
                print(self.G)
                
                while len(predecessors) != 0:
                    config = self.G.nodes[predecessors[0]]['config']
                    path.insert(0, config)
                    predecessors = list(self.G.predecessors(predecessors[0]))

                start = time.time()
                while time.time() - start < self.max_shortcut:
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

                print('path length:', len(path))
                return path
