#!/usr/bin/env python

import numpy as np
import random
import networkx as nx
import time


def getDistance(q1, q2):
    """
    Returns the total radian distance from configuration q1 to configuration q2.
    """
    # total_distance = 0
    # for index in range(len(q1)):
    #     try:
    #         total_distance += ((q1[index] - q2[index]) ** 2)
    #     except:
    #         print('dis', q1, q2)
    #         return
    # return total_distance ** (1 / 2)
    x = q1-q2
    return np.sqrt(x.dot(x))


class RRT:
    # Test step size
    def __init__(self, robot, nonmovable = None, max_step=0.05, max_time=20):
        self.robot = robot
        self.max_time = max_time
        self.max_step = max_step
        self.G = nx.DiGraph()
        self.nonmovable = nonmovable

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
        q_list = []
        for num in range(1, sample):
            if self.robot.arm.IsCollisionFree(q1 + (q2 - q1) / sample * (num + 1), obstacles = self.nonmovable):
                q_new = q1 + (q2 - q1) / sample * num
                q_list.append(q_new)
            else:
                break
        if self.robot.arm.IsCollisionFree(q2, obstacles = self.nonmovable):
            q_list.append(q2)

        return q_list

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
            if random.random() < 0.1:
                q_rand = q_goal
                node_closest = self.closest_node(q_rand)
            else:
                q_rand = self.sample_config()

                node_closest = self.closest_node(q_rand)

            for q in self.collisionFree(self.G.nodes[node_closest]['config'], q_rand):
                new_node = self.G.number_of_nodes()
                self.G.add_node(new_node, config=q_rand)
                self.G.add_edge(node_closest, new_node)
                node_closest = self.closest_node(q)

            if (q_rand == q_goal).all():
                self.G = nx.relabel_nodes(self.G, {new_node: 'q_goal'})
                path = [self.G.nodes['q_goal']['config']]
                predecessors = list(self.G.predecessors('q_goal'))
                print(self.G)
                
                while len(predecessors) != 0:
                    path.insert(0, self.G.nodes[predecessors[0]]['config'])
                    predecessors = list(self.G.predecessors(predecessors[0]))
                # while time.time() - start < self.max_time:
                #     if len(path) < 3:
                #         break
                #     n1 = random.randint(0, len(path) - 2)
                #     n2 = random.randint(n1 + 1, len(path) - 1)
                #
                #     result = self.collisionFree(path[n1], path[n2])
                #     if result:
                #         prior_distance = 0
                #         for n in range(len(path[n1 + 1:n2 + 1])):
                #             prior_distance += getDistance(path[n - 1], path[n])
                #         if getDistance(path[n1], path[n2]) < prior_distance:
                #             new_path = path[:n1 + 1]
                #             new_path.extend(path[n2:])
                #             path = new_path

                return path
