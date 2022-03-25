#!/usr/bin/env python

import numpy as np
import random
import networkx as nx
import time


def getDistance(q1, q2):
    """
    Returns the total radian distance from configuration q1 to configuration q2.
    """
    total_distance = 0
    for index in range(len(q1)):
        total_distance += ((q1[index] - q2[index]) ** 2)
    return total_distance ** (1 / 2)


class RRT:
    def __init__(self, robot, max_step=2, max_time=10):
        self.robot = robot
        self.max_time = max_time
        self.max_step = max_step
        self.G = nx.DiGraph()

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

    # Second Test Completed
    def collisionFree(self, q1, q2, sample):
        """
        Returns [boolean, configuration]
        Returns True if the entire path from configuration q1 to configuration q2 is collison free.
        If false, returns a new configuration where the path from q1 to q_new is conllison free.
        """
        for num in range(sample):
            if not self.robot.arm.IsCollisionFree(q1 + (q2 - q1) / sample * (num + 1)):
                if num >= 1:
                    q_new = q1 + (q2 - q1)/sample * num
                    return [False, q_new]
                else:
                    return [False, None]
        return [True, q2]

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
                q_rand = self.robot.arm.randomConfiguration()

                if not self.robot.arm.IsCollisionFree(q_rand):
                    continue
                node_closest = self.closest_node(q_rand)
                if getDistance(q_rand, self.G.nodes[node_closest]['config']) > self.max_step:
                    q_rand = self.max_step / np.linalg.norm(q_rand - self.G.nodes[node_closest]['config']) * (
                                q_rand - self.G.nodes[node_closest]['config'])

            result = self.collisionFree(self.G.nodes[node_closest]['config'], q_rand, 5)
            if not result[0] or result[1] is None:
                continue
            if result[1] is not None:
                q_rand = result[1]

            new_node = self.G.number_of_nodes()
            self.G.add_node(new_node, config=q_rand)
            self.G.add_edge(node_closest, new_node)

            if (q_rand == q_goal).all():
                self.G = nx.relabel_nodes(self.G, {new_node: 'q_goal'})
                path = [self.G.nodes['q_goal']['config']]
                predecessors = list(self.G.predecessors('q_goal'))

                while len(predecessors) != 0:
                    path.insert(0, self.G.nodes[predecessors[0]]['config'])
                    predecessors = list(self.G.predecessors(predecessors[0]))
                while time.time() - start < self.max_time:
                    if len(path) < 3:
                        break
                    n1 = random.randint(0, len(path) - 2)
                    n2 = random.randint(n1 + 1, len(path) - 1)

                    result = self.collisionFree(path[n1], path[n2], 10)
                    if result[0]:
                        prior_distance = 0
                        for n in range(len(path[n1+1:n2+1])):
                            prior_distance += getDistance(path[n-1], path[n])
                        if getDistance(path[n1], path[n2]) < prior_distance:
                            new_path = path[:n1+1]
                            new_path.extend(path[n2:])
                            path = new_path
                return path