#!/usr/bin/env python

#from __future__ import print_function

import os
import IPython
import pb_robot
import numpy as np
import random
import networkx as nx
import time
import matplotlib.pyplot as py

class RRT:
    def __init__(self, robot, q_start = None, q_goal = None, max_time = 10):
        self.G = nx.DiGraph()
        self.robot = robot
        self.q_start = q_start
        self.q_goal = q_goal
        self.G.add_node('q_start', config = self.q_start)
        self.max_time = max_time

    #Test Completed
    def closest_node(self, q):
        node = None
        shortest_distance = 999999999999
        for n in self.G.nodes:
            new_distance = self.getDistance(q, self.G.nodes[n]['config'])
            if shortest_distance > new_distance:
                node = n
                shortest_distance = new_distance
        return node

    #Second Test Completed
    def collisionFree(self, q1, q2, sample):
        for num in range(sample):
            if not self.robot.arm.IsCollisionFree(q1+(q2-q1)/sample*(num+1)):
                if num > 1:
                    return [False, q1+(q2-q1)/sample*(num)]
                else:
                    return [False, None]     
        return [True, None]

    #Test Completed
    def getDistance(self, q1, q2):
        total_distance = 0
        for index in range(len(q1)):
            total_distance += ((q1[index]-q2[index])**2)
        return total_distance**(1/2)

    #Working?
    def motion(self):
        print("running")
        self.robot.arm.SetJointValues(self.q_start)

        start = time.time()
        while time.time()-start < self.max_time:
            q_rand = self.robot.arm.randomConfiguration()
            if (self.G.number_of_nodes()) % 10 == 0:
                q_rand = self.q_goal
            if not self.robot.arm.IsCollisionFree(q_rand):
                continue
            node_closest = self.closest_node(q_rand)
            result = self.collisionFree(self.G.nodes[node_closest]['config'], q_rand, 10)
            if not result[0] and result[1] is None:
                continue
            elif not result[0] and result[1] is not None:
                q_rand = result[1]
            
            new_node = self.G.number_of_nodes()
            self.G.add_node(new_node, config = q_rand)
            self.G.add_edge(node_closest, new_node, weight = self.getDistance(self.G.nodes[node_closest]['config'], q_rand))
            
            if (q_rand == self.q_goal).all():
                #self.G.nodes[new_node]['config'] = 'q_goal'
                nx.draw(self.G, with_labels=True)
                py.show()
                path = [q_rand]
                predecessors = list(self.G.predecessors(new_node))

                while len(predecessors) != 0:
                    path.insert(0, self.G.nodes[predecessors[0]]['config'])
                    predecessors = list(self.G.predecessors(predecessors[0]))

                return path

    #Untested
    def execute(self, q_list):
        for q in q_list:
            self.robot.arm.SetJointValues(q)
            time.sleep(1)
        
    
        
    
