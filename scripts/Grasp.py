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

class Grasp:
    def __init__(self, robot, init_q, pose):
        self.init_q = init_q
        self.pose = pose
        self.robot = robot

    def grasp:
        newq = self.robot.arm.ComputeIK(self.pose)
        
        
    
        
    
