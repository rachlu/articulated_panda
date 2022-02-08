from RRT import RRT
import numpy as np

rt = RRT()

print(rt.getDistance(np.array([0, 0, 1]), np.array([0,0,2])))
