import math
import numpy

# Relative offset to valid position. Object in table frame
bowl_place = numpy.array([[1, 0, 0, 0],
                            [0, 1, 0, 0.4],
                            [0, 0, 1, .005],
                            [0., 0., 0., 1.]])

# Allowable range for placement
bowl_bw = numpy.array([[-0.01, 0.01], [0, 0], [0, 0], [0, 0], [0, 0], [-math.pi, math.pi]])

rotation = numpy.array([[math.cos(math.pi/2), -math.sin(math.pi/2), 0, 0],
                      [math.sin(math.pi/2), math.cos(math.pi/2), 0, 0],
                      [0, 0, 1, 0],
                      [0., 0., 0., 1.]])

# x, y, z
translation = numpy.array([[1, 0, 0, 0.2],
                            [0, 1, 0, 0.4],
                            [0, 0, 1, 0.005],
                            [0., 0., 0., 1.]])

fork_place = numpy.dot(translation, rotation)

translation = numpy.array([[1, 0, 0, -0.2],
                            [0, 1, 0, 0.4],
                            [0, 0, 1, 0.005],
                            [0., 0., 0., 1.]])
knife_place = numpy.dot(translation, rotation)

translation = numpy.array([[1, 0, 0, -0.33],
                            [0, 1, 0, 0.4],
                            [0, 0, 1, 0.005],
                            [0., 0., 0., 1.]])
spoon_place = numpy.dot(translation, rotation)

utensils_bw = numpy.array([[-0.02, 0.02], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0]])

