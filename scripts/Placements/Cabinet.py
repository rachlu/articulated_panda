import math
import numpy

# Relative offset to valid position. Object in table frame
translation = numpy.array([[1, 0, 0, 0.33],
                            [0, 1, 0, -0.3],
                            [0, 0, 1, 0.4],
                            [0., 0., 0., 1.]])
bowl_place = translation

# Allowable range for placement
bowl_bw = numpy.array([[0, 0], [-0.01, 0.01], [0, 0], [0, 0], [0, 0], [-math.pi, math.pi]])

# x, y, z
translation = numpy.array([[1, 0, 0, 0.33],
                            [0, 1, 0, -0.3],
                            [0, 0, 1, 0.4],
                            [0., 0., 0., 1.]])

fork_place = translation
# translation = numpy.array([[1, 0, 0, 0.5],
#                             [0, 1, 0, 0.2],
#                             [0, 0, 1, 0.005],
#                             [0., 0., 0., 1.]])
knife_place = translation

# translation = numpy.array([[1, 0, 0, 0.5],
#                             [0, 1, 0, .33],
#                             [0, 0, 1, 0.005],
#                             [0., 0., 0., 1.]])
spoon_place = translation

utensils_bw = numpy.array([[0, 0], [-0.01, 0.01], [0, 0], [0, 0], [0, 0], [-math.pi, math.pi]])

