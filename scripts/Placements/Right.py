import numpy, math

# Relative offset to valid position. Object in table frame
bowl_place = numpy.array([[1, 0, 0, 0],
                            [0, 1, 0, 0.4],
                            [0, 0, 1, .005],
                            [0., 0., 0., 1.]])

# Allowable range for placement
bowl_bw = numpy.array([[-0.01, 0.01], [0, 0], [0, 0], [0, 0], [0, 0], [-math.pi, math.pi]])

t_ee = numpy.array([[1, 0, 0, 0],
                            [0, math.cos(math.pi/2), -math.sin(math.pi/2), 0],
                            [0, math.sin(math.pi/2), math.cos(math.pi/2), 0],
                            [0., 0., 0., 1.]])
t_e = numpy.array([[math.cos(3*math.pi/2), 0, math.sin(3*math.pi/2), 0],
                           [0, 1, 0, .3],
                           [-math.sin(3*math.pi/2), 0, math.cos(3*math.pi/2), 0],
                           [0., 0., 0., 1.]])

# x, y, z
translation = numpy.array([[1, 0, 0, 0.2],
                            [0, 1, 0, 0.4],
                            [0, 0, 1, -0.2794],
                            [0., 0., 0., 1.]])
rotation = numpy.dot(t_ee, t_e)

fork_place = numpy.dot(translation, rotation)

translation = numpy.array([[1, 0, 0, -0.2],
                            [0, 1, 0, 0.38],
                            [0, 0, 1, -0.2794],
                            [0., 0., 0., 1.]])
knife_place = numpy.dot(translation, rotation)

translation = numpy.array([[1, 0, 0, -0.33],
                            [0, 1, 0, 0.4],
                            [0, 0, 1, -0.2794],
                            [0., 0., 0., 1.]])
spoon_place = numpy.dot(translation, rotation)

utensils_bw = numpy.array([[-0.02, 0.02], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0]])

