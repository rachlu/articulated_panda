import os
import pb_robot
import numpy
import math
import util


def collision_free(objects, obj):
    """
    :param objects: List of objects in the scene
    :param obj: New object to be added
    :return: True if the obj is not in collision with any of the objects in objects and
    the new object is a set distance away from the other objects.
    """
    for obj2 in objects:
        if pb_robot.collisions.body_collision(obj2, obj):
            return False
        else:
            x = obj.get_transform()[0][-1]
            y = obj.get_transform()[1][-1]
            x2 = obj2.get_transform()[0][-1]
            y2 = obj2.get_transform()[1][-1]
            if math.sqrt((x-x2)**2+(y-y2)**2) < 0.15:
                return False
    return True


def cabinetIK(obj):
    def func(pose_worldF):
        z = pose_worldF[2][-1]
        name = None
        link = None
        for l in obj.links:
            if 'knob' not in l.get_link_name():
                continue
            position = l.get_link_tform(True)
            if abs(z - position[2][-1]) < 0.01:
                name = l.get_link_name()
                link = l
                break
        current = obj.get_configuration()
        position = link.get_link_tform(True)
        x = position[0][-1] - pose_worldF[0][-1]
        y = position[1][-1] - pose_worldF[1][-1]
        if abs(x) > abs(y):
            diff = x
        else:
            diff = y
        if 'top' in name:
            conf = current + numpy.array((diff, 0))
        elif 'bottom' in name:
            conf = current + numpy.array((0, diff))
        return conf
    return func


def execute():
    # Launch pybullet
    pb_robot.utils.connect(use_gui=True)
    pb_robot.utils.disable_real_time()
    pb_robot.utils.set_default_camera()
    pb_robot.viz.draw_pose(pb_robot.geometry.pose_from_tform(numpy.eye(4)), length=0.5, width=10)

    # Create robot object
    robot = pb_robot.panda.Panda()

    # Add floor object 
    objects_path = pb_robot.helper.getDirectory("YCB_Robot")
    floor_file = os.path.join(objects_path, 'short_floor.urdf')
    floor = pb_robot.body.createBody(floor_file)
    floor_pose = floor.get_transform()
    floor_pose[0][3] += 0.16764
    floor.set_transform(floor_pose)
    openable = ['door', 'cabinet']

    cabinet_file = os.path.join(objects_path, 'cabinet.urdf')
    cabinet = pb_robot.body.createBody(cabinet_file)
    pos = numpy.array([[1, 0, 0, 0.8],
                       [0, 1, 0, -0.3],
                       [0, 0, 1, pb_robot.placements.stable_z(cabinet, floor)],
                       [0, 0, 0, 1]])
    rotate = util.get_rotation_arr('Z', 2 * math.pi)
    cabinet.set_transform(numpy.dot(pos, rotate))
    cabinet.setIK(cabinetIK(cabinet))

    # Add fork object
    fork_file = os.path.join(objects_path, 'fork.urdf')
    fork = pb_robot.body.createBody(fork_file)

    while not collision_free([robot, cabinet], fork):
        random_pos = util.sampleTable('fork')[0].pose
        fork.set_transform(random_pos)
    
    # Add knife object
    knife_file = os.path.join(objects_path, 'knife.urdf')
    knife = pb_robot.body.createBody(knife_file)
    while not collision_free([fork, robot, cabinet], knife):
        random_pos = util.sampleTable('knife')[0].pose
        knife.set_transform(random_pos)

    # Add spoon object
    spoon_file = os.path.join(objects_path, 'spoon.urdf')
    spoon = pb_robot.body.createBody(spoon_file)
    while not collision_free([fork, knife, robot, cabinet], spoon):
        random_pos = util.sampleTable('spoon')[0].pose
        spoon.set_transform(random_pos)

    # Add bowl object
    bowl_file = os.path.join(objects_path, 'bowl.urdf')
    bowl = pb_robot.body.createBody(bowl_file)
    while not collision_free([knife, spoon, fork, robot, cabinet], bowl):
        random_pos = util.sampleTable('bowl')[0].pose
        bowl.set_transform(random_pos)

    # door_file = os.path.join(objects_path, 'door.urdf')
    #
    # door = pb_robot.body.createBody(door_file)
    # pos = numpy.array([[1, 0, 0, .6],
    #                   [0, 1, 0, -0.5],
    #                   [0, 0, 1, pb_robot.placements.stable_z(door, floor)],
    #                   [0, 0, 0, 1]])
    # rotate = util.get_rotation_arr('Z', math.pi/2)

    # rotate = util.get_rotation_arr('Z', 3*math.pi/2)
    # pos = numpy.array([[1, 0, 0, .4],
    #                   [0, 1, 0, 0.5],
    #                   [0, 0, 1, pb_robot.placements.stable_z(door, floor)],
    #                   [0, 0, 0, 1]])

    # rotate = util.get_rotation_arr('Z', math.pi)
    # pos = numpy.array([[1, 0, 0, .8],
    #                   [0, 1, 0, 0.5],
    #                   [0, 0, 1, pb_robot.placements.stable_z(door, floor)],
    #                   [0, 0, 0, 1]])

    # rotate = util.get_rotation_arr('Z', 0)
    # pos = numpy.array([[1, 0, 0, -0.4],
    #                   [0, 1, 0, 0.4],
    #                   [0, 0, 1, pb_robot.placements.stable_z(door, floor)],
    #                   [0, 0, 0, 1]])
    # door.set_transform(numpy.dot(pos, rotate))

    # spring_file = os.path.join(objects_path, 'block.urdf')
    # spring = pb_robot.body.createBody(spring_file)
    # pos = numpy.array([[1, 0, 0, 0.4],
    #                    [0, 1, 0, 0.4],
    #                    [0, 0, 1, pb_robot.placements.stable_z(spring, floor)],
    #                    [0, 0, 0, 1]])
    # spring.set_transform(pos)
    # objects = {'fork': fork, 'spoon': spoon, 'knife': knife, 'bowl': bowl, 'door': door}

    objects = {'fork': fork, 'spoon': spoon, 'knife': knife, 'bowl': bowl, 'cabinet': cabinet}

    # objects = {'door': door, 'cabinet': cabinet, 'spring':spring}
    # objects = {'cabinet': cabinet}
    return objects, openable, floor, robot
