#!/usr/bin/env python

# from __future__ import print_function

import os
import IPython
import pb_robot
import numpy
import math
import random
import time
from RRT import RRT
from tsr.tsr import TSR

if __name__ == '__main__':
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

    # Add fork object
    fork_file = os.path.join(objects_path, 'fork.urdf')
    fork = pb_robot.body.createBody(fork_file)
    fork_pose = numpy.array([[1, 0, 0, 0.3],
                             [0, 0, -1, 0.2],
                             [0, 1, 0, pb_robot.placements.stable_z(fork, floor) + 0.01],
                             [0., 0., 0., 1.]])
    fork.set_transform(fork_pose)

    # Add knife object
    knife_file = os.path.join(objects_path, 'knife.urdf')
    knife = pb_robot.body.createBody(knife_file)
    knife_pose = numpy.array([[1., 0., 0., -0.4],
                              [0., 0., -1., 0.4],
                              [0., 1., 0., pb_robot.placements.stable_z(knife, floor) + 0.01],
                              [0., 0., 0., 1.]])
    knife.set_transform(knife_pose)

    # Add spoon object
    spoon_file = os.path.join(objects_path, 'spoon.urdf')
    spoon = pb_robot.body.createBody(spoon_file)
    spoon_pose = numpy.array([[1, 0, 0, -0.3],
                              [0, 0, -1, -0.2],
                              [0, 1, 0, pb_robot.placements.stable_z(spoon, floor) + 0.01],
                              [0., 0., 0., 1.]])
    spoon.set_transform(spoon_pose)

    # Add plate object
    plate_file = os.path.join(objects_path, 'plate.urdf')
    plate = pb_robot.body.createBody(plate_file)
    plate_pose = numpy.array([[0., 0., 1., 0.8],
                              [0., 1., 0., 0],
                              [1., 0., 0., pb_robot.placements.stable_z(plate, floor)],
                              [0., 0., 0., 1.]])
    plate.set_transform(plate_pose)


    # pb_robot.viz.draw_pose(pb_robot.geometry.pose_from_tform(robot.arm.ComputeFK(robot.arm.GetJointValues())), width=10, length=0.2)


    def get_relative(world, pose):
        return numpy.dot(numpy.linalg.inv(pose), world)


    def test():
        q_start = robot.arm.randomConfiguration()
        while not robot.arm.IsCollisionFree(q_start):
            q_start = robot.arm.randomConfiguration()
        robot.arm.SetJointValues(q_start)
        pb_robot.viz.draw_pose(pb_robot.geometry.pose_from_tform(robot.arm.ComputeFK(q_start)), width=10, length=0.2)
        ans = input("Continue?(y/n)")
        if ans == 'n':
            return
        q_goal = robot.arm.randomConfiguration()
        while not robot.arm.IsCollisionFree(q_goal):
            q_goal = robot.arm.randomConfiguration()
        robot.arm.SetJointValues(q_goal)
        pb_robot.viz.draw_pose(pb_robot.geometry.pose_from_tform(robot.arm.ComputeFK(q_goal)), width=10, length=0.2)
        ans = input("Continue?(y/n)")
        if ans == 'n':
            return
        motion = RRT(robot, q_start, q_goal)
        motion.execute(motion.motion())


    relative = {}
    bw_range = {}

    def set_info():
        # plate
        t_o = plate.get_transform()
        t_ee = numpy.array([[math.cos(math.pi / 2), -math.sin(math.pi / 2), 0, .70],
                            [math.sin(math.pi / 2), math.cos(math.pi / 2), 0, 0],
                            [0, 0, 1, .18],
                            [0., 0., 0., 1.]])
        t_e = numpy.array([[1, 0, 0, 0],
                           [0, math.cos(math.pi), -math.sin(math.pi), 0],
                           [0, math.sin(math.pi), math.cos(math.pi), 0],
                           [0., 0., 0., 1.]])
        rel = get_relative(numpy.dot(t_ee, t_e), t_o)
        relative[plate] = rel

        bw = numpy.array([[], [], [], [], [], []])
        bw_range[plate] = bw

        # fork
        t_o = fork.get_transform()
        t_ee = numpy.array([[math.cos(math.pi), -math.sin(math.pi), 0, .2],
                            [math.sin(math.pi), math.cos(math.pi), 0, .2],
                            [0, 0, 1, .15],
                            [0., 0., 0., 1.]])
        rel = get_relative(numpy.dot(t_ee, t_e), t_o)
        relative[fork] = rel

        bw = numpy.array([[0.2, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0]])
        bw_range[fork] = bw

        # knife
        t_o = knife.get_transform()
        t_ee = numpy.array([[math.cos(math.pi), -math.sin(math.pi), 0, -.4],
                            [math.sin(math.pi), math.cos(math.pi), 0, .4],
                            [0, 0, 1, .15],
                            [0., 0., 0., 1.]])
        rel = get_relative(numpy.dot(t_ee, t_e), t_o)
        relative[knife] = rel

        bw = numpy.array([[-0.12, 0.12], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0]])
        bw_range[knife] = bw

        # spoon
        t_o = spoon.get_transform()
        t_ee = numpy.array([[math.cos(math.pi), -math.sin(math.pi), 0, -.2],
                            [math.sin(math.pi), math.cos(math.pi), 0, -.2],
                            [0, 0, 1, .15],
                            [0., 0., 0., 1.]])
        rel = get_relative(numpy.dot(t_ee, t_e), t_o)
        relative[spoon] = rel

        bw = numpy.array([[-0.2, 0.1], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0]])
        bw_range[spoon] = bw

    set_info()
    #grasp_tsr = TSR(knife.get_transform(), relative[knife], bw_range[knife])
    #pb_robot.viz.draw_tsr(grasp_tsr)

    def grasp(obj):
        # r,g,b = x,y,z
        q = numpy.array(robot.arm.GetJointValues())
        t_o = obj.get_transform()
        pose = numpy.dot(t_o, relative[obj])
        newq = numpy.array(robot.arm.ComputeIK(pose))
        robot.arm.SetJointValues(newq)
        motion = RRT(robot, q, newq)
        motion.execute(motion.motion())



    IPython.embed()

    # Close out Pybullet
    pb_robot.utils.wait_for_user()
    pb_robot.utils.disconnect()
