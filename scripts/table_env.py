#!/usr/bin/env python

# from __future__ import print_function

import os
import IPython
import pb_robot
import numpy

import time
import pybullet
from RRT import RRT
from tsr.tsr import TSR
from Grasp import Grasp
from Place import Place

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

    objects = {'fork': fork, 'spoon': spoon, 'knife': knife, 'plate': plate}

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
        motion = RRT(robot)
        motion.execute(motion.motion(q_start, q_goal))
    grasp = Grasp(robot, objects)
    place = Place(robot, objects, floor)

    # pb_robot.viz.draw_tsr(place.place_tsr['knife'])
    # pb_robot.viz.draw_tsr(place.place_tsr['fork'])
    # pb_robot.viz.draw_tsr(place.place_tsr['spoon'])
    # pb_robot.viz.draw_tsr(place.place_tsr['plate'])

    def execute(obj):
        q_start = numpy.array(robot.arm.GetJointValues())
        grasp_pose, q_grasp = grasp.grasp(obj)
        place_pose = place.place_tsr[obj].sample()
        relative_grasp = numpy.dot(numpy.linalg.inv(objects[obj].get_transform()), grasp_pose)
        q_goal = robot.arm.ComputeIK(numpy.dot(place_pose, relative_grasp))

        while not robot.arm.IsCollisionFree(q_goal):
            print('not collision free')
            place_pose = place.place_tsr[obj].sample()

            relative_grasp = numpy.dot(numpy.linalg.inv(objects[obj].get_transform()), grasp_pose)
            q_goal = robot.arm.ComputeIK(numpy.dot(place_pose, relative_grasp))

        robot.arm.hand.Open()
        motion = RRT(robot)
        print('start', q_start)
        print('q_grasp', q_grasp)
        motion.execute(motion.motion(q_start, q_grasp))
        robot.arm.hand.Close()
        robot.arm.Grab(objects[obj], relative_grasp)
        input('continue')
        print('q_goal', q_goal)
        motion.execute(motion.motion(q_grasp, q_goal))
        robot.arm.Release(objects[obj])
        robot.arm.hand.Open()

    for obj in objects:
        print(obj)
        execute(obj)

    IPython.embed()

    # Close out Pybullet
    pb_robot.utils.wait_for_user()
    pb_robot.utils.disconnect()
