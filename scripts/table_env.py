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
    default = {}


    def set_default():
        for obj in objects:
            default[obj] = objects[obj].get_transform()
        default['robot'] = robot.arm.GetJointValues()
    set_default()

    # def test():
    #     q_start = robot.arm.randomConfiguration()
    #     while not robot.arm.IsCollisionFree(q_start):
    #         q_start = robot.arm.randomConfiguration()
    #     robot.arm.SetJointValues(q_start)
    #     pb_robot.viz.draw_pose(pb_robot.geometry.pose_from_tform(robot.arm.ComputeFK(q_start)), width=10, length=0.2)
    #     ans = input("Continue?(y/n)")
    #     if ans == 'n':
    #         return
    #     q_goal = robot.arm.randomConfiguration()
    #     while not robot.arm.IsCollisionFree(q_goal):
    #         q_goal = robot.arm.randomConfiguration()
    #     robot.arm.SetJointValues(q_goal)
    #     pb_robot.viz.draw_pose(pb_robot.geometry.pose_from_tform(robot.arm.ComputeFK(q_goal)), width=10, length=0.2)
    #     ans = input("Continue?(y/n)")
    #     if ans == 'n':
    #         return
    #     motion = RRT(robot)
    #     execute(motion.motion(q_start, q_goal))


    grasp = Grasp(robot, objects)
    place = Place(robot, objects, floor)

    # pb_robot.viz.draw_tsr(place.place_tsr['knife'])
    # pb_robot.viz.draw_tsr(place.place_tsr['fork'])
    # pb_robot.viz.draw_tsr(place.place_tsr['spoon'])
    # pb_robot.viz.draw_tsr(place.place_tsr['plate'])
    path = {}
    relative_grasps = {}


    def reset_env():
        for item in objects:
            objects[item].set_transform(default[item])
        robot.arm.SetJointValues(default['robot'])


    def plan_path(obj):
        moves = {}
        q_start = numpy.array(robot.arm.GetJointValues())

        # Pick
        robot.arm.hand.Open()
        motion = RRT(robot)
        new_path = None
        while new_path is None:
            grasp_pose, q_grasp = grasp.grasp(obj)
            new_path = motion.motion(q_start, q_grasp)
        execute_path(new_path)
        moves['pick'] = new_path
        robot.arm.hand.Close()

        # Place
        new_path = None
        while new_path is None:
            place_pose = place.place_tsr[obj].sample()
            relative_grasp = numpy.dot(numpy.linalg.inv(objects[obj].get_transform()), grasp_pose)
            relative_grasps[obj] = relative_grasp
            q_goal = robot.arm.ComputeIK(numpy.dot(place_pose, relative_grasps[obj]))
            while not robot.arm.IsCollisionFree(q_goal):
                place_pose = place.place_tsr[obj].sample()

                relative_grasps[obj] = numpy.dot(numpy.linalg.inv(objects[obj].get_transform()), grasp_pose)
                q_goal = robot.arm.ComputeIK(numpy.dot(place_pose, relative_grasps[obj]))
            new_path = motion.motion(q_grasp, q_goal)
        robot.arm.Grab(objects[obj], relative_grasps[obj])
        execute_path(new_path)
        moves['place'] = new_path
        robot.arm.Release(objects[obj])
        path[obj] = moves

    def execute_path(q_list):
        if q_list is None:
            print("No path")
            return
        # input("Execute path?")
        for q in q_list:
            robot.arm.SetJointValues(q)
            time.sleep(.1)

    def execute_path_all():
        for item in objects:
            for q in path[item]['pick']:
                robot.arm.SetJointValues(q)
                time.sleep(1)
            robot.arm.hand.Close()
            robot.arm.Grab(objects[item], relative_grasps[item])
            for q in path[item]['place']:
                robot.arm.SetJointValues(q)
                time.sleep(1)
            robot.arm.Release(objects[item])
            robot.arm.hand.Open()
        robot.arm.SetJointValues(default['robot'])


    def plan():
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, False)
        print('Calculating Path...')
        for obj in objects:
            print(obj)
            plan_path(obj)
        reset_env()
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, True)
        print("Done")

    IPython.embed()

    # Close out Pybullet
    pb_robot.utils.wait_for_user()
    pb_robot.utils.disconnect()
