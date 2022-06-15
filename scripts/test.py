from RRT import RRT
import numpy
from Plan import Plan
import pb_robot
import table_env
import IPython
from Grasp import Grasp
import math
import vobj
from TAMP_Functions import TAMP_Functions
from Place import Place

if __name__ == '__main__':
    # pb_robot.utils.connect(use_gui=True)
    # pb_robot.utils.disable_real_time()
    # pb_robot.utils.set_default_camera()

    objects, floor, robot = table_env.execute()
    robot.arm.hand.Open()
    grasp = Grasp(robot, objects)
    rrt = RRT(robot)
    place = Place(robot, objects, floor)
    '''
    while True:
        q = grasp.grasp('fork')[1]
        robot.arm.SetJointValues(q)
        #robot.arm.hand.Close()
        print(robot.arm.IsCollisionFree(q))
        ans = input('next?')
        if ans.upper() == 'N':
            break
    '''
    '''
    while True:
        q_start = robot.arm.GetJointValues()
        print('start', q_start)
        q_end = rrt.sample_config()
        print('goal', q_end)
        path = rrt.motion(q_start, q_end)
        print(path)
        if path is None:
            continue
        input('execute?')
        p = vobj.TrajPath(robot, path)
        p.execute()
        ans = input('next')
        if ans.upper() == 'N':
            break
    '''
    '''
    while True:
        q_start = robot.arm.GetJointValues()
        q_goal = rrt.sample_config()
        path = rrt.motion(q_start, q_goal)
        print(path)
        if path is None:
            continue
        p = vobj.TrajPath(robot, path)
        p.execute()
        input('next')
    '''
    
    obj = 'fork'    
    grasp, q = grasp.grasp(obj)
    robot.arm.SetJointValues(q)
    grasp = numpy.dot(numpy.linalg.inv(objects[obj].get_transform()), grasp)
    robot.arm.Grab(objects[obj], grasp)
    robot.arm.hand.Close()
    #old_pos = objects['plate'].get_transform()
    while True:
        obj_pose = place.samplePlacePose(obj)
        world_grasp = numpy.dot(obj_pose, grasp)
        new_q = robot.arm.ComputeIK(world_grasp)
        if new_q is None:
            continue
        #rrt = RRT(robot, nonmovable=[floor])
        #q_start = robot.arm.GetJointValues()
        #path = vobj.TrajPath(robot, rrt.motion(q_start, new_q))
        #print(path.path)
        '''
        for num in range(len(path.path)):
            print((num+1), '/', len(path.path))
            print(robot.arm.IsCollisionFree(path.path[num]))
        path.execute()
        '''
        robot.arm.SetJointValues(new_q)
        print(robot.arm.IsCollisionFree(new_q))
        ans = input('next? (R?)')
        while ans.upper() == 'R':
            path.execute()
            ans = input('next? (R?)')
        if ans.upper() == 'N':
            break
    
    '''
    for obj in objects:
        obj_pose = place.samplePlacePose(obj)
        objects[obj].set_transform(obj_pose)
    '''
    IPython.embed()
    pb_robot.utils.wait_for_user()
    pb_robot.utils.disconnect()
