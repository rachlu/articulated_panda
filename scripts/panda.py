import rospy
from franka_interface import ArmInterface
import table_env
from Grasp import Grasp
import numpy

if __name__ == '__main__':
    rospy.init_node('testing_node')
    arm = ArmInterface()
    objects, floor, _ = table_env.execute()

    grasp = Grasp(robot, objects)
    arm.hand.open()
    motion = RRT(robot)
    new_path = None
    q_start = arm.convertToList(arm.joint_angles)
    while new_path is None:
        grasp_pose, q_grasp = grasp.grasp('plate')
        relative_grasp = numpy.dot(numpy.linalg.inv(objects['plate'].get_transform()), grasp_pose)
        print(relative_grasp)
        grasp_in_world = numpy.dot(objects['plate'].get_transform(), relative_grasp)
        conf = robot.arm.ComputeIK(grasp_in_world)
        print(q_grasp, conf)
        new_path = motion.motion(q_start, conf)

    final_path = []
    for q in new_path:
        final_path.append(arm.convertToDict(q))
    print(final_path)
    # arm.execute_position_path(final_path)
>>>>>>> 4b440a135ac8cd4811294b6b629649b3dded08d7
