import time
import util


class BodyConf:
    def __init__(self, body, q):
        self.body = body
        self.conf = q

    def __repr__(self):
        return 'q{}'.format(id(self) % 1000)


class TrajPath:
    def __init__(self, robot, path):
        self.robot = robot
        self.path = path

    def __repr__(self):
        return 't{}'.format(id(self) % 1000)

    def execute(self, arm):
        ans = 'Y'
        while ans.upper() == 'Y':
            for q in self.path:
                self.robot.arm.SetJointValues(q)
                time.sleep(0.2)
            ans = input('Redo?')
        if arm:
            path = util.convert(arm, self.path)
            arm.execute_position_path(path)


class HandCmd:
    def __init__(self, robot, obj, grasp=None, status=None):
        self.robot = robot
        self.obj = obj
        self.grasp = grasp
        self.status = status

    def execute(self, arm, *args):
        # if self.status is not None:
        #     if self.status.upper() == 'OPEN':
        #         self.robot.arm.hand.Open()
        #         input("Open")
        #         if arm:
        #             arm.hand.open()
        #     else:
        #         self.robot.arm.hand.Close()
        #         input("Close")
        #         if arm:
        #             arm.hand.grasp(0.02, 40, epsilon_inner=0.1, epsilon_outer=0.1)
        #     return
        if len(self.robot.arm.grabbedObjects) != 0:
            self.robot.arm.hand.Open()
            self.robot.arm.Release(self.obj)
        else:
            print('grabbed')
            self.robot.arm.hand.Close()
            self.robot.arm.Grab(self.obj, self.grasp, self.status)

    def set_status(self, status):
        self.status = status

    def __repr__(self):
        return 't{}'.format(id(self) % 1000)


class Pose:
    def __init__(self, obj, pose):
        self.obj = obj
        self.pose = pose

    def __repr__(self):
        return 'p{}'.format(id(self) % 1000)
