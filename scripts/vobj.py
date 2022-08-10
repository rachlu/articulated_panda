import time


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

    def execute(self, *args):
        holding = False
        if args:
            obj, location, increment = args
            position = obj.get_configuration()
            holding = True
        for q in self.path:
            self.robot.arm.SetJointValues(q)
            if holding:
                if location is None:
                    obj.set_configuration(position)
                elif location.upper() == 'TOP':
                    obj.set_configuration(position)
                else:
                    obj.set_configuration(position)
                position += increment
            time.sleep(0.2)

    # def __str__(self):
    #    return str(self.path)


class HandCmd:
    def __init__(self, robot, obj, grasp=None, status=None):
        self.robot = robot
        self.obj = obj
        self.grasp = grasp
        self.status = status

    def execute(self):
        if self.status is not None:
            if self.status.upper() == 'OPEN':
                self.robot.arm.hand.Open()
            else:
                self.robot.arm.hand.Close()
            return
        if len(self.robot.arm.grabbedObjects) != 0:
            self.robot.arm.hand.Open()
            self.robot.arm.Release(self.obj)
        else:
            self.robot.arm.hand.Close()
            self.robot.arm.Grab(self.obj, self.grasp)

    def set_status(self, status):
        self.status = status

    def __repr__(self):
        return 't{}'.format(id(self) % 1000)

    # def __str__(self):
    #    result = 'Hand Cmd'
    #    return result


class Pose:
    def __init__(self, obj, pose):
        self.obj = obj
        self.pose = pose

    def __repr__(self):
        return 'p{}'.format(id(self) % 1000)
