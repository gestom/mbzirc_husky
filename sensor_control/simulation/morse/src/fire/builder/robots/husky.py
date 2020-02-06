from morse.builder import *

class Husky(GroundRobot):
    """
    A template robot model for husky, with a motion controller and a pose sensor.
    """
    def __init__(self, name = None, debug = True):

        # husky.blend is located in the data/robots directory
        GroundRobot.__init__(self, 'fire/robots/husky.blend', name)
        self.properties(classpath = "fire.robots.husky.Husky")

        ###################################
        # Actuators
        ###################################


        # (v,w) motion controller
        # Check here the other available actuators:
        # http://www.openrobots.org/morse/doc/stable/components_library.html#actuators
        self.motion = MotionVW()
        self.append(self.motion)
        #self.motion = MotionVWDiff()
        self.motion.add_interface("ros", topic='/cmd_vel')
        #self.append(self.motion)

        # Optionally allow to move the robot with the keyboard
        if debug:
            keyboard = Keyboard()
            keyboard.properties(ControlType = 'Position')
            self.append(keyboard)

        ###################################
        # Sensors
        ###################################

        self.pose = Pose()
        self.pose.add_interface("ros")
        self.append(self.pose)

        self.odometry = Odometry()
        self.odometry.add_interface("ros", topic="/odom")
        self.append(self.odometry)

        self.ls = Hokuyo()
        self.ls.translate(0, 0, 0.4)
        self.ls.add_interface("ros", topic='/scan')
        self.append(self.ls)
