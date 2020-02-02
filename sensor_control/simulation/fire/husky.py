from morse.builder import *

def createHusky():

    robot = Morsy()

    ls = Hokuyo()
    ls.translate(0, 0, 0.9)
    ls.add_interface('ros')
    robot.append(ls)

    robot.add_default_interface("ros")

    return robot
