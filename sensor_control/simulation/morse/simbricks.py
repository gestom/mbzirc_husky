import random
import sys
sys.path.append('.')
sys.path.append('./data')
sys.path.append('./src')

#import husky
from morse.builder import *
from src.fire.builder.robots import Husky

husky = Husky()
#husky.translate(0, 0, 0)
#husky.add_default_interface('ros')

#add robot to scene
#robot = husky.createHusky()
#robot.translate(0, 0, 0)
#robot.rotate(0, 0, random.uniform(0, 1) - 0.5)

#add scene
#building = PassiveObject('../models/tower.blend','Object002')
#building.translate(x=random.uniform(15, 20), y=random.uniform(7, -9), z=0)

env = Environment('buildings_2', fastmode = True)
env.set_camera_location([-18.0, -6.7, 10.8])
env.set_camera_rotation([1.09, 0, -1.14])
env.show_framerate(True)

