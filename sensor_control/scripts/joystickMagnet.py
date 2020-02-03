#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
#from mbzirc_husky.msg import sprayActionGoal
import subprocess
import time

pub = None

onSignal = '\x62\x01\x40\xA3'
offSignal = '\x62\x01\x41\xA4'
output = '/dev/ttyACM2'
state = "off"

def callback(data):
	global pub, state
	if data.buttons[6] or data.buttons[7] and state == "off":

            print("trig pressed firing magnet")
            with open(output, "w") as f:
                f.write(onSignal)
            state = "on"
        elif data.buttons[6] == 0 and data.buttons[7] == 0 and state == "on":
            state = "off"
            with open(output, "w") as f:
                f.write(offSignal)

if __name__ == '__main__':

	rospy.init_node('joystickMagnet', anonymous=True)
	rospy.Subscriber("/joy_teleop/joy", Joy, callback)
	rospy.spin()

        if state == "on":
            with open(output, "w") as f:
                f.write(offSignal)
