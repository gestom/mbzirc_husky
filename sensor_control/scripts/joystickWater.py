#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
#from mbzirc_husky.msg import sprayActionGoal
import subprocess
import time

pub = None

signal = '\x62\x01\x92\xF5'
output = '/dev/ttyACM0'
state = "off"

def callback(data):
	global pub, state
	if data.buttons[6] or data.buttons[7] and state == "off":

            print("trig pressed firing water")
            with open(output, "w") as f:
                f.write(signal)
            state = "on"
            #time.sleep(0.04)
            #with open(output, "w") as f:
            #    f.write(signal)
            
            #print("pressed")
		#msg = sprayActionGoal()
                #print(msg)
		#msg.goal.duration = 30
                #print(msg)
		#pub.publish(msg)
        elif data.buttons[6] == 0 and data.buttons[7] == 0 and state == "on":
            state = "off"
            with open(output, "w") as f:
                f.write(signal)

if __name__ == '__main__':

	rospy.init_node('joystickWaterPump', anonymous=True)
	#pub = rospy.Publisher('/sprayServer/goal', sprayActionGoal, queue_size=1)
	rospy.Subscriber("/joy_teleop/joy", Joy, callback)
	rospy.spin()

        if state == "on":
            with open(output, "w") as f:
                f.write(signal)
