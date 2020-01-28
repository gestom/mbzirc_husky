#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from mbzirc_husky.msg import sprayActionGoal

pub = None

def callback(data):
	global pub
	if data.buttons[6] or data.buttons[7]:
                print("pressed")
		msg = sprayActionGoal()
                print(msg)
		msg.goal.duration = 30
                print(msg)
		pub.publish(msg)

if __name__ == '__main__':

	rospy.init_node('joystickWaterPump', anonymous=True)
	pub = rospy.Publisher('/sprayServer/goal', sprayActionGoal, queue_size=1)
	rospy.Subscriber("/joy_teleop/joy", Joy, callback)
	rospy.spin()
