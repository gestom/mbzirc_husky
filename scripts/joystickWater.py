#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from mbzirc_husky.msg import spray

pub = None

def callback(data):
	global pub
	print(data)
	if data.buttons[6] or data.buttons[7]:
		msg = spray
		spray.timeout = 0.2
		pub.publish(msg)

if __name__ == '__main__':

	rospy.init_node('joystickWaterPump', anonymous=True)
	pub = rospy.Publisher('/sprayServer/goal', spray, queue_size=1)
	rospy.Subscriber("/joy/feedback", Joy, callback)
	rospy.spin()
