import rospy
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

publisher = None
rotVel = 0.02

def callback(msg):

    msg = np.array(msg.data)
    msg = np.reshape(msg, (-1, 32))

    colHigh = []

    for col in msg[0]:
        colHigh.append(0)

    for rowIndex in range(len(msg)):
        for colIndex in range(len(msg[rowIndex])):
            if colHigh[colIndex] < msg[rowIndex][colIndex]:
                colHigh[colIndex] = msg[rowIndex][colIndex]

    peak = 0
    peakIdx = -1
    for i in range(len(colHigh)):
        if colHigh[i] > peak:
            peak = colHigh[i]
            peakIdx = i

    if peak < 100:
        print("no fire")
        return

    vel = 0
    if peakIdx < 14:
        print("left", peak)
        vel = -rotVel
    elif peakIdx < 18:
        print("centre", peak)
    else:
        print("right", peak)
        vel = rotVel

    msg = Twist()
    msg.linear.x = 0
    msg.linear.y = 0
    msg.linear.z = 0
    msg.angular.z = vel
    publisher.publish(msg)

if __name__ == "__main__":

    #minimum rotation speed for husky ~0.02 in z axis for cmd_vel
    rospy.init_node("fireController")
    publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=0) 
    rospy.Subscriber("/thermal/raw_temp_array", Float64MultiArray, callback)

    rospy.spin()
