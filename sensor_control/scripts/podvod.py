#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
import time
import random

pointArray = []

def callback(msg):
    global pointArray
    print(msg.point.x, msg.point.y)
    pointArray.append([msg.point.x, msg.point.y])

    if len(pointArray) > 3:
        del pointArray[0]

if __name__ == "__main__":
    rospy.init_node("podvod")

    pub = rospy.Publisher("/podvod", String, queue_size=10)
    rospy.Subscriber("/clicked_point", PointStamped, callback)
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():

        print("====")
        m = ""

        for i in range(len(pointArray)):
            
            m += str(pointArray[i][0]) + " " + str(pointArray[i][1]) + " "

        if m == "":
            time.sleep(1)
            continue

        print(m)
        ms = String()
        ms.data = m
        print(ms)
        pub.publish(ms)

        time.sleep(1)
