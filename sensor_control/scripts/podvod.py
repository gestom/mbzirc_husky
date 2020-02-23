#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
import time
import random

fn = "../maps/arena/wps.txt"
pointArray = []

def callback(msg):
    global pointArray
    print(msg.point.x, msg.point.y)
    pointArray.append([msg.point.x, msg.point.y])

    if len(pointArray) > 4:
        del pointArray[0]
        print("Additional points, removing oldest")

    if len(pointArray) > 3:
        print("Writing file: " + fn)
        with open(fn, "w") as f:
            for i in pointArray:
                f.write(str(i[0]) + " " + str(i[1]) + "\n")


if __name__ == "__main__":
    rospy.init_node("podvod")

    pub = rospy.Publisher("/podvod", String, queue_size=10)
    rospy.Subscriber("/clicked_point", PointStamped, callback)
    #rospy.spin()
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
