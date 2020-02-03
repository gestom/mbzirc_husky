#!/usr/bin/env python
import rospy
import math
from dynamic_reconfigure.server import Server
from mbzirc_husky.cfg import boundsConfig
from visualization_msgs import msg as msgTemplate
from geometry_msgs.msg import Point

width = 30
height = 20
rot = 0
x = 0
y = 0

def callback(config, level):
    global width, height, rot, x, y
    width = config.w
    height = config.h
    rot = config.r
    x = config.x
    y = config.y
    return config

if __name__ == "__main__":
    rospy.init_node("bounds")
    srv = Server(boundsConfig, callback)

    boundsVisPub = rospy.Publisher('/boundsVis', msgTemplate.Marker, queue_size=0)

    rate = rospy.Rate(2)

    while not rospy.is_shutdown():

        m = msgTemplate.Marker()
        m.header.frame_id = 'map'
        m.header.stamp = rospy.Time.now()

        m.type = 4
        m.action = 0

        m.pose.position.x = x 
        m.pose.position.y = y 
        m.pose.position.z = 0.1 

        m.pose.orientation.x = 0
        m.pose.orientation.y = 0
        m.pose.orientation.z = math.sin(rot/2.)
        m.pose.orientation.w = math.cos(rot/2.)

        m.scale.x = 0.2

        m.color.a = 1
        m.color.r = 1
        m.color.g = 1
        m.color.b = 0

        points = []

        z = 0
        points.append(Point(x = width/2+2, y = height/2, z=z))
        points.append(Point(x = -width/2, y = height/2, z=z))
        points.append(Point(x = -width/2, y = -height/2, z=z))
        points.append(Point(x = width/2, y = -height/2, z=z))
        points.append(Point(x = width/2, y = height/2, z=z))

        m.points = points

        boundsVisPub.publish(m)

        rate.sleep()
