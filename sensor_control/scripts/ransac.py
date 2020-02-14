import rospy 
import numpy as np
from sensor_msgs.msg import LaserScan
import laser_geometry.laser_geometry as lg
import sensor_msgs.point_cloud2 as pc2
import random
from visualization_msgs import msg as msgTemplate
from geometry_msgs.msg import Point

lp = lg.LaserProjection()
pub = None

iterations = 1000
tolerance = 0.03
nPointsA = 55
nPointsB = 55
maxDistBetweenPoints = 0.3

def scanCallback(msg):

    timestamp = msg.header.stamp
    frame = msg.header.frame_id

    msg = lp.projectLaser(msg)

    points = list(pc2.read_points(msg))
    if len(points) < 5:
        print("N Points error, returning")
        return

    for iteration in range(iterations):

        samplePointA = random.randint(0, len(points)-1)
        samplePointB = random.randint(0, len(points)-1)
        while samplePointA == samplePointB:
            samplePointB = random.randint(0, len(points)-1)

        samplePointA = points[samplePointA]
        samplePointB = points[samplePointB]

        modelLineDX = samplePointB[0] - samplePointA[0]
        modelLineDY = samplePointB[1] - samplePointA[1]

        lineNormalX = -modelLineDY
        lineNormalY = modelLineDX
        mag = ((lineNormalX**2) + (lineNormalY**2)) ** 0.5
        lineNormalX /= lineNormalX
        lineNormalY /= lineNormalY

        lineNormalX *= 0.3 #distance from line one and two
        lineNormalY *= 0.3 

        samplePointAB = [samplePointA[0], samplePointA[1]]
        samplePointAB[0] += lineNormalX
        samplePointAB[1] += lineNormalY
        samplePointBB = [samplePointB[0], samplePointB[1]]
        samplePointBB[0] += lineNormalX
        samplePointBB[1] += lineNormalY

        pointDistance = ((modelLineDX ** 2) + (modelLineDY ** 2)) ** 0.5

        allPoints = []
        lineA = []
        lineB = []
        for point in points:

            #twice triangle area
            tta = ((samplePointB[1] - samplePointA[1]) * point[0]) - ((samplePointB[0] - samplePointA[0]) * point[1]) + (samplePointB[0]*samplePointA[1]) - (samplePointB[1]*samplePointA[0])
            distanceToLine = abs(tta/pointDistance)
           

            ttaB = ((samplePointBB[1] - samplePointAB[1]) * point[0]) - ((samplePointBB[0] - samplePointAB[0]) * point[1]) + (samplePointBB[0]*samplePointAB[1]) - (samplePointBB[1]*samplePointAB[0])
            distanceToLineB = abs(ttaB/pointDistance)

            point = list(point)
            if distanceToLine < tolerance:
                #point.append(0)
                lineA.append(point)
                #allPoints.append(point)
            elif distanceToLineB < tolerance:
                #point.append(1)
                #allPoints.append(point)
                lineB.append(point)

        #break lines by gaps
        if len(lineA) > nPointsA and len(lineB) > nPointsB:
            print("Done")
            
            msg = msgTemplate.Marker()
            msg.header.frame_id = frame
            msg.header.stamp = timestamp
            msg.type = 7
            msg.action = 0
            msg.id = 0
            
            msg.scale.x = 0.2
            msg.scale.y = 0.2
            msg.scale.z = 0.2

            msg.color.a = 1
            msg.color.r = 0
            msg.color.g = 1
            msg.color.b = 0

            pubPoints = []
            for point in lineA:
                p = Point(x=point[0],y=point[1],z=0.3)
                pubPoints.append(p)

            msg.points = pubPoints
            pub.publish(msg)

            msg.id = 1
            msg.color.r = 1
            pubPoints = []

            for point in lineB:
                p = Point(x=point[0],y=point[1],z=0.3)
                pubPoints.append(p)

            msg.points = pubPoints
            pub.publish(msg)

            return
    print("Failed")

if __name__ == "__main__":

    rospy.init_node("ransac")
    pub = rospy.Publisher("/ransac/lines", msgTemplate.Marker, queue_size=5)
    pub = rospy.Publisher("/ransac/clusterer", msgTemplate.Marker, queue_size=5)
    rospy.Subscriber("/scan", LaserScan, scanCallback)
    rospy.spin()
