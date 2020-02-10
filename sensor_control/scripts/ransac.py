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

iterations = 250
tolerance = 0.05
nPointsA = 50
nPointsB = 30
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

        for point in points:

            #twice triangle area
            tta = ((samplePointB[1] - samplePointA[1]) * point[0]) - ((samplePointB[0] - samplePointA[0]) * point[1]) + (samplePointB[0]*samplePointA[1]) - (samplePointB[1]*samplePointA[0])
            distanceToLine = abs(tta/pointDistance)
           

            ttaB = ((samplePointBB[1] - samplePointAB[1]) * point[0]) - ((samplePointBB[0] - samplePointAB[0]) * point[1]) + (samplePointBB[0]*samplePointAB[1]) - (samplePointBB[1]*samplePointAB[0])
            distanceToLineB = abs(ttaB/pointDistance)

            point = list(point)
            if distanceToLine < tolerance:
                point.append(0)
                allPoints.append(point)
            elif distanceToLineB < tolerance:
                point.append(1)
                allPoints.append(point)

        #break lines by gaps
        sections = []
        currentSection = []
        for i in range(len(allPoints)-1):
             cp = allPoints[i]
             np = allPoints[i]
             print(cp, np)
             distanceToNext = ((np[0] - cp[0])**2 + (np[1] - cp[1])**2) ** 0.5
             currentSection.append(allPoints[i])
             if distanceToNext > maxDistBetweenPoints:
                 sections.append(currentSection)
                 currentSection = []
        sections.append(currentSection)
        sections = filter(None, sections)
          
        #get biggest section
        biggestSectionIdx = -1
        biggestSectionSize = 0
        for i in range(len(sections)):
            if len(sections[i]) > biggestSectionSize:
                biggestSectionIdx = i
                biggestSectionSize = len(sections[i])
        section = sections[biggestSectionIdx]

        inliersA = []
        inliersB = []
        print(section)
        for i in section:
            if i[-1] == 0:
                inliersA.append(i)
            if i[-1] == 1:
                inliersB.append(i)

        if len(inliersA) > nPointsA and len(inliersB) > nPointsB:
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
            for point in inliersA:
                p = Point(x=point[0],y=point[1],z=0.3)
                pubPoints.append(p)

            msg.points = pubPoints
            pub.publish(msg)

            msg.id = 1
            msg.color.r = 1
            pubPoints = []

            for point in inliersB:
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
