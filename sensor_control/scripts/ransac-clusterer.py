import tf
import rospy
from visualization_msgs import msg as msgTemplate
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import String
from geometry_msgs.msg import Point
import numpy as np

distanceFromCluster = 0.5
requiredObvs = 200

pub = None
pub_str = nNone
frame = None
ts = None
existingClusters = []
tfListener = None

def createNewCluster(x, y, distX, distY):
    global existingClusters
    cluster = {'x': x, 'y': y, 'distX': distX, 'distY': distY, 'totalX': x, 'totalY': y, 'observations': 1, 'totalDistX': distX, 'totalDistY': distY}
    existingClusters.append(cluster)

def resetCallback():
    global existingClusters
    print("Resetting ransac clusters")
    existingClusters = []

def appendToCluster(clusIdx, x, y, distX, distY):
    global existingClusters

    cluster = existingClusters[clusIdx]
    cluster["totalX"] += x
    cluster["totalY"] += y
    cluster["totalDistX"] += (distX)
    cluster["totalDistY"] += (distY)
    cluster["observations"] += 1 
    cluster["x"] = cluster["totalX"] / cluster["observations"]
    cluster["y"] = cluster["totalY"] / cluster["observations"]
    cluster["distX"] = cluster["totalDistX"] / cluster["observations"]
    cluster["distY"] = cluster["totalDistY"] / cluster["observations"]

    existingClusters[clusIdx] = cluster

def ransacCallback(msg):
   global existingClusters, frame, ts

   frame = msg.header.frame_id
   ts = msg.header.stamp

   points = list(pc2.read_points(msg))

   if len(points) < 2:
       return
   elif len(points) > 2:
       print("Bad number of POIs")
       return

   #if abs(points[0][0]) < 0.1 or abs(points[0][1]) < 0.1 or abs(points[1][0]) < 0.1 or abs(points[1][1]) < 0.1:
   #    print("Ignoring strangley small ransac points")
   #    print(points)
   #    return
   if points[0][0] > 999 or points[0][1] > 999 or points[1][0] > 999 or points[1][1] > 999:
       print("Bullshit values in ransac, ignoring")
       return

   #tf lookup
   transform = None
   try:
       transform = tfListener.lookupTransform('velodyne', 'map', rospy.Time(0))
   except:
       print("Aborting ransac, unable to get tf transform")
       return
   transformMat = tf.transformations.translation_matrix(transform[0])
   rotationMat = tf.transformations.quaternion_matrix(transform[1])
   mat = np.dot(transformMat, rotationMat)
   
   #points[0] = np.dot(mat, np.append(points[0][:2], [1.0, 1.0]))
   #points[1] = np.dot(mat, np.append(points[0][:2], [1.0, 1.0])) 

   #naively use closest as close point
   distToA = abs(((points[0][0]**2) + (points[0][1]**2)**0.5))
   distToB = abs(((points[1][0]**2) + (points[1][1]**2)**0.5))
  
   first = 0
   second = 1
   if distToB < distToA:
       first = 1
       second = 0
   
   addedToCluster = False 
   for clusIdx in range(len(existingClusters)):
       distX = points[first][0] - existingClusters[clusIdx]["x"]
       distY = points[first][1] - existingClusters[clusIdx]["y"]
       distToClust = (distX**2 + distY**2)**0.5

       if distToClust < distanceFromCluster:
           appendToCluster(clusIdx, points[first][0], points[first][1], points[second][0], points[second][1])
           addedToCluster = True
   if addedToCluster == False:
       createNewCluster(points[first][0], points[first][1], points[second][0], points[second][1])

def publishBestCluster():
    if len(existingClusters) == 0:
        return
    msg = msgTemplate.Marker()
    msg.header.frame_id = frame
    msg.header.stamp = ts
    msg.type = 7
    msg.action = 0
    msg.id = 0
    msg.scale.x = 0.5
    msg.scale.y = 0.5
    msg.scale.z = 0.5
    msg.color.a = 1
    msg.color.r = 1
    msg.color.g = 0
    msg.color.b = 1
    bestCluster = None
    bestClusterObvs = 0
    for i in existingClusters:
        if i["observations"]>bestClusterObvs:
            bestClusterObvs = i["observations"]
            bestCluster = i
    points = []
    print(bestCluster)
    if bestCluster["observations"] < requiredObvs:
        print("Not enough observations yet")
        return
    points.append(Point(x=bestCluster["x"], y=bestCluster["y"], z=0.3))
    points.append(Point(x=bestCluster["distX"], y=bestCluster["distY"], z=0.3))
    msg.points = points
    pub.publish(msg)
    pub_str.publish("%f %f %f %f", bestCluster["x"], bestCluster["y"], bestCluster["distX"], bestCluster["distY"])

if __name__ == "__main__":

    rospy.init_node("ransacClusterer")
    tfListener = tf.TransformListener()
    pub = rospy.Publisher("/ransac/clusterer", msgTemplate.Marker, queue_size=5)
    pub_str = rospy.Publisher("/ransac/clusterer_str", String, queue_size=5)
    rospy.Subscriber("/ransac/poi", PointCloud2, ransacCallback)
    rospy.Subscriber("/ransac/clusterer_reset", String, resetCallback)
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        publishBestCluster()
        rate.sleep()
