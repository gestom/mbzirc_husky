import rospy
from sensor_msgs.msg import PointCloud2
from visualization_msgs import msg as msgTemplate
import numpy as np
import tf
import sensor_msgs.point_cloud2 as pc2

def velodyneCallback(msg):

    transform = None
    transform = tfListener.lookupTransform("/map", "/velodyne", rospy.Time(0.))
    #try:
    #    transform = tfListener.lookupTransform("/map", "/velodyne", rospy.Time(0-0.5))
    #except:
    #    print("Aborting, unable to get tf transformation")
    #    return

    trans_mat = tf.transformations.translation_matrix(transform[0])
    rot_mat = tf.transformations.quaternion_matrix(transform[1])
    mat = np.dot(trans_mat, rot_mat)

    points = pc2.read_points(msg)
    mapPoints = []
    for point in points:
        
        transformedPoint = np.append(point[0:3], 1.0)
	transformedPoint = np.dot(mat, transformedPoint)
	point = np.append(transformedPoint[0:3], point[3:])
        mapPoints.append(point)
 
    m = msgTemplate.Marker()

    m.header.frame_id = "map";
    m.header.stamp = rospy.Time.now();

    m.type = 7
    m.action = 0

    m.scale.x = 0.2
    m.scale.y = 0.2
    m.scale.z = 0.2

    m.color.a = 1.0
    m.color.r = 0.0
    m.color.g = 1.0
    m.color.b = 0.5

    m.points = mapPoints

    pointsPublisher.publish(m)

if __name__ == "__main__":
    rospy.init_node("pnlistener")
    tfListener = tf.TransformListener()
    pointsPublisher = rospy.Publisher('/transformedPoints', msgTemplate.Marker, queue_size=1)
    rospy.Subscriber("/velodyne_points", PointCloud2, velodyneCallback)
    rospy.spin()
