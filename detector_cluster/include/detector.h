#ifndef __DETECTOR_H__
#define __DETECTOR_H__

#include "ros/ros.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>
#include <geometry_msgs/Point32.h>
#include <iostream>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <sstream>
#include <cstdlib>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <opencv2/core.hpp>

#define LEN_MULTIPLIER (0.03271908282177614)    // SPECIFIC FOR VLP16
#define INV_SQRT_2PI (0.3989422804014327)

#define GET_CENTER(x1, x2) ((x1 + x2)/2)

typedef cv::Point3f MyPoint;
typedef std::array<MyPoint, 2> BrickLine;

class BrickDetector {

public:

    ros::NodeHandle *n;

    BrickDetector(ros::NodeHandle node) {
        n = &node;
        fetch_parameters();
    }

    std::array<std::vector<BrickLine>, 4> find_bricks(sensor_msgs::PointCloud2 &ptcl);

    std::array<std::vector<BrickLine>, 4> match_detections(std::array<std::vector<BrickLine>, 4> lines);

    void filter_on_line(MyPoint p1, MyPoint p2, std::array<std::vector<BrickLine>, 4> &ret);

    std::array<std::array<double, 2>, 4> get_piles(std::array<std::vector<BrickLine>, 4> &lines);

    void subscribe_ptcl(sensor_msgs::PointCloud2 ptcl);


private:

    // parameters
    // 910-911 pts per one plane of VLP-16 lidar
    int LIDAR_ROWS;              // VLP-16
    float HEIGHT_TOLERANCE;      // for creating ground plane
    float GROUND_SAFE_DISTANCE;  // minimal distance from ground
    float LIDAR_HEIGHT;          // height of the lidar from ground
    float CLUSTERING_DIST;       // maximal distance used for clustering
    int MIN_CLUSTER_SIZE;        // minimal number of points in one cluster
    int POINT_ITERATION_STEP;    // step during splitting
    float SPLITTING_DISTANCE;    // minimal distance to split into more clusters
    int MINIMAL_LINE_LENGTH;     // sets minimal number of points to line
    float MAXIMAL_JOIN_ANGLE;    // maximal join angle in degrees
    float MAXIMAL_JOIN_DISTANCE; // maximal distance to join sections
    float MAXIMAL_JOIN_SHIFT;    // maximal distance of first point from previous line
    float BRICK_SIZE;            // size of the smallest brick - we presume that all sizes are multiplication of this
    float SIZE_TOLERANCE;        // each brick should be size % BRICK_SIZE == 0
    float MAX_DIST;              // maximal possible distance of a detection
    float MAX_HEIGHT;            // maximal possible height of a detection

    void fetch_parameters();

    double fetch_pointcloud(sensor_msgs::PointCloud2 &ptcl, std::vector<MyPoint> *ret);

    std::vector<double> get_plane(float min_z, std::vector<MyPoint> *point_rows);

    void
    filter_ground(std::vector<double> ground, std::vector<MyPoint> *point_rows, std::vector<MyPoint> *ret);

    void create_clusters(std::vector<MyPoint> *filtered,
                         std::vector<std::array<int, 2>> *ret);

    void split_and_merge(std::vector<MyPoint> *filtered,
                         std::vector<std::array<int, 2>> *clusters,
                         std::vector<BrickLine> *ret);

    void filter_size(std::vector<BrickLine> *in_lines, double size,
                     std::vector<BrickLine> &ret);

};

void init_velodyne_subscriber(int argc, char **argv);

#endif