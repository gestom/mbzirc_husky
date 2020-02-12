#ifndef __DETECTOR_H__
#define __DETECTOR_H__

#include "ros/ros.h"
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
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sstream>
#include <cstdlib>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <opencv2/core.hpp>
#include <random>
#include <iterator>
#include <std_msgs/builtin_string.h>
#include <string>

#define LEN_MULTIPLIER (0.03271908282177614)    // SPECIFIC FOR VLP16
#define INV_SQRT_2PI (0.3989422804014327)

#define GET_CENTER(x1, x2) ((x1 + x2)/2)

typedef cv::Point3d MyPoint;
typedef std::array<MyPoint, 2> BrickLine;

class BrickDetector {

public:

    ros::NodeHandle *n;
    std::array<std::vector<MyPoint>, 4> wall_setup;
    std::array<std::vector<MyPoint>, 4> wall_setup_back;

    BrickDetector(ros::NodeHandle node) {
        n = &node;
        fetch_parameters();
        build_walls();
    }

    std::array<std::vector<BrickLine>, 5> find_brick_segments(std::vector<MyPoint> *filtered_ptcl);

    std::vector<BrickLine> find_wall_segments(std::vector<MyPoint> *filtered_ptcl);

    std::vector<BrickLine> match_detections(std::vector<BrickLine> lines,
                                            double obj_height, double obj_top, double req_height, int min_detections,
                                            int max_detections);

    void filter_on_line(MyPoint p1, MyPoint p2, std::array<std::vector<BrickLine>, 4> &ret);

    std::array<MyPoint, 4> get_piles(std::array<std::vector<BrickLine>, 4> &lines);

    std::array<std::vector<MyPoint>, 4> get_centers_in_piles(std::array<std::vector<BrickLine>, 4> &lines,
                                                            std::array<MyPoint, 4> &piles);

    std::array<MyPoint, 2>
    fit_detection(std::array<std::vector<MyPoint>, 4> proposal, std::array<std::vector<MyPoint>, 4> wall, std::array<cv::Point2d, 2> tar_pt);

    void subscribe_ptcl(sensor_msgs::PointCloud2 ptcl);


private:

    // parameters
    // 910-911 pts per one plane of VLP-16 lidar
    int LIDAR_ROWS;                 // VLP-16
    float HEIGHT_TOLERANCE;         // for creating ground plane
    float GROUND_SAFE_DISTANCE;     // minimal distance from ground
    float LIDAR_HEIGHT;             // height of the lidar from ground
    float CLUSTERING_DIST;          // maximal distance used for clustering
    float WALL_CLUSTERING_DIST;     // maximal distance for clastering bigger wall segments
    int MIN_CLUSTER_SIZE;           // minimal number of points in one cluster
    int POINT_ITERATION_STEP;       // step during splitting
    float SPLITTING_DISTANCE;       // minimal distance to split into more clusters
    float WALL_SPLITTING_DISTANCE;  // minimal distance to split into more clusters
    int MINIMAL_LINE_LENGTH;        // sets minimal number of points to line
    float MAXIMAL_JOIN_ANGLE;       // maximal join angle in degrees
    float MAXIMAL_JOIN_DISTANCE;    // maximal distance to join sections
    float MAXIMAL_JOIN_SHIFT;       // maximal distance of first point from previous line
    float BRICK_SIZE;               // size of the smallest brick - we presume that all sizes are multiplication of this
    float SIZE_TOLERANCE;           // each brick should be size % BRICK_SIZE == 0
    float MAX_DIST;                 // maximal possible distance of a detection
    float MAX_BRICK_DIST;           // maximal possible distance of brick detection
    float MAX_HEIGHT;               // maximal possible height of a detection
    double BRICK_HEIGHT = 0.2;
    std::string PUBLISH_FRAME = "velodyne";      // frame in which you publish the results

    void fetch_parameters();

    double fetch_pointcloud(sensor_msgs::PointCloud2 &ptcl, std::vector<MyPoint> *ret);

    void
    filter_ground(std::vector<double> ground, std::vector<MyPoint> *point_rows, std::vector<MyPoint> *ret);

    void create_clusters(std::vector<MyPoint> *filtered,
                         std::vector<std::array<int, 2>> *ret,
                         double clustering_distance,
                         double max_dist);

    void split_and_merge(std::vector<MyPoint> *filtered,
                         std::vector<std::array<int, 2>> *clusters,
                         std::vector<BrickLine> *ret,
                         double splitting_distance);

    void filter_size(std::vector<BrickLine> *in_lines, double size, double tolerance,
                     std::vector<BrickLine> &ret);

    void build_walls();

};

void init_velodyne_subscriber(int argc, char **argv);

#endif