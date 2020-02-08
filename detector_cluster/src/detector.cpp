
#include "detector.h"

using namespace std;

// variables
ros::Publisher vis_pub;
ros::Publisher center_pub;

double normal_pdf(double x, double m, double s)
{
    double a = (x - m) / s;
    return INV_SQRT_2PI / s * std::exp(-0.5f * a * a);
}

double diff_angle(double a1, double a2) {
    auto c1 = std::cos(a1);
    auto s1 = std::sin(a1);

    auto c2 = std::cos(a2);
    auto s2 = std::sin(a2);

    auto dot = c1 * c2 + s1 * s2;
    auto cross = c1 * s2 - s1 * c2;

    return std::atan2(cross, dot);
}

array<array<double, 2>, 4> BrickDetector::get_piles(array<vector<BrickLine>, 4> &lines){

    array<array<double, 2>, 4> ret{};

    double var = 1.4;
    for(int clr_idx = 0; clr_idx < 4; clr_idx++){
        if (lines[clr_idx].size() > 1) {

            vector<array<double, 2>> probs;
            for (int i = 0; i < lines[clr_idx].size(); i++){
                array<double, 2> prob = {1.0, 1.0};
                probs.push_back(prob);
            }

            double probs_sum_x = lines[clr_idx].size();
            double probs_sum_y = lines[clr_idx].size();
            double mean_x = 0;
            double mean_y = 0;
            double sum_x = 0;
            double sum_y = 0;

            for (int iter = 0; iter < 25; iter++){

                sum_x = 0;
                sum_y = 0;
                for (int i = 0; i < lines[clr_idx].size(); i++) {
                    double center_x = GET_CENTER(lines[clr_idx][i][0].x, lines[clr_idx][i][1].x);
                    double center_y = GET_CENTER(lines[clr_idx][i][0].y, lines[clr_idx][i][1].y);
                    sum_x += center_x * probs[i][0];
                    sum_y += (center_y) * probs[i][1];
                }

                mean_x = sum_x / probs_sum_x;
                mean_y = sum_y / probs_sum_y;

                probs_sum_x = 0;
                probs_sum_y = 0;
                for (int i = 0; i < lines[clr_idx].size(); i++) {
                    double center_x = GET_CENTER(lines[clr_idx][i][0].x, lines[clr_idx][i][1].x);
                    double center_y = GET_CENTER(lines[clr_idx][i][0].y, lines[clr_idx][i][1].y);
                    double prob_x = normal_pdf(center_x, mean_x, var);
                    probs_sum_x += prob_x;
                    double prob_y = normal_pdf(center_y, mean_y, var);
                    probs_sum_y += prob_y;
                    probs[i] = {prob_x, prob_y};
                }

            }

            ret[clr_idx] = {mean_x, mean_y};

        }
    }

    return ret;

}

array<vector<BrickLine>, 4> BrickDetector::match_detections(array<vector<BrickLine>, 4> lines) {

    std::array<std::vector<BrickLine>, 4> ret;
    for (int clr_idx = 0; clr_idx < 4; clr_idx++){

        std::sort(lines[clr_idx].begin(), lines[clr_idx].end(), [](const BrickLine &l1, const BrickLine &l2) {
            return l1[0].z < l2[1].z;
        });

        for (int i = 0; i < lines[clr_idx].size(); i++){
            MyPoint curr_center = GET_CENTER(lines[clr_idx][i][0], lines[clr_idx][i][1]);
            int hits = 0;
            double curr_yaw = atan2(curr_center.y, curr_center.x) * 180 / M_PI;
            double curr_dist = norm(curr_center);
            double expected_z_dist = curr_dist*LEN_MULTIPLIER;
            int expected_hits = int((0.4/expected_z_dist) - 1);                                  // should be 40 heigh
            double expected_yaw = abs(acos((0.3*0.3) / (2*curr_dist*curr_dist) - 1));     // 20 cm is ok yaw diff
            if (expected_hits > 1) {
                for (int j = 0; j < lines[clr_idx].size(); j++) {
                    if (i != j) {
                        MyPoint new_center = GET_CENTER(lines[clr_idx][j][0], lines[clr_idx][j][1]);
                        double new_yaw = atan2(new_center.y, new_center.x) * 180 / M_PI;
                        double yaw_diff = abs(diff_angle(curr_yaw, new_yaw));
                        if (yaw_diff < expected_yaw) {
                            double new_dist = norm(new_center);
                            if (abs(new_dist - curr_dist) < 0.5) {
                                if (abs(curr_center.z - new_center.z) > expected_z_dist * 0.7) {
                                    hits++;
                                    if (curr_center.z + LIDAR_HEIGHT > 0.45) {        // 45cm is too high
                                        hits = 0;
                                    }
                                }
                            }
                        }
                    }
                }
            }
            if (hits >= expected_hits){
                ret[clr_idx].push_back(lines[clr_idx][i]);
            }
        }
    }
    return ret;
}


void BrickDetector::filter_on_line(MyPoint p1, MyPoint p2, array<vector<BrickLine>, 4> &ret){

    // compute the line
    array<double, 3> line = {-(p1.y - p2.y), p1.x - p2.x, 0};    // line in general form
    line[2] = -(p1.x * line[0] + p1.y * line[1]);

    double line_size = sqrt(line[0] * line[0] + line[1] * line[1]);

    for(int color_idx = 0; color_idx < 4; color_idx++){
        for (int i = ret[color_idx].size() - 1; i >= 0; i--){
            MyPoint pt1 = ret[color_idx][i][0];
            MyPoint pt2 = ret[color_idx][i][1];
            double dist = abs(pt1.x * line[0] + pt1.y * line[1] + line[2]) / line_size;
            double angle1 = atan2(pt1.y - pt2.y, pt1.x - pt2.x);
            double angle2 = atan2(p1.y - p2.y, p1.x - p2.x);
            double diff = abs(diff_angle(angle1, angle2) * 180 / M_PI);
            double max_diff = max(diff, abs(diff - 180));
            if (dist > 1.0 and max_diff > 30.0) {   // delete everything further than one meter from line and with bigger than 30 degree deviation
                ret[color_idx].erase(ret[color_idx].begin() + i);
            }
        }
    }
}

void BrickDetector::filter_size(vector<BrickLine> *in_lines, double size,
                                vector<BrickLine> &ret) {
    for (int row = 0; row < LIDAR_ROWS; row++) {
        for (auto &line : in_lines[row]) {
            float dx = line[0].x - line[1].x;
            float dy = line[0].y - line[1].y;
            float dist = sqrt(dx * dx + dy * dy);
            float diff = abs(dist - size);
            if (diff < SIZE_TOLERANCE) {
                ret.push_back(line);
            }
        }
    }
}

void BrickDetector::split_and_merge(vector<MyPoint> *filtered,
                                    vector<array<int, 2>> *clusters,
                                    vector<BrickLine> *ret) {

    for (int row = 0; row < LIDAR_ROWS; row++) {
        while (not clusters[row].empty()) {

            array<int, 2> cluster_idxs = clusters[row].back();
            clusters[row].pop_back();

            MyPoint p1 = filtered[row][cluster_idxs[0]];
            MyPoint p2 = filtered[row][cluster_idxs[1]];

            // compute the line
            array<double, 3> line = {-(p1.y - p2.y), p1.x - p2.x, 0};    // line in general form
            line[2] = -(p1.x * line[0] + p1.y * line[1]);

            double line_size = sqrt(line[0] * line[0] + line[1] * line[1]);

            // find most distant point
            double max_dist = 0;
            int split_index;
            double dist;
            for (int i = cluster_idxs[0]; i <= cluster_idxs[1]; i += POINT_ITERATION_STEP) {
                dist = abs(filtered[row][i].x * line[0] +
                           filtered[row][i].y * line[1] + line[2]) / line_size;

                if (dist > max_dist) {
                    max_dist = dist;
                    split_index = i;
                }
            }

            // split using most distant point and append to clusters or results
            if (max_dist > SPLITTING_DISTANCE) {
                if (split_index - cluster_idxs[0] > MINIMAL_LINE_LENGTH) {
                    array<int, 2> new_cluster = {cluster_idxs[0], split_index};
                    clusters[row].push_back(new_cluster);
                }
                if (cluster_idxs[1] - split_index > MINIMAL_LINE_LENGTH) {
                    array<int, 2> new_cluster = {split_index, cluster_idxs[1]};
                    clusters[row].push_back(new_cluster);
                }
            } else {
                BrickLine tmp = {p1, p2};
                ret[row].push_back(tmp);
            }
        }

        /*
        // connect all colinear lines - this doesnt really improve the result
        for (int row = 0; row < LIDAR_ROWS; row++) {
            for (int i = ret[row].size() - 1; i > 0; i--) {

                // compute distance
                vector<float> vec = {ret[row][i][0].x - ret[row][i - 1][1].x,
                                     ret[row][i][0].y - ret[row][i - 1][1].y,
                                     ret[row][i][0].z - ret[row][i - 1][1].z};
                float dist = sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
                if (dist > MAXIMAL_JOIN_DISTANCE) {
                    continue;
                }

                // compute angle
                vector<float> line1 = {ret[row][i][0].x - ret[row][i][1].x,
                                       ret[row][i][0].y - ret[row][i][1].y,
                                       ret[row][i][0].z - ret[row][i][1].z};
                vector<float> line2 = {ret[row][i - 1][0].x - ret[row][i - 1][1].x,
                                       ret[row][i - 1][0].y - ret[row][i - 1][1].y,
                                       ret[row][i - 1][0].z - ret[row][i - 1][1].z};
                float size1 = sqrt(line1[0] * line1[0] + line1[1] * line1[1] + line1[2] * line1[2]);
                float size2 = sqrt(line2[0] * line2[0] + line2[1] * line2[1] + line2[2] * line2[2]);
                float nom = line1[0] * line2[0] + line1[1] * line2[1] + line1[2] * line2[2];
                float denum = (size1 * size2);
                float angle_deg = acos(nom / denum) * 180 / M_PI;

                if (angle_deg > MAXIMAL_JOIN_ANGLE) {
                    continue;
                }

                // compute distance from line
                array<float, 3> line = {-line1[1], line1[0], 0};
                line[2] = -(ret[row][i][0].x * line[0] +
                            ret[row][i][1].y * line[1]);
                float line_size = sqrt(line[0] * line[0] + line[1] * line[1]);
                dist = abs(ret[row][i][1].x * line[0] +
                           ret[row][i][1].y * line[1] + line[2]) / line_size;

                if (dist > MAXIMAL_JOIN_SHIFT) {
                    continue;
                }

                BrickLine tmp = {ret[row][i - 1][0], ret[row][i][1]};
                ret[row].erase(ret[row].begin() + i);
                ret[row].erase(ret[row].begin() + i - 1);
                ret[row].insert(ret[row].begin() + i - 1, tmp);

            }
        }
        */
    }
}


void BrickDetector::create_clusters(vector<MyPoint> *filtered,
                                    vector<array<int, 2>> *ret) {
    float dist;
    float d_x;
    float d_y;
    for (int row = 0; row < LIDAR_ROWS; row++) {
        int start = 0;
        int end;
        for (end = 1; end < filtered[row].size(); end++) {
            d_x = filtered[row][end - 1].x - filtered[row][end].x;
            d_y = filtered[row][end - 1].y - filtered[row][end].y;
            dist = sqrt(d_x * d_x + d_y * d_y);
            if (dist > CLUSTERING_DIST) {
                if ((end - 1) - start > MIN_CLUSTER_SIZE) {
                    array<int, 2> tmp = {start, end - 1};
                    ret[row].push_back(tmp);
                }
                start = end;
            }
        }

        end--;
        if (dist < CLUSTERING_DIST and end - start > MIN_CLUSTER_SIZE) {
            array<int, 2> tmp = {start, end};
            ret[row].push_back(tmp);
        }

    }
}

void
BrickDetector::filter_ground(vector<double> ground, vector<MyPoint> *point_rows, vector<MyPoint> *ret) {
    // remove ground points and points too high

    for (int row = 0; row < LIDAR_ROWS; row++) {
        for (int i = 0; point_rows[row].size() > i; i++) {
            double dist = (ground[0] * point_rows[row][i].x) +
                          (ground[1] * point_rows[row][i].y) +
                          (ground[2] * point_rows[row][i].z) + ground[3];

            if (dist > GROUND_SAFE_DISTANCE and dist < MAX_HEIGHT) {   // 2 meters high is too much
                ret[row].push_back(point_rows[row][i]);
            }
        }
    }
}


vector<double> BrickDetector::get_plane(float min_z, vector<MyPoint> *point_rows) {

    // filter
    vector<MyPoint> ground_points;
    for (int row = 0; row < 2; row++) { // only four bottom rows
        for (int i = 0; i < point_rows[row].size(); i++) {
            if (point_rows[row][i].z <
                min_z + HEIGHT_TOLERANCE) { // height filter
                ground_points.push_back(point_rows[0][i]);
            }
        }
    }

    // sometimes occurs measurement with nothing inside
    if (ground_points.size() > 10) {

        // cast to eigen matrix
        Eigen::MatrixXf points_vec(3, ground_points.size());
        for (int i = 0; i < ground_points.size(); i++) {
            points_vec(0, i) = ground_points[i].x;
            points_vec(1, i) = ground_points[i].y;
            points_vec(2, i) = ground_points[i].z;
        }

        Eigen::Matrix<float, 3, 1> mean = points_vec.rowwise().mean();
        const Eigen::Matrix3Xf centered_points = points_vec.colwise() - mean;
        int setting = Eigen::ComputeFullU | Eigen::ComputeThinV;
        Eigen::JacobiSVD<Eigen::Matrix3Xf> svd =
                centered_points.jacobiSvd(setting);
        Eigen::Vector3f normal = svd.matrixU().col(2);


        double d = -(normal[0] * mean[0] + normal[1] * mean[1] + normal[2] * mean[2]);
        vector<double> ret;
        if (normal[2] > 0) {    // set c value of plane aways positive
            ret.insert(ret.end(), {normal[0], normal[1], normal[2], d});
        } else {
            ret.insert(ret.end(), {-normal[0], -normal[1], -normal[2], -d});
        }
        return ret;
    }

    vector<double> ret;
    return ret;
}


double BrickDetector::fetch_pointcloud(sensor_msgs::PointCloud2 &ptcl, vector<MyPoint> *ret) {
    double min_z = numeric_limits<double>::infinity(); // some really high number

    sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_ring((ptcl), "ring");
    sensor_msgs::PointCloud2ConstIterator<float> iter_x((ptcl), "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y((ptcl), "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z((ptcl), "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_ring, ++iter_y, ++iter_z) {
        /*
        if (*iter_z < min_z and *iter_ring < 2) {
            if (sqrt((*iter_x)*(*iter_x) + (*iter_y)*(*iter_y)) < MAX_DIST) { // only close points
                min_z = *iter_z; // point with lowest height
            }
        }
        */
        if (sqrt((*iter_x) * (*iter_x) + (*iter_y) * (*iter_y)) < MAX_DIST) {
            ret[*iter_ring].emplace_back(*iter_x, *iter_y, *iter_z);
        }

    }
    // when parsing velodyne_packets the points can be in incorrect order - sorting by yaw
    for (size_t row = 0; row < LIDAR_ROWS; row++) {
        std::sort(ret[row].begin(), ret[row].end(), [](const MyPoint &pt1, const MyPoint &pt2) {
            double angle1 = atan2(pt1.y, pt1.x);
            double angle2 = atan2(pt2.y, pt2.x);
            return diff_angle(angle1, angle2) < 0;
        });
    }

    return min_z;
}

void BrickDetector::fetch_parameters() {
    // get parameters
    n->param("/detector/lidar_rows", LIDAR_ROWS, 16);
    n->param("/detector/height_tolerance", HEIGHT_TOLERANCE, float(0.25));
    n->param("/detector/ground_safe_distance", GROUND_SAFE_DISTANCE, float(0.05));
    n->param("/detector/lidar_height", LIDAR_HEIGHT, float(0.4));
    n->param("/detector/clustering_dist", CLUSTERING_DIST, float(0.06));
    n->param("/detector/min_cluster_size", MIN_CLUSTER_SIZE, 10);
    n->param("/detector/point_iteration_step", POINT_ITERATION_STEP, 1);
    n->param("/detector/splitting_distance", SPLITTING_DISTANCE, float(0.1));
    n->param("/detector/minimal_line_length", MINIMAL_LINE_LENGTH, 5);
    n->param("/detector/maximal_join_angle", MAXIMAL_JOIN_ANGLE, float(3));
    n->param("/detector/maximal_join_distamce", MAXIMAL_JOIN_DISTANCE, float(0.1));
    n->param("/detector/maximal_join_shift", MAXIMAL_JOIN_SHIFT, float(0.02));
    n->param("/detector/brick_size", BRICK_SIZE, float(0.3));
    n->param("/detector/size_tolerance", SIZE_TOLERANCE, float(0.07));
    n->param("/detector/max_dist", MAX_DIST, float(5));
    n->param("/detector/max_height", MAX_HEIGHT, float(2));
}

array<vector<BrickLine>, 4> BrickDetector::find_bricks(sensor_msgs::PointCloud2 &ptcl) {

    // fill the row buffers and find point with the lowest height
    vector<MyPoint> point_rows[LIDAR_ROWS];
    double min_z = fetch_pointcloud(ptcl, point_rows);
    // vector<double> ground = get_plane(min_z, point_rows);    // get ground plane
    std::vector<double> ground = {0, 0, 1, LIDAR_HEIGHT};

    // if (not ground.empty()) { // did we manage to obtain ground in this measurement?

    // cout << "groud plane: " << ground[0] << " " << ground[1] << " " << ground[2] << " " << ground[3] << endl;

    vector<MyPoint> filtered[LIDAR_ROWS];
    filter_ground(ground, point_rows, filtered);        // filter ground points and project points to ground

    vector<array<int, 2>> clusters[LIDAR_ROWS];
    create_clusters(filtered, clusters);                // group points into the clusters

    vector<BrickLine> lines[LIDAR_ROWS];
    split_and_merge(filtered, clusters, lines);         // create lines using split-and-merge algorithm

    vector<BrickLine> red_lines;          // get red lines
    filter_size(lines, 0.3, red_lines);

    vector<BrickLine> green_lines;        // get green lines
    filter_size(lines, 0.6, green_lines);

    vector<BrickLine> blue_lines;         // get blue lines
    filter_size(lines, 1.2, blue_lines);

    vector<BrickLine> orange_lines;       // get orange lines
    filter_size(lines, 1.8, orange_lines);

    cout << "red detected: " << red_lines.size() << endl << "green detected: " << green_lines.size() << endl
         << "blue detected: " << blue_lines.size() << endl << "orange detected: " << orange_lines.size() << endl
         << endl;

    array<vector<BrickLine>, 4> ret = {red_lines, green_lines, blue_lines, orange_lines};
    return ret;
}

void BrickDetector::subscribe_ptcl(sensor_msgs::PointCloud2 ptcl) // callback
{

    array<vector<BrickLine>, 4> all_lines = find_bricks(ptcl);

    // MyPoint my_pt1(2.37, -2.45, 0.0);
    // MyPoint my_pt2(4.1, 1.89, 0);
    // filter_on_line(my_pt1, my_pt2, all_lines);

    array<vector<BrickLine>, 4> matched_lines = match_detections(all_lines);

    array<array<double, 2>, 4> pile_centers = get_piles(matched_lines);

    /// visualise using marker publishing in rviz --------------------------------------------------------------
    visualization_msgs::MarkerArray lists;
    visualization_msgs::Marker line_list1;
    line_list1.header.frame_id = "velodyne";
    line_list1.header.stamp = ros::Time::now();
    line_list1.ns = "points_and_lines";
    line_list1.action = visualization_msgs::Marker::ADD;
    line_list1.pose.orientation.w = 1.0;
    line_list1.id = 0;
    line_list1.type = visualization_msgs::Marker::LINE_LIST;
    line_list1.scale.x = 0.025;
    line_list1.color.a = 1.0;
    line_list1.color.r = 1.0;
    for (auto &red_line : matched_lines[0]) {
        geometry_msgs::Point p1;
        geometry_msgs::Point p2;

        p1.x = red_line[0].x;
        p1.y = red_line[0].y;
        p1.z = red_line[0].z;
        line_list1.points.push_back(p1);

        p2.x = red_line[1].x;
        p2.y = red_line[1].y;
        p2.z = red_line[1].z;
        line_list1.points.push_back(p2);
    }

    // green
    visualization_msgs::Marker line_list2;
    line_list2.header.frame_id = "velodyne";
    line_list2.header.stamp = ros::Time::now();
    line_list2.ns = "points_and_lines";
    line_list2.action = visualization_msgs::Marker::ADD;
    line_list2.pose.orientation.w = 1.0;
    line_list2.id = 1;
    line_list2.type = visualization_msgs::Marker::LINE_LIST;
    line_list2.scale.x = 0.025;
    line_list2.color.a = 1.0;
    line_list2.color.g = 1.0;
    for (auto &green_line : matched_lines[1]) {
        geometry_msgs::Point p1;
        geometry_msgs::Point p2;

        p1.x = green_line[0].x;
        p1.y = green_line[0].y;
        p1.z = green_line[0].z;
        line_list2.points.push_back(p1);

        p2.x = green_line[1].x;
        p2.y = green_line[1].y;
        p2.z = green_line[1].z;
        line_list2.points.push_back(p2);
    }

    // blue
    visualization_msgs::Marker line_list3;
    line_list3.header.frame_id = "velodyne";
    line_list3.header.stamp = ros::Time::now();
    line_list3.ns = "points_and_lines";
    line_list3.action = visualization_msgs::Marker::ADD;
    line_list3.pose.orientation.w = 1.0;
    line_list3.id = 2;
    line_list3.type = visualization_msgs::Marker::LINE_LIST;
    line_list3.scale.x = 0.025;
    line_list3.color.a = 1.0;
    line_list3.color.b = 1.0;
    for (auto &line : matched_lines[2]) {
        geometry_msgs::Point p1;
        geometry_msgs::Point p2;

        p1.x = line[0].x;
        p1.y = line[0].y;
        p1.z = line[0].z;
        line_list3.points.push_back(p1);

        p2.x = line[1].x;
        p2.y = line[1].y;
        p2.z = line[1].z;
        line_list3.points.push_back(p2);
    }

    // orange
    visualization_msgs::Marker line_list4;
    line_list4.header.frame_id = "velodyne";
    line_list4.header.stamp = ros::Time::now();
    line_list4.ns = "points_and_lines";
    line_list4.action = visualization_msgs::Marker::ADD;
    line_list4.pose.orientation.w = 1.0;
    line_list4.id = 3;
    line_list4.type = visualization_msgs::Marker::LINE_LIST;
    line_list4.scale.x = 0.025;
    line_list4.color.a = 1.0;
    line_list4.color.g = 1.0;
    line_list4.color.r = 1.0;
    for (auto &line : matched_lines[3]) {
        geometry_msgs::Point p1;
        geometry_msgs::Point p2;

        p1.x = line[0].x;
        p1.y = line[0].y;
        p1.z = line[0].z;
        line_list4.points.push_back(p1);

        p2.x = line[1].x;
        p2.y = line[1].y;
        p2.z = line[1].z;
        line_list4.points.push_back(p2);
    }


    lists.markers.push_back(line_list1);
    lists.markers.push_back(line_list2);
    lists.markers.push_back(line_list3);
    lists.markers.push_back(line_list4);

    visualization_msgs::Marker centers;
    centers.header.frame_id = "velodyne";
    centers.header.stamp = ros::Time::now();
    centers.ns = "pile_centers";
    centers.action = visualization_msgs::Marker::ADD;
    centers.pose.orientation.w = 1.0;
    centers.id = 0;
    centers.type = visualization_msgs::Marker::SPHERE_LIST;
    centers.scale.x = 0.25;
    centers.scale.y = 0.25;
    centers.scale.z = 0.25;
    centers.color.a = 1.0;
    centers.color.g = 1.0;

    for(int i = 0; i < 4; i++){
        if (pile_centers[i][0] != 0 and pile_centers[i][1] != 0){
            geometry_msgs::Point pt;
            cout << pile_centers[i][0] << " : " << pile_centers[i][1] << endl;
            pt.x = pile_centers[i][0];
            pt.y = pile_centers[i][1];
            pt.z = 0.0;
            centers.points.push_back(pt);
        }
    }

    center_pub.publish(centers);
    vis_pub.publish(lists);

    /// --------------------------------------------------------------------------------------
}

void init_velodyne_subscriber(int argc, char **argv) {
    ros::init(argc, argv, "detector");
    ros::NodeHandle n;
    n = ros::NodeHandle("~");

    BrickDetector detector(n);

    vis_pub = n.advertise<visualization_msgs::MarkerArray>("/visualization_marker", 5);
    center_pub = n.advertise<visualization_msgs::Marker>("/pile_centers", 5);
    ros::Subscriber sub = n.subscribe("/velodyne_points", 5, &BrickDetector::subscribe_ptcl, &detector);
    ros::spin();
}

int main(int argc, char **argv) {
    init_velodyne_subscriber(argc, argv);
    return 0;
}
